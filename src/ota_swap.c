#include "ota_swap.h"

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "boards.h"
#include "nrfx_nvmc.h"

#define OTA_METADATA_MAGIC          0x3141544Fu /* "OTA1" */
#define OTA_METADATA_MAGIC_RECOVERY 0x5241544Fu /* "OTAR" */
#define OTA_METADATA_STATE_PENDING  0x444E4550u /* "PEND" */
#define OTA_METADATA_STATE_FAILED   0x4C494146u /* "FAIL" */
#define OTA_METADATA_VERSION        1u

#define OTA_FLASH_PAGE_SIZE 4096u
#define OTA_MAX_PAGE_COUNT  (BANK_SIZE / OTA_FLASH_PAGE_SIZE)
#define OTA_MAX_FAILURES    5u

#define OTA_PROGRESS_MAGIC  0x5054414Fu /* "OATP" */

typedef struct {
  uint32_t magic;
  uint16_t version;
  uint16_t headerSize;
  uint32_t state;
  uint32_t appAddr;
  uint32_t otaAddr;
  uint32_t imageBase;
  uint32_t imageSize;
  uint32_t imageCrc32;
  uint16_t pageSize;
  uint16_t pageCount;
  uint16_t framePayloadSize;
  uint16_t frameCount;
  uint16_t reserved0;
  uint32_t sequence;
  uint32_t pageCrc32[OTA_MAX_PAGE_COUNT];
  uint8_t pageState[OTA_MAX_PAGE_COUNT];
  uint8_t reserved1;
  uint16_t reserved2;
  uint32_t metadataCrc32;
} ota_metadata_t;

typedef struct {
  uint32_t magic;
  uint16_t pagesDone;
  uint8_t failures;
  uint8_t flags;
  uint32_t reserved0;
  uint32_t reserved1;
} ota_progress_record_t;

typedef struct {
  uint16_t pagesDone;
  uint8_t failures;
  uint8_t flags;
} ota_progress_t;

static uint8_t s_new_page[OTA_FLASH_PAGE_SIZE] __attribute__((aligned(4)));
static uint8_t s_backup_page[OTA_FLASH_PAGE_SIZE] __attribute__((aligned(4)));

static uint32_t crc32_update(uint32_t state, uint8_t const* data, uint32_t len) {
  static const uint32_t table[16] = {
    0x00000000u, 0x1db71064u, 0x3b6e20c8u, 0x26d930acu,
    0x76dc4190u, 0x6b6b51f4u, 0x4db26158u, 0x5005713cu,
    0xedb88320u, 0xf00f9344u, 0xd6d6a3e8u, 0xcb61b38cu,
    0x9b64c2b0u, 0x86d3d2d4u, 0xa00ae278u, 0xbdbdf21cu
  };

  for (uint32_t i = 0; i < len; ++i) {
    uint8_t tbl_idx = (uint8_t)(state ^ data[i]);
    state = table[tbl_idx & 0x0fu] ^ (state >> 4);
    tbl_idx = (uint8_t)(state ^ (data[i] >> 4));
    state = table[tbl_idx & 0x0fu] ^ (state >> 4);
  }
  return state;
}

static uint32_t crc32_calculate(uint8_t const* data, uint32_t len) {
  return ~crc32_update(0xFFFFFFFFu, data, len);
}

static uint32_t image_crc32(uint32_t address, uint32_t len) {
  return crc32_calculate((uint8_t const*)address, len);
}

static bool metadata_valid(ota_metadata_t const* meta) {
  if (meta->magic != OTA_METADATA_MAGIC && meta->magic != OTA_METADATA_MAGIC_RECOVERY) {
    return false;
  }
  if (meta->version != OTA_METADATA_VERSION ||
      meta->headerSize != sizeof(ota_metadata_t) ||
      meta->appAddr != BANK0_ADDR ||
      meta->otaAddr != BANK1_ADDR ||
      meta->imageBase != BANK0_ADDR ||
      meta->imageSize == 0 ||
      meta->imageSize > BANK_SIZE ||
      meta->pageSize != OTA_FLASH_PAGE_SIZE ||
      meta->pageCount == 0 ||
      meta->pageCount > OTA_MAX_PAGE_COUNT) {
    return false;
  }

  uint32_t stored_crc = meta->metadataCrc32;
  ota_metadata_t copy;
  memcpy(&copy, meta, sizeof(copy));
  copy.metadataCrc32 = 0xFFFFFFFFu;
  uint32_t calc_crc = crc32_calculate((uint8_t const*)&copy, offsetof(ota_metadata_t, metadataCrc32));
  return stored_crc == calc_crc;
}

static uint32_t page_len(ota_metadata_t const* meta, uint16_t page) {
  uint32_t offset = (uint32_t)page * OTA_FLASH_PAGE_SIZE;
  uint32_t len = meta->imageSize - offset;
  return len > OTA_FLASH_PAGE_SIZE ? OTA_FLASH_PAGE_SIZE : len;
}

static void flash_page_write(uint32_t address, uint8_t const* data) {
  nrfx_nvmc_page_erase(address);
  nrfx_nvmc_words_write(address, (uint32_t const*)data, OTA_FLASH_PAGE_SIZE / 4u);
}

static bool flash_page_matches(uint32_t address, uint8_t const* data) {
  return memcmp((void const*)address, data, OTA_FLASH_PAGE_SIZE) == 0;
}

static ota_progress_record_t const* progress_records(void) {
  return (ota_progress_record_t const*)(BANK_SETTINGS_ADDR + sizeof(ota_metadata_t));
}

static uint32_t progress_record_count(void) {
  return (OTA_FLASH_PAGE_SIZE - sizeof(ota_metadata_t)) / sizeof(ota_progress_record_t);
}

static ota_progress_t progress_read(void) {
  ota_progress_t progress = { 0, 0, 0 };
  ota_progress_record_t const* records = progress_records();
  uint32_t const count = progress_record_count();

  for (uint32_t i = 0; i < count; ++i) {
    if (records[i].magic == 0xFFFFFFFFu) {
      break;
    }
    if (records[i].magic == OTA_PROGRESS_MAGIC) {
      progress.pagesDone = records[i].pagesDone;
      progress.failures = records[i].failures;
      progress.flags = records[i].flags;
    }
  }
  return progress;
}

static bool progress_append(ota_progress_t const* progress) {
  ota_progress_record_t const* records = progress_records();
  uint32_t const count = progress_record_count();

  for (uint32_t i = 0; i < count; ++i) {
    if (records[i].magic == 0xFFFFFFFFu) {
      ota_progress_record_t record = {
        .magic = OTA_PROGRESS_MAGIC,
        .pagesDone = progress->pagesDone,
        .failures = progress->failures,
        .flags = progress->flags,
        .reserved0 = 0xFFFFFFFFu,
        .reserved1 = 0xFFFFFFFFu
      };
      nrfx_nvmc_words_write((uint32_t)&records[i], (uint32_t const*)&record, sizeof(record) / 4u);
      return true;
    }
  }
  return false;
}

static void metadata_write(ota_metadata_t const* meta) {
  ota_metadata_t copy;
  memcpy(&copy, meta, sizeof(copy));
  copy.metadataCrc32 = 0xFFFFFFFFu;
  copy.metadataCrc32 = crc32_calculate((uint8_t const*)&copy, offsetof(ota_metadata_t, metadataCrc32));

  nrfx_nvmc_page_erase(BANK_SETTINGS_ADDR);
  nrfx_nvmc_words_write(BANK_SETTINGS_ADDR, (uint32_t const*)&copy, (sizeof(copy) + 3u) / 4u);
}

static void write_failure_flag(uint8_t failures) {
  ota_metadata_t failed;
  memset(&failed, 0xFF, sizeof(failed));
  failed.magic = 0u;
  failed.version = OTA_METADATA_VERSION;
  failed.headerSize = sizeof(failed);
  failed.state = OTA_METADATA_STATE_FAILED;
  failed.appAddr = BANK0_ADDR;
  failed.otaAddr = BANK1_ADDR;
  failed.pageSize = OTA_FLASH_PAGE_SIZE;
  failed.sequence = failures;
  metadata_write(&failed);
}

static bool vectors_look_sane(uint32_t app_addr, uint32_t limit) {
  uint32_t const msp = *((uint32_t const*)(app_addr + 0u));
  uint32_t const reset = *((uint32_t const*)(app_addr + 4u));
  return (msp >= 0x20000000u && msp <= 0x20040000u && ((msp & 7u) == 0u)) &&
         (((reset & ~1u) >= app_addr) && ((reset & ~1u) < limit) && ((reset & 1u) == 1u));
}

static bool rollback_pages(ota_metadata_t const* meta, uint16_t pages) {
  if (pages > meta->pageCount) {
    pages = meta->pageCount;
  }

  for (uint16_t page = 0; page < pages; ++page) {
    uint32_t const ota_addr = BANK1_ADDR + (uint32_t)page * OTA_FLASH_PAGE_SIZE;
    uint32_t const app_addr = BANK0_ADDR + (uint32_t)page * OTA_FLASH_PAGE_SIZE;
    uint32_t const len = page_len(meta, page);
    memcpy(s_backup_page, (void const*)ota_addr, OTA_FLASH_PAGE_SIZE);
    if (crc32_calculate(s_backup_page, len) != meta->pageCrc32[page]) {
      return false;
    }
    flash_page_write(app_addr, s_backup_page);
    if (!flash_page_matches(app_addr, s_backup_page)) {
      return false;
    }
  }
  return true;
}

static bool install_ota(ota_metadata_t const* source_meta) {
  ota_progress_t progress = progress_read();
  progress.failures++;
  if (!progress_append(&progress)) {
    return false;
  }

  if (progress.failures > OTA_MAX_FAILURES) {
    (void)rollback_pages(source_meta, progress.pagesDone);
    write_failure_flag(progress.failures);
    return true;
  }

  for (uint16_t page = progress.pagesDone; page < source_meta->pageCount; ++page) {
    uint32_t const ota_addr = BANK1_ADDR + (uint32_t)page * OTA_FLASH_PAGE_SIZE;
    uint32_t const app_addr = BANK0_ADDR + (uint32_t)page * OTA_FLASH_PAGE_SIZE;
    uint32_t const len = page_len(source_meta, page);

    memcpy(s_new_page, (void const*)ota_addr, OTA_FLASH_PAGE_SIZE);
    if (crc32_calculate(s_new_page, len) != source_meta->pageCrc32[page]) {
      return false;
    }

    memcpy(s_backup_page, (void const*)app_addr, OTA_FLASH_PAGE_SIZE);

    flash_page_write(ota_addr, s_backup_page);
    if (!flash_page_matches(ota_addr, s_backup_page)) {
      return false;
    }

    flash_page_write(app_addr, s_new_page);
    if (!flash_page_matches(app_addr, s_new_page)) {
      return false;
    }

    progress.pagesDone = page + 1u;
    if (!progress_append(&progress)) {
      return false;
    }
  }

  if (!vectors_look_sane(BANK0_ADDR, BANK0_ADDR + BANK_SIZE)) {
    (void)rollback_pages(source_meta, source_meta->pageCount);
    write_failure_flag(progress.failures);
    return true;
  }
  if (image_crc32(BANK0_ADDR, source_meta->imageSize) != source_meta->imageCrc32) {
    (void)rollback_pages(source_meta, source_meta->pageCount);
    write_failure_flag(progress.failures);
    return true;
  }

  ota_metadata_t recovery;
  memcpy(&recovery, source_meta, sizeof(recovery));
  recovery.magic = OTA_METADATA_MAGIC_RECOVERY;
  recovery.state = OTA_METADATA_STATE_PENDING;
  recovery.imageCrc32 = image_crc32(BANK1_ADDR, source_meta->imageSize);
  for (uint16_t page = 0; page < source_meta->pageCount; ++page) {
    uint32_t const ota_addr = BANK1_ADDR + (uint32_t)page * OTA_FLASH_PAGE_SIZE;
    recovery.pageCrc32[page] = image_crc32(ota_addr, page_len(source_meta, page));
  }
  metadata_write(&recovery);
  return true;
}

static bool recover_ota(ota_metadata_t const* meta) {
  ota_progress_t progress = progress_read();
  progress.failures++;
  (void)progress_append(&progress);

  if (!rollback_pages(meta, meta->pageCount)) {
    if (progress.failures > OTA_MAX_FAILURES) {
      write_failure_flag(progress.failures);
      return true;
    }
    return false;
  }

  write_failure_flag(progress.failures);
  return true;
}

bool ota_swap_process(void) {
  ota_metadata_t const* meta = (ota_metadata_t const*)BANK_SETTINGS_ADDR;

  if (!metadata_valid(meta)) {
    return false;
  }

  if (meta->magic == OTA_METADATA_MAGIC) {
    return install_ota(meta);
  }
  if (meta->magic == OTA_METADATA_MAGIC_RECOVERY) {
    return recover_ota(meta);
  }
  return false;
}

bool ota_swap_clear_pending_magic(void) {
  uint32_t const magic = *((uint32_t const*)BANK_SETTINGS_ADDR);
  if (magic != OTA_METADATA_MAGIC && magic != OTA_METADATA_MAGIC_RECOVERY) {
    return true;
  }

  uint32_t const cleared = 0u;
  nrfx_nvmc_words_write(BANK_SETTINGS_ADDR, &cleared, 1u);
  return *((uint32_t const*)BANK_SETTINGS_ADDR) == cleared;
}
