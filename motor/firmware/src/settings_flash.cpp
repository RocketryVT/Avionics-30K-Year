#include "settings_flash.hpp"

#include <stdint.h>
#include <string.h>

#include "hardware/flash.h"
#include "pico/flash.h"
#include "pico/stdlib.h"

// Where the program ends in flash, from the SDK's linker script.
extern "C" char __flash_binary_end;

namespace {

// Which 4 KB sector of flash holds the settings: the last one before the
// final 64 KB.
//
// Not the very last sector, which is the usual choice. Every .uf2 this
// build makes carries one extra block aimed at the end of flash - picotool
// adds it as a workaround for RP2350 erratum E10. Current chips are told to
// ignore it, but an early chip would write it, wiping that sector on every
// reflash. One 64 KB block back is clear of that whatever the chip.
//
// The program starts at the beginning of flash and is well under 100 KB, so
// it is nowhere near. settings_sector_is_free() makes sure it stays that way.
constexpr uint32_t SETTINGS_OFFSET =
    PICO_FLASH_SIZE_BYTES - 64 * 1024 - FLASH_SECTOR_SIZE;

const uint8_t *stored()
{
    return reinterpret_cast<const uint8_t *>(XIP_BASE + SETTINGS_OFFSET);
}

// One page, the least flash can be written in. In RAM, because flash cannot
// be read while it is being written.
uint8_t g_page[FLASH_PAGE_SIZE];

// Called by flash_safe_execute() with interrupts off.
void erase_and_write(void *)
{
    flash_range_erase(SETTINGS_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(SETTINGS_OFFSET, g_page, FLASH_PAGE_SIZE);
}

}  // namespace

bool settings_sector_is_free()
{
    const uintptr_t program_end =
        reinterpret_cast<uintptr_t>(&__flash_binary_end);
    return program_end <= XIP_BASE + SETTINGS_OFFSET;
}

LoadResult load_settings(GearMode *gear, AxisSpec *az, AxisSpec *el)
{
    return decode_settings(stored(), SETTINGS_RECORD_LEN, gear, az, el);
}

void save_settings(GearMode gear, const AxisSpec &az, const AxisSpec &el)
{
    // The rest of the page is left as erased flash reads, 0xFF.
    memset(g_page, 0xFF, sizeof(g_page));
    encode_settings(gear, az, el, g_page, sizeof(g_page));

    // Only one core runs, so this just turns interrupts off for the write.
    // A failure leaves flash as it was, which the caller's read-back shows.
    flash_safe_execute(erase_and_write, nullptr, 100);
}
