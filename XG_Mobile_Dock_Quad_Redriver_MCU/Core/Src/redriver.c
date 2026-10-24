/* Driver for the four DS160PR410 redrivers on the quad-redriver board.
 *
 * Register map and sequences come from the public DS160PR410 data sheet (SNLS645)
 * and Programming Guide (SNLU255). Unlike the 8-channel redrivers used on the other
 * boards, a DS160PR410 has four channels that share one register page: channels are
 * selected through the share registers 0xFC / 0xFF and then written through the
 * same channel register offsets.
 *
 * The configuration is a per-channel boot and running setting, as on the original
 * redriver board (XG_Mobile_Dock_Redriver_MCU): every lane gets its boot setting when the
 * host releases reset, and every lane switches to its running setting together after the
 * boot hold. See redriver_channels[] below.
 */
#include "main.h"
#include <stdio.h>

extern I2C_HandleTypeDef hi2c2;

/* 7-bit SMBus addresses are set by the EQ1_ADDR1 / EQ0_ADDR0 pin straps (data sheet
 * Table 5: L0,L0 = 0x18, L0,L1 = 0x19, L1,L0 = 0x1C, L1,L1 = 0x1D). Each redriver
 * carries four lanes in a single direction. The levels below are the strap
 * positions set on the board's slide switches (EQ1_ADDR1, EQ0_ADDR0). */
#define CPU_GPU_0_3_ADDR_I2C 0x18 /* L0,L0 */
#define CPU_GPU_4_7_ADDR_I2C 0x19 /* L0,L1 */
#define GPU_CPU_0_3_ADDR_I2C 0x1C /* L1,L0 */
#define GPU_CPU_4_7_ADDR_I2C 0x1D /* L1,L1 */

#define REDRIVER_I2C_TIMEOUT_MS 50U
#define REDRIVER_READY_TRIALS   50U
#define REDRIVER_WRITE_RETRIES  5

/* Share registers (always accessible) */
#define SHARE_REG_DEVICE_ID     0xF1U
#define SHARE_DEVICE_ID         0x26U
#define SHARE_REG_CHANNEL_SEL   0xFCU /* [3:0] one bit per channel */
#define SHARE_REG_PAGE_SEL      0xFFU
#define PAGE_SHARE              0x00U /* share registers */
#define PAGE_CHANNEL            0x01U /* channel registers of the channels in 0xFC */

/* Channel registers (identical for each of the four channels) */
#define CH_REG_RX_DET_STATUS    0x01U /* [7] rx_det_comp_p, [6] rx_det_comp_n (latched) */
#define CH_REG_CTLE             0x03U /* [5:3] eq_bst2, [2:0] eq_bst1 */
#define CH_REG_CTLE_MODE        0x04U /* [5] eq_hi_gain (+3.5 dB), [0] stage 1 bypass */
#define CH_REG_DRIVER           0x06U /* [7:6] TX VOD select */
#define CH_REG_RX_DETECT        0x0DU /* [6] mr_rx_det_man, [5] en_rx_det_count */

#define CTLE_STAGES_MASK        0x3FU
#define CTLE_MODE_MASK          0x21U
#define CTLE_MODE_BYPASS        0x01U
#define CTLE_MODE_HI_GAIN       0x20U
#define DRIVER_VOD_MASK         0xC0U
#define DRIVER_VOD_SHIFT        6
/* reg 0x06 [5] drv_eq_en_override makes bits [4] drv_en_pre, [3] drv_en and [2] drv_en_cm_loop
 * (and reg 0x04 [6] eq_term_en, [3] eq_en) take effect. All of them off with the override on
 * powers the channel's driver and equalizer down; the override off returns control to the part. */
#define DRIVER_ENABLE_MASK      0x3CU
#define DRIVER_OVERRIDE_ALL_OFF 0x20U
#define CTLE_ENABLE_MASK        0x48U
#define RX_DETECT_MASK          0x60U
#define RX_DETECT_FORCE_VALID   0x40U /* mr_rx_det_man: always report a valid receiver */
#define RX_DETECT_USE_COUNTER   0x20U /* en_rx_det_count */

/* Always report a valid receiver at the far end, as the other redriver boards do:
 * the dock only powers the redrivers while PCIe is out of reset, and this keeps
 * them driving even when the far-end detect is marginal. */
#define REDRIVER_FORCE_RX_DETECT 1

// Channel numbers used in redriver_channels[]. A redriver has four channels (0-3); the
// 0-3 redrivers carry lanes 0-3 and the 4-7 redrivers carry lanes 4-7.
#define CHANNEL_0 0
#define CHANNEL_1 1
#define CHANNEL_2 2
#define CHANNEL_3 3
#define CHANNEL_4 0
#define CHANNEL_5 1
#define CHANNEL_6 2
#define CHANNEL_7 3

// EQ (CTLE) levels, reg 0x03 / 0x04 (data sheet Table 1, programming guide Table 8). One
// EQ_LEVEL_<n> per level; the boost at 4 GHz / 8 GHz is in the comment. A higher level
// boosts the high frequencies more (and the noise with them).
#define EQ_LEVEL_0   0   // -0.3 dB /  -0.8 dB  (flattest)
#define EQ_LEVEL_1   1   //  0.4 dB /   1.3 dB
#define EQ_LEVEL_2   2   //  3.3 dB /   5.7 dB
#define EQ_LEVEL_3   3   //  3.8 dB /   7.1 dB
#define EQ_LEVEL_4   4   //  4.9 dB /   8.4 dB
#define EQ_LEVEL_5   5   //  5.2 dB /   9.1 dB
#define EQ_LEVEL_6   6   //  5.4 dB /   9.8 dB
#define EQ_LEVEL_7   7   //  6.5 dB /  10.7 dB
#define EQ_LEVEL_8   8   //  6.7 dB /  11.3 dB
#define EQ_LEVEL_9   9   //  7.7 dB /  12.6 dB
#define EQ_LEVEL_10  10  //  8.7 dB /  13.6 dB
#define EQ_LEVEL_11  11  //  9.1 dB /  14.4 dB
#define EQ_LEVEL_12  12  //  9.4 dB /  15.0 dB
#define EQ_LEVEL_13  13  // 10.3 dB /  15.9 dB
#define EQ_LEVEL_14  14  // 10.6 dB /  16.5 dB
#define EQ_LEVEL_15  15  // 11.8 dB /  17.8 dB  (strongest)

// CTLE DC gain, reg 0x04[5]: extra flat gain on top of the EQ at every frequency.
#define DC_GAIN_0_DB      0
#define DC_GAIN_3_5_DB    1

// TX output swing (VOD), reg 0x06[7:6] (higher = bigger swing).
#define VOD_MINUS_6_DB    0
#define VOD_MINUS_3_5_DB  1
#define VOD_MINUS_1_6_DB  2
#define VOD_0_DB          3

// Register encoding of an EQ level: stage 1 bypass bit, eq_bst1 field and eq_bst2 field
// (already in bits [5:3]), plus the 4 GHz / 8 GHz boost for the log.
typedef struct {
    uint8_t bypass;
    uint8_t stage1;
    uint8_t stage2;
    const char *db;
} ctle_setting_t;

static const ctle_setting_t ctle_settings[16] = {
    { 1, 0, 0x00, "-0.3 / -0.8" }, // EQ_LEVEL_0
    { 1, 0, 0x18, "0.4 / 1.3" },   // EQ_LEVEL_1
    { 0, 1, 0x00, "3.3 / 5.7" },   // EQ_LEVEL_2
    { 0, 1, 0x10, "3.8 / 7.1" },   // EQ_LEVEL_3
    { 0, 2, 0x08, "4.9 / 8.4" },   // EQ_LEVEL_4
    { 0, 2, 0x10, "5.2 / 9.1" },   // EQ_LEVEL_5
    { 0, 2, 0x18, "5.4 / 9.8" },   // EQ_LEVEL_6
    { 0, 3, 0x10, "6.5 / 10.7" },  // EQ_LEVEL_7
    { 0, 3, 0x18, "6.7 / 11.3" },  // EQ_LEVEL_8
    { 0, 4, 0x18, "7.7 / 12.6" },  // EQ_LEVEL_9
    { 0, 5, 0x18, "8.7 / 13.6" },  // EQ_LEVEL_10
    { 0, 5, 0x20, "9.1 / 14.4" },  // EQ_LEVEL_11
    { 0, 5, 0x28, "9.4 / 15.0" },  // EQ_LEVEL_12
    { 0, 6, 0x28, "10.3 / 15.9" }, // EQ_LEVEL_13
    { 0, 6, 0x30, "10.6 / 16.5" }, // EQ_LEVEL_14
    { 0, 7, 0x38, "11.8 / 17.8" }, // EQ_LEVEL_15
};

// One redriver operating point for a channel.
typedef struct {
    uint8_t eq;        // EQ_LEVEL_0..EQ_LEVEL_15
    uint8_t dc_gain;   // DC_GAIN_0_DB / DC_GAIN_3_5_DB
    uint8_t vod;       // VOD_MINUS_6_DB .. VOD_0_DB
} redriver_setting_t;

// ---- Per-channel configuration (data): one row per lane and direction. ----
typedef struct {
    uint16_t addr;
    uint16_t chan;
    uint8_t  enabled;          // CHANNEL_ENABLED / CHANNEL_DISABLED
    redriver_setting_t boot;   // written when the host releases reset (PCIe training)
    redriver_setting_t run;    // written after the boot hold, all lanes together
} redriver_channel_t;

// redriver_channels[].enabled values. A disabled channel has its driver and equalizer powered down.
#define CHANNEL_ENABLED  1
#define CHANNEL_DISABLED 0

// Per-channel boot and running settings (the single source of truth for redriver config).
// The boot setting is written to every lane when the host releases reset; after the boot
// hold (REDRIVER_BOOT_HOLD_MS in xg_mobile_dock.c) every lane gets its running setting at
// once. enabled = CHANNEL_ENABLED / CHANNEL_DISABLED. Boot = run on every row right now.
static redriver_channel_t redriver_channels[] = {
    { GPU_CPU_0_3_ADDR_I2C, CHANNEL_0, CHANNEL_ENABLED,
      { .eq = EQ_LEVEL_0, .dc_gain = DC_GAIN_0_DB, .vod = VOD_MINUS_6_DB },
      { .eq = EQ_LEVEL_0, .dc_gain = DC_GAIN_0_DB, .vod = VOD_MINUS_6_DB } },
    { GPU_CPU_0_3_ADDR_I2C, CHANNEL_1, CHANNEL_ENABLED,
      { .eq = EQ_LEVEL_0, .dc_gain = DC_GAIN_0_DB, .vod = VOD_MINUS_6_DB },
      { .eq = EQ_LEVEL_0, .dc_gain = DC_GAIN_0_DB, .vod = VOD_MINUS_6_DB } },
    { GPU_CPU_0_3_ADDR_I2C, CHANNEL_2, CHANNEL_ENABLED,
      { .eq = EQ_LEVEL_0, .dc_gain = DC_GAIN_0_DB, .vod = VOD_MINUS_6_DB },
      { .eq = EQ_LEVEL_0, .dc_gain = DC_GAIN_0_DB, .vod = VOD_MINUS_6_DB } },
    { GPU_CPU_0_3_ADDR_I2C, CHANNEL_3, CHANNEL_ENABLED,
      { .eq = EQ_LEVEL_0, .dc_gain = DC_GAIN_0_DB, .vod = VOD_MINUS_6_DB },
      { .eq = EQ_LEVEL_0, .dc_gain = DC_GAIN_0_DB, .vod = VOD_MINUS_6_DB } },
    { GPU_CPU_4_7_ADDR_I2C, CHANNEL_4, CHANNEL_ENABLED,
      { .eq = EQ_LEVEL_0, .dc_gain = DC_GAIN_0_DB, .vod = VOD_MINUS_6_DB },
      { .eq = EQ_LEVEL_0, .dc_gain = DC_GAIN_0_DB, .vod = VOD_MINUS_6_DB } },
    { GPU_CPU_4_7_ADDR_I2C, CHANNEL_5, CHANNEL_ENABLED,
      { .eq = EQ_LEVEL_0, .dc_gain = DC_GAIN_0_DB, .vod = VOD_MINUS_6_DB },
      { .eq = EQ_LEVEL_0, .dc_gain = DC_GAIN_0_DB, .vod = VOD_MINUS_6_DB } },
    { GPU_CPU_4_7_ADDR_I2C, CHANNEL_6, CHANNEL_ENABLED,
      { .eq = EQ_LEVEL_0, .dc_gain = DC_GAIN_0_DB, .vod = VOD_MINUS_6_DB },
      { .eq = EQ_LEVEL_0, .dc_gain = DC_GAIN_0_DB, .vod = VOD_MINUS_6_DB } },
    { GPU_CPU_4_7_ADDR_I2C, CHANNEL_7, CHANNEL_ENABLED,
      { .eq = EQ_LEVEL_0, .dc_gain = DC_GAIN_0_DB, .vod = VOD_MINUS_6_DB },
      { .eq = EQ_LEVEL_0, .dc_gain = DC_GAIN_0_DB, .vod = VOD_MINUS_6_DB } },
    { CPU_GPU_0_3_ADDR_I2C, CHANNEL_0, CHANNEL_ENABLED,
      { .eq = EQ_LEVEL_8, .dc_gain = DC_GAIN_0_DB, .vod = VOD_0_DB },
      { .eq = EQ_LEVEL_8, .dc_gain = DC_GAIN_0_DB, .vod = VOD_0_DB } },
    { CPU_GPU_0_3_ADDR_I2C, CHANNEL_1, CHANNEL_ENABLED,
      { .eq = EQ_LEVEL_8, .dc_gain = DC_GAIN_0_DB, .vod = VOD_0_DB },
      { .eq = EQ_LEVEL_8, .dc_gain = DC_GAIN_0_DB, .vod = VOD_0_DB } },
    { CPU_GPU_0_3_ADDR_I2C, CHANNEL_2, CHANNEL_ENABLED,
      { .eq = EQ_LEVEL_8, .dc_gain = DC_GAIN_0_DB, .vod = VOD_0_DB },
      { .eq = EQ_LEVEL_8, .dc_gain = DC_GAIN_0_DB, .vod = VOD_0_DB } },
    { CPU_GPU_0_3_ADDR_I2C, CHANNEL_3, CHANNEL_ENABLED,
      { .eq = EQ_LEVEL_8, .dc_gain = DC_GAIN_0_DB, .vod = VOD_0_DB },
      { .eq = EQ_LEVEL_8, .dc_gain = DC_GAIN_0_DB, .vod = VOD_0_DB } },
    { CPU_GPU_4_7_ADDR_I2C, CHANNEL_4, CHANNEL_ENABLED,
      { .eq = EQ_LEVEL_8, .dc_gain = DC_GAIN_0_DB, .vod = VOD_0_DB },
      { .eq = EQ_LEVEL_8, .dc_gain = DC_GAIN_0_DB, .vod = VOD_0_DB } },
    { CPU_GPU_4_7_ADDR_I2C, CHANNEL_5, CHANNEL_ENABLED,
      { .eq = EQ_LEVEL_8, .dc_gain = DC_GAIN_0_DB, .vod = VOD_0_DB },
      { .eq = EQ_LEVEL_8, .dc_gain = DC_GAIN_0_DB, .vod = VOD_0_DB } },
    { CPU_GPU_4_7_ADDR_I2C, CHANNEL_6, CHANNEL_ENABLED,
      { .eq = EQ_LEVEL_8, .dc_gain = DC_GAIN_0_DB, .vod = VOD_0_DB },
      { .eq = EQ_LEVEL_8, .dc_gain = DC_GAIN_0_DB, .vod = VOD_0_DB } },
    { CPU_GPU_4_7_ADDR_I2C, CHANNEL_7, CHANNEL_ENABLED,
      { .eq = EQ_LEVEL_8, .dc_gain = DC_GAIN_0_DB, .vod = VOD_0_DB },
      { .eq = EQ_LEVEL_8, .dc_gain = DC_GAIN_0_DB, .vod = VOD_0_DB } },
};
#define NUM_REDRIVER_CHANNELS (sizeof(redriver_channels) / sizeof(redriver_channels[0]))

typedef struct {
    uint8_t addr;
    const char *name;
} redriver_bank_t;

static const redriver_bank_t redriver_banks[] = {
    { CPU_GPU_0_3_ADDR_I2C, "CPU->GPU 0-3" },
    { CPU_GPU_4_7_ADDR_I2C, "CPU->GPU 4-7" },
    { GPU_CPU_0_3_ADDR_I2C, "GPU->CPU 0-3" },
    { GPU_CPU_4_7_ADDR_I2C, "GPU->CPU 4-7" },
};
#define NUM_REDRIVER_BANKS (sizeof(redriver_banks) / sizeof(redriver_banks[0]))

static uint32_t status_last_print_ms;

// Last I2C failure, kept for the debug log (set by redriver_read / redriver_write and by
// a read-back mismatch). Printed once when a setting still fails after all retries.
static char redriver_last_error[112];

static const char *redriver_hal_text(HAL_StatusTypeDef status) {
    switch (status) {
        case HAL_OK:      return "OK";
        case HAL_ERROR:   return "ERROR";
        case HAL_BUSY:    return "BUSY";
        case HAL_TIMEOUT: return "TIMEOUT";
        default:          return "?";
    }
}

static void redriver_note_error(const char *op, uint8_t addr, uint8_t reg, HAL_StatusTypeDef status) {
    snprintf(redriver_last_error, sizeof(redriver_last_error), "%s 0x%02X reg 0x%02X failed: HAL %s, I2C error 0x%02lX",
             op, addr, reg, redriver_hal_text(status), (unsigned long)HAL_I2C_GetError(&hi2c2));
}

static HAL_StatusTypeDef redriver_read(uint8_t addr, uint8_t reg, uint8_t *value) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c2, (uint16_t)(addr << 1), reg, I2C_MEMADD_SIZE_8BIT,
                                                value, 1, REDRIVER_I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        redriver_note_error("read", addr, reg, status);
    }
    return status;
}

static HAL_StatusTypeDef redriver_write(uint8_t addr, uint8_t reg, uint8_t value) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c2, (uint16_t)(addr << 1), reg, I2C_MEMADD_SIZE_8BIT,
                                                 &value, 1, REDRIVER_I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        redriver_note_error("write", addr, reg, status);
    }
    return status;
}

// Read-modify-write of the bits in mask on the channel currently selected in 0xFC.
static HAL_StatusTypeDef redriver_update(uint8_t addr, uint8_t reg, uint8_t mask, uint8_t value) {
    uint8_t current = 0;

    if (redriver_read(addr, reg, &current) != HAL_OK) {
        return HAL_ERROR;
    }
    return redriver_write(addr, reg, (uint8_t)((current & (uint8_t)~mask) | (value & mask)));
}

// Select one channel of a redriver for channel-register access.
static HAL_StatusTypeDef redriver_select_channel(uint8_t addr, uint8_t chan) {
    if (redriver_write(addr, SHARE_REG_PAGE_SEL, PAGE_CHANNEL) != HAL_OK) {
        return HAL_ERROR;
    }
    return redriver_write(addr, SHARE_REG_CHANNEL_SEL, (uint8_t)(1U << chan));
}

static uint8_t ctle_register_value(const redriver_setting_t *s) {
    const ctle_setting_t *ctle = &ctle_settings[s->eq];
    return (uint8_t)(ctle->stage1 | ctle->stage2);
}

static uint8_t mode_register_value(const redriver_setting_t *s) {
    const ctle_setting_t *ctle = &ctle_settings[s->eq];
    return (uint8_t)((ctle->bypass ? CTLE_MODE_BYPASS : 0) | (s->dc_gain == DC_GAIN_3_5_DB ? CTLE_MODE_HI_GAIN : 0));
}

static uint8_t driver_register_value(const redriver_setting_t *s) {
    return (uint8_t)(s->vod << DRIVER_VOD_SHIFT);
}

static int redriver_setting_valid(const redriver_setting_t *s) {
    return s->eq <= EQ_LEVEL_15 && s->dc_gain <= DC_GAIN_3_5_DB && s->vod <= VOD_0_DB;
}

// Lane number and direction of a table row, for the log.
static int redriver_row_lane(const redriver_channel_t *c) {
    int upper = (c->addr == CPU_GPU_4_7_ADDR_I2C || c->addr == GPU_CPU_4_7_ADDR_I2C);
    return (upper ? 4 : 0) + c->chan;
}

static const char *redriver_row_direction(const redriver_channel_t *c) {
    return (c->addr == CPU_GPU_0_3_ADDR_I2C || c->addr == CPU_GPU_4_7_ADDR_I2C) ? "CPU->GPU" : "GPU->CPU";
}

static const char *redriver_vod_text(uint8_t vod) {
    static const char *const text[4] = { "-6", "-3.5", "-1.6", "0" };
    return text[vod & 3];
}

static void redriver_print_setting(const char *label, const redriver_channel_t *c, const redriver_setting_t *s) {
    printf("  %s %s lane %d (0x%02X ch%d): EQ level %u (%s dB @ 4/8 GHz), DC gain %s dB, VOD %s dB\n",
           label, redriver_row_direction(c), redriver_row_lane(c), c->addr, (int)c->chan, s->eq,
           ctle_settings[s->eq].db, s->dc_gain == DC_GAIN_3_5_DB ? "+3.5" : "0", redriver_vod_text(s->vod));
}

// Write one operating point to a channel: 0x04 = bypass | DC gain, 0x03 = EQ stages,
// 0x06[7:6] = VOD. The page is left on the share registers afterwards so later share
// reads are not redirected to a channel.
static HAL_StatusTypeDef redriver_write_setting(uint8_t addr, uint8_t chan, const redriver_setting_t *s, int init) {
    HAL_StatusTypeDef status = HAL_ERROR;

    if (redriver_select_channel(addr, chan) != HAL_OK) {
        goto done;
    }
#if REDRIVER_FORCE_RX_DETECT
    if (init && redriver_update(addr, CH_REG_RX_DETECT, RX_DETECT_MASK,
                                RX_DETECT_FORCE_VALID | RX_DETECT_USE_COUNTER) != HAL_OK) {
        goto done;
    }
#endif
    // A channel that was powered down earlier gets its enables back under the part's control.
    if (init && redriver_update(addr, CH_REG_DRIVER, DRIVER_ENABLE_MASK, 0) != HAL_OK) {
        goto done;
    }
    if (redriver_update(addr, CH_REG_CTLE_MODE, CTLE_MODE_MASK, mode_register_value(s)) != HAL_OK ||
        redriver_update(addr, CH_REG_CTLE, CTLE_STAGES_MASK, ctle_register_value(s)) != HAL_OK ||
        redriver_update(addr, CH_REG_DRIVER, DRIVER_VOD_MASK, driver_register_value(s)) != HAL_OK) {
        goto done;
    }
    status = HAL_OK;

done:
    redriver_write(addr, SHARE_REG_PAGE_SEL, PAGE_SHARE);
    return status;
}

// Compare a channel's registers against an operating point: 0 = match, 1 = mismatch,
// -1 = I2C error.
static int redriver_verify_setting(uint8_t addr, uint8_t chan, const redriver_setting_t *s) {
    uint8_t ctle = 0, mode = 0, driver = 0;
    int result = 0;

    if (redriver_select_channel(addr, chan) != HAL_OK ||
        redriver_read(addr, CH_REG_CTLE, &ctle) != HAL_OK ||
        redriver_read(addr, CH_REG_CTLE_MODE, &mode) != HAL_OK ||
        redriver_read(addr, CH_REG_DRIVER, &driver) != HAL_OK) {
        result = -1;
    } else if ((ctle & CTLE_STAGES_MASK) != ctle_register_value(s) ||
               (mode & CTLE_MODE_MASK) != mode_register_value(s) ||
               (driver & DRIVER_VOD_MASK) != driver_register_value(s)) {
        snprintf(redriver_last_error, sizeof(redriver_last_error),
                 "read-back mismatch: 0x03=%02X (want %02X) 0x04=%02X (want %02X) 0x06=%02X (want %02X)",
                 ctle & CTLE_STAGES_MASK, ctle_register_value(s), mode & CTLE_MODE_MASK, mode_register_value(s),
                 driver & DRIVER_VOD_MASK, driver_register_value(s));
        result = 1;
    }
    redriver_write(addr, SHARE_REG_PAGE_SEL, PAGE_SHARE);
    return result;
}

// Write and read back one operating point, retrying while a redriver that has just left
// standby settles. On failure redriver_last_error holds the reason from the last attempt.
static int redriver_apply_setting(const redriver_channel_t *c, const redriver_setting_t *s, int init) {
    redriver_last_error[0] = '\0';
    for (int attempt = 0; attempt < REDRIVER_WRITE_RETRIES; attempt++) {
        if (attempt > 0) {
            HAL_Delay(20);
        }
        if (redriver_write_setting((uint8_t)c->addr, (uint8_t)c->chan, s, init) == HAL_OK &&
            redriver_verify_setting((uint8_t)c->addr, (uint8_t)c->chan, s) == 0) {
            if (attempt > 0) {
                printf("  (0x%02X ch%d needed %d attempts; earlier: %s)\n", c->addr, (int)c->chan, attempt + 1,
                       redriver_last_error);
            }
            return 1;
        }
    }
    return 0;
}

/* Power the redrivers up (host out of reset) or down (host in reset) through their
 * PWDN pins: a high level is standby, low is active. The PWDN slide switches on the
 * board must be set to take these MCU signals. */
void power_up_down_redrivers(GPIO_PinState reset) {
    printf("Enabling redrivers %d if 1, disabling if 0\n", reset == GPIO_PIN_SET);
    reset = (reset == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(GPU_CPU_PD_0_3_GPIO_Port, GPU_CPU_PD_0_3_Pin, reset);
    HAL_GPIO_WritePin(GPU_CPU_PD_4_7_GPIO_Port, GPU_CPU_PD_4_7_Pin, reset);
    HAL_GPIO_WritePin(CPU_GPU_PD_0_3_GPIO_Port, CPU_GPU_PD_0_3_Pin, reset);
    HAL_GPIO_WritePin(CPU_GPU_PD_4_7_GPIO_Port, CPU_GPU_PD_4_7_Pin, reset);
}

static void print_redriver_bank_status(const redriver_bank_t *bank) {
    uint8_t device_id = 0;

    if (redriver_write(bank->addr, SHARE_REG_PAGE_SEL, PAGE_SHARE) != HAL_OK ||
        redriver_read(bank->addr, SHARE_REG_DEVICE_ID, &device_id) != HAL_OK) {
        printf("Redriver %s (0x%02X): no response (%s)\n", bank->name, bank->addr, redriver_last_error);
        return;
    }
    printf("Redriver %s (0x%02X) id=%02X RX detect (P/N latched) per channel:", bank->name, bank->addr,
           device_id);
    if (redriver_write(bank->addr, SHARE_REG_PAGE_SEL, PAGE_CHANNEL) == HAL_OK) {
        for (int channel = 0; channel < 4; channel++) {
            uint8_t detect = 0;

            if (redriver_write(bank->addr, SHARE_REG_CHANNEL_SEL, (uint8_t)(1U << channel)) == HAL_OK &&
                redriver_read(bank->addr, CH_REG_RX_DET_STATUS, &detect) == HAL_OK) {
                printf(" %d:%d%d", channel, (detect >> 7) & 1, (detect >> 6) & 1);
            } else {
                printf(" %d:??", channel);
            }
        }
    }
    printf("\n");
    redriver_write(bank->addr, SHARE_REG_PAGE_SEL, PAGE_SHARE);
}

/* Periodic status, only while the redrivers are powered up (host out of reset). */
void print_redrivers_status(void) {
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - status_last_print_ms) < 10000U) {
        return;
    }
    status_last_print_ms = now;
    if (HAL_GPIO_ReadPin(RST_GPIO_Port, RST_Pin) != GPIO_PIN_SET) {
        return;
    }
    printf("Redriver status:\n");
    for (unsigned int i = 0; i < NUM_REDRIVER_BANKS; i++) {
        print_redriver_bank_status(&redriver_banks[i]);
    }
}


// ============================================================================
// Boot and running configuration
//
// initialize_redrivers_boot() probes the redrivers and writes every enabled lane's boot
// setting (and powers disabled lanes down) when the host releases reset.
// initialize_redrivers_running() writes every lane's running setting, all lanes together,
// once the boot hold has passed (see xg_mobile_dock.c).
// ============================================================================

// 1 for a lane whose boot setting was written and read back; the running setting is only
// written to those lanes. Thread context only.
static int redriver_channel_ready[NUM_REDRIVER_CHANNELS];

/* Power a channel's driver and equalizer down (CHANNEL_DISABLED) and read it back. */
static int redriver_power_down_channel(const redriver_channel_t *c) {
    uint8_t addr = (uint8_t)c->addr, chan = (uint8_t)c->chan;
    int ok = 0;

    for (int attempt = 0; attempt < REDRIVER_WRITE_RETRIES && !ok; attempt++) {
        uint8_t driver = 0, mode = 0;

        if (attempt > 0) {
            HAL_Delay(20);
        }
        if (redriver_select_channel(addr, chan) == HAL_OK &&
            redriver_update(addr, CH_REG_DRIVER, DRIVER_ENABLE_MASK, DRIVER_OVERRIDE_ALL_OFF) == HAL_OK &&
            redriver_update(addr, CH_REG_CTLE_MODE, CTLE_ENABLE_MASK, 0) == HAL_OK &&
            redriver_read(addr, CH_REG_DRIVER, &driver) == HAL_OK &&
            redriver_read(addr, CH_REG_CTLE_MODE, &mode) == HAL_OK &&
            (driver & DRIVER_ENABLE_MASK) == DRIVER_OVERRIDE_ALL_OFF && (mode & CTLE_ENABLE_MASK) == 0) {
            ok = 1;
        }
        redriver_write(addr, SHARE_REG_PAGE_SEL, PAGE_SHARE);
    }
    return ok;
}

// Scan I2C2 and list every address that acknowledges, to find redrivers strapped to an
// unexpected address or a bus that is not connected to the MCU (SCL/SDA switches).
static void redriver_scan_bus(void) {
    int found = 0;

    printf("Redriver debug: scanning I2C2 (0x08-0x77):");
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(addr << 1), 2, 5) == HAL_OK) {
            printf(" 0x%02X", addr);
            found++;
        }
    }
    printf(found ? "\n" : " nothing answered (check power-down/PWDN, SCL/SDA and EN_SMB switches)\n");
}

// Probe every redriver, then write each enabled channel's boot setting (RX detect + the
// boot operating point) and read it back. Disabled channels are powered down.
void initialize_redrivers_boot(void) {
    int bank_ok[NUM_REDRIVER_BANKS];
    unsigned int banks_found = 0, lanes_enabled = 0, lanes_written = 0;

    printf("Redriver probe on I2C2:\n");
    for (unsigned int b = 0; b < NUM_REDRIVER_BANKS; b++) {
        const redriver_bank_t *bank = &redriver_banks[b];
        uint8_t device_id = 0;

        bank_ok[b] = 0;
        HAL_StatusTypeDef ready = HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(bank->addr << 1), REDRIVER_READY_TRIALS,
                                                        REDRIVER_I2C_TIMEOUT_MS);
        if (ready != HAL_OK) {
            printf("  %s (0x%02X): NOT FOUND (no ACK after %u tries: HAL %s, I2C error 0x%02lX)\n", bank->name,
                   bank->addr, (unsigned)REDRIVER_READY_TRIALS, redriver_hal_text(ready),
                   (unsigned long)HAL_I2C_GetError(&hi2c2));
            continue;
        }
        if (redriver_write(bank->addr, SHARE_REG_PAGE_SEL, PAGE_SHARE) != HAL_OK ||
            redriver_read(bank->addr, SHARE_REG_DEVICE_ID, &device_id) != HAL_OK) {
            printf("  %s (0x%02X): ACKs its address but register access failed (%s)\n", bank->name, bank->addr,
                   redriver_last_error);
            continue;
        }
        if (device_id != SHARE_DEVICE_ID) {
            printf("  %s (0x%02X): found, but device ID 0x%02X (expected 0x%02X) - wrong part or address?\n",
                   bank->name, bank->addr, device_id, SHARE_DEVICE_ID);
        } else {
            printf("  %s (0x%02X): found, device ID 0x%02X\n", bank->name, bank->addr, device_id);
        }
        bank_ok[b] = 1;
        banks_found++;
    }
    printf("Redriver probe: %u of %u redrivers found\n", banks_found, (unsigned)NUM_REDRIVER_BANKS);
    if (banks_found < NUM_REDRIVER_BANKS) {
        redriver_scan_bus();
    }

    printf("Redriver boot settings:\n");
    for (unsigned int i = 0; i < NUM_REDRIVER_CHANNELS; i++) {
        redriver_channel_t *c = &redriver_channels[i];
        int ok = 0;

        redriver_channel_ready[i] = 0;
        for (unsigned int b = 0; b < NUM_REDRIVER_BANKS; b++) {
            if (redriver_banks[b].addr == c->addr) {
                ok = bank_ok[b];
            }
        }
        if (!c->enabled) {
            if (ok) {
                if (redriver_power_down_channel(c)) {
                    printf("  %s lane %d (0x%02X ch%d): disabled (driver and equalizer powered down)\n",
                           redriver_row_direction(c), redriver_row_lane(c), c->addr, (int)c->chan);
                } else {
                    printf("  %s lane %d (0x%02X ch%d): power-down write FAILED: %s\n", redriver_row_direction(c),
                           redriver_row_lane(c), c->addr, (int)c->chan, redriver_last_error);
                }
            }
            continue;
        }
        lanes_enabled++;
        if (!redriver_setting_valid(&c->boot) || !redriver_setting_valid(&c->run)) {
            printf("  %s lane %d: invalid setting in redriver_channels[]\n", redriver_row_direction(c),
                   redriver_row_lane(c));
            continue;
        }
        if (!ok) {
            printf("  %s lane %d (0x%02X ch%d): skipped, redriver not found\n", redriver_row_direction(c),
                   redriver_row_lane(c), c->addr, (int)c->chan);
            continue;
        }
        if (redriver_apply_setting(c, &c->boot, 1)) {
            redriver_channel_ready[i] = 1;
            lanes_written++;
            redriver_print_setting("boot", c, &c->boot);
        } else {
            printf("  %s lane %d (0x%02X ch%d): boot setting write FAILED after %d tries: %s\n",
                   redriver_row_direction(c), redriver_row_lane(c), c->addr, (int)c->chan, REDRIVER_WRITE_RETRIES,
                   redriver_last_error);
        }
    }
    printf("Redriver boot: %u of %u enabled lanes written and verified\n", lanes_written, lanes_enabled);
}

// Write every ready channel's running setting and read it back, all lanes together.
void initialize_redrivers_running(void) {
    unsigned int lanes_ready = 0, lanes_written = 0;

    printf("Redriver running settings:\n");
    for (unsigned int i = 0; i < NUM_REDRIVER_CHANNELS; i++) {
        redriver_channel_t *c = &redriver_channels[i];

        if (!c->enabled || !redriver_channel_ready[i]) {
            continue;
        }
        lanes_ready++;
        if (redriver_apply_setting(c, &c->run, 0)) {
            lanes_written++;
            redriver_print_setting("run ", c, &c->run);
        } else {
            printf("  %s lane %d (0x%02X ch%d): running setting write FAILED after %d tries: %s\n",
                   redriver_row_direction(c), redriver_row_lane(c), c->addr, (int)c->chan, REDRIVER_WRITE_RETRIES,
                   redriver_last_error);
        }
    }
    printf("Redriver running: %u of %u lanes written and verified\n", lanes_written, lanes_ready);
}
