/* Driver for the four DS160PR410 redrivers on the quad-redriver board.
 *
 * Register map and sequences come from the public DS160PR410 data sheet (SNLS645)
 * and Programming Guide (SNLU255). Unlike the 8-channel redrivers used on the other
 * boards, a DS160PR410 has four channels that share one register page: channels are
 * selected through the share registers 0xFC / 0xFF and then written through the
 * same channel register offsets.
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
#define REDRIVER_CONFIG_RETRIES 5

/* Share registers (always accessible) */
#define SHARE_REG_DEVICE_ID     0xF1U
#define SHARE_DEVICE_ID         0x26U
#define SHARE_REG_CHANNEL_SEL   0xFCU /* [3:0] one bit per channel */
#define SHARE_REG_PAGE_SEL      0xFFU
#define PAGE_SHARE              0x00U /* share registers */
#define PAGE_CHANNEL            0x01U /* channel registers of the channels in 0xFC */
#define PAGE_CHANNEL_BROADCAST  0x03U /* writes go to all channels, reads from 0xFC */

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
#define RX_DETECT_MASK          0x60U
#define RX_DETECT_FORCE_VALID   0x40U /* mr_rx_det_man: always report a valid receiver */
#define RX_DETECT_USE_COUNTER   0x20U /* en_rx_det_count */

/* TX VOD select, reg 0x06 [7:6] */
#define VOD_MINUS_6_DB          0x00U
#define VOD_MINUS_3_5_DB        0x40U
#define VOD_MINUS_1_6_DB        0x80U
#define VOD_0_DB                0xC0U /* default, recommended */

/* Always report a valid receiver at the far end, as the other redriver boards do:
 * the dock only powers the redrivers while PCIe is out of reset, and this keeps
 * them driving even when the far-end detect is marginal. */
#define REDRIVER_FORCE_RX_DETECT 1

/* CTLE index 0..15 (data sheet Table 1, programming guide Table 8). Each entry is the
 * stage 1 bypass bit, the eq_bst1 field and the eq_bst2 field (already in bits [5:3]). */
typedef struct {
    uint8_t bypass;
    uint8_t stage1;
    uint8_t stage2;
} ctle_setting_t;

static const ctle_setting_t ctle_settings[16] = {
    { 1, 0, 0x00 }, /*  0: -0.3 dB @ 4 GHz, -0.8 dB @ 8 GHz */
    { 1, 0, 0x18 }, /*  1:  0.4 /  1.3 */
    { 0, 1, 0x00 }, /*  2:  3.3 /  5.7 */
    { 0, 1, 0x10 }, /*  3:  3.8 /  7.1 */
    { 0, 2, 0x08 }, /*  4:  4.9 /  8.4 */
    { 0, 2, 0x10 }, /*  5:  5.2 /  9.1 */
    { 0, 2, 0x18 }, /*  6:  5.4 /  9.8 */
    { 0, 3, 0x10 }, /*  7:  6.5 / 10.7 */
    { 0, 3, 0x18 }, /*  8:  6.7 / 11.3 */
    { 0, 4, 0x18 }, /*  9:  7.7 / 12.6 */
    { 0, 5, 0x18 }, /* 10:  8.7 / 13.6 */
    { 0, 5, 0x20 }, /* 11:  9.1 / 14.4 */
    { 0, 5, 0x28 }, /* 12:  9.4 / 15.0 */
    { 0, 6, 0x28 }, /* 13: 10.3 / 15.9 */
    { 0, 6, 0x30 }, /* 14: 10.6 / 16.5 */
    { 0, 7, 0x38 }, /* 15: 11.8 / 17.8 */
};

/* Per-redriver operating point. DC gain is the CTLE DC gain (0 dB or +3.5 dB). */
typedef struct {
    uint8_t addr;
    const char *name;
    uint8_t ctle_index;
    uint8_t dc_gain_hi;   /* 0 = 0 dB, 1 = +3.5 dB */
    uint8_t vod;          /* VOD_* */
} redriver_bank_t;

static const redriver_bank_t redriver_banks[] = {
    /* CPU -> GPU: 0 dB gain, middle CTLE */
    { CPU_GPU_0_3_ADDR_I2C, "CPU->GPU 0-3", 8, 0, VOD_0_DB },
    { CPU_GPU_4_7_ADDR_I2C, "CPU->GPU 4-7", 8, 0, VOD_0_DB },
    /* GPU -> CPU: lowest CTLE, output level reduced to -6 dB */
    { GPU_CPU_0_3_ADDR_I2C, "GPU->CPU 0-3", 0, 0, VOD_MINUS_6_DB },
    { GPU_CPU_4_7_ADDR_I2C, "GPU->CPU 4-7", 0, 0, VOD_MINUS_6_DB },
};
#define NUM_REDRIVER_BANKS (sizeof(redriver_banks) / sizeof(redriver_banks[0]))

static uint32_t status_last_print_ms;

static HAL_StatusTypeDef redriver_read(uint8_t addr, uint8_t reg, uint8_t *value) {
    return HAL_I2C_Mem_Read(&hi2c2, (uint16_t)(addr << 1), reg, I2C_MEMADD_SIZE_8BIT,
                            value, 1, REDRIVER_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef redriver_write(uint8_t addr, uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(&hi2c2, (uint16_t)(addr << 1), reg, I2C_MEMADD_SIZE_8BIT,
                             &value, 1, REDRIVER_I2C_TIMEOUT_MS);
}

/* Read-modify-write of the bits in mask. Every channel starts from the same
 * defaults, so reading back through the one channel selected in 0xFC while writes
 * are broadcast to all four is safe. */
static HAL_StatusTypeDef redriver_update(uint8_t addr, uint8_t reg, uint8_t mask, uint8_t value) {
    uint8_t current = 0;

    if (redriver_read(addr, reg, &current) != HAL_OK) {
        return HAL_ERROR;
    }
    return redriver_write(addr, reg, (uint8_t)((current & (uint8_t)~mask) | (value & mask)));
}

static uint8_t ctle_register_value(const redriver_bank_t *bank) {
    const ctle_setting_t *ctle = &ctle_settings[bank->ctle_index];
    return (uint8_t)(ctle->stage1 | ctle->stage2);
}

static uint8_t mode_register_value(const redriver_bank_t *bank) {
    const ctle_setting_t *ctle = &ctle_settings[bank->ctle_index];
    return (uint8_t)((ctle->bypass ? CTLE_MODE_BYPASS : 0) | (bank->dc_gain_hi ? CTLE_MODE_HI_GAIN : 0));
}

/* Compare each channel's registers against the intended operating point. */
static int redriver_verify_bank(const redriver_bank_t *bank) {
    int mismatches = 0;

    if (redriver_write(bank->addr, SHARE_REG_PAGE_SEL, PAGE_CHANNEL) != HAL_OK) {
        return -1;
    }
    for (int channel = 0; channel < 4; channel++) {
        uint8_t ctle = 0, mode = 0, driver = 0;

        if (redriver_write(bank->addr, SHARE_REG_CHANNEL_SEL, (uint8_t)(1U << channel)) != HAL_OK ||
            redriver_read(bank->addr, CH_REG_CTLE, &ctle) != HAL_OK ||
            redriver_read(bank->addr, CH_REG_CTLE_MODE, &mode) != HAL_OK ||
            redriver_read(bank->addr, CH_REG_DRIVER, &driver) != HAL_OK) {
            return -1;
        }
        if ((ctle & CTLE_STAGES_MASK) != ctle_register_value(bank) ||
            (mode & CTLE_MODE_MASK) != mode_register_value(bank) ||
            (driver & DRIVER_VOD_MASK) != bank->vod) {
            printf("Redriver 0x%02X ch%d mismatch: 0x03=%02X 0x04=%02X 0x06=%02X\n",
                   bank->addr, channel, ctle, mode, driver);
            mismatches++;
        }
    }
    return mismatches;
}

static HAL_StatusTypeDef redriver_configure_bank(const redriver_bank_t *bank) {
    uint8_t device_id = 0;
    HAL_StatusTypeDef status = HAL_ERROR;

    if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(bank->addr << 1), REDRIVER_READY_TRIALS,
                              REDRIVER_I2C_TIMEOUT_MS) != HAL_OK) {
        printf("Redriver %s (0x%02X) not found on I2C2\n", bank->name, bank->addr);
        return HAL_ERROR;
    }

    if (redriver_write(bank->addr, SHARE_REG_PAGE_SEL, PAGE_SHARE) != HAL_OK ||
        redriver_read(bank->addr, SHARE_REG_DEVICE_ID, &device_id) != HAL_OK) {
        printf("Redriver %s (0x%02X) did not answer a register read\n", bank->name, bank->addr);
        return HAL_ERROR;
    }
    if (device_id != SHARE_DEVICE_ID) {
        printf("Redriver %s (0x%02X): unexpected device ID 0x%02X (expected 0x%02X)\n",
               bank->name, bank->addr, device_id, SHARE_DEVICE_ID);
    }

    /* Channel page with broadcast writes; read-back comes from channel 0 only. */
    if (redriver_write(bank->addr, SHARE_REG_PAGE_SEL, PAGE_CHANNEL_BROADCAST) != HAL_OK ||
        redriver_write(bank->addr, SHARE_REG_CHANNEL_SEL, 0x01U) != HAL_OK ||
        redriver_update(bank->addr, CH_REG_CTLE_MODE, CTLE_MODE_MASK, mode_register_value(bank)) != HAL_OK ||
        redriver_update(bank->addr, CH_REG_CTLE, CTLE_STAGES_MASK, ctle_register_value(bank)) != HAL_OK ||
        redriver_update(bank->addr, CH_REG_DRIVER, DRIVER_VOD_MASK, bank->vod) != HAL_OK) {
        printf("Redriver %s (0x%02X): register write failed\n", bank->name, bank->addr);
        goto done;
    }
#if REDRIVER_FORCE_RX_DETECT
    if (redriver_update(bank->addr, CH_REG_RX_DETECT, RX_DETECT_MASK,
                        RX_DETECT_FORCE_VALID | RX_DETECT_USE_COUNTER) != HAL_OK) {
        printf("Redriver %s (0x%02X): RX detect write failed\n", bank->name, bank->addr);
        goto done;
    }
#endif

    if (redriver_verify_bank(bank) == 0) {
        status = HAL_OK;
    }

done:
    /* Leave the share page selected so later share-register reads are not
     * redirected to a channel. */
    redriver_write(bank->addr, SHARE_REG_PAGE_SEL, PAGE_SHARE);
    return status;
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

/* Apply the configuration to all four redrivers. Must run in thread context with
 * the redrivers powered up: it performs blocking I2C. A redriver that has just
 * left standby may need a moment before it answers, so each bank is retried. */
void redriver_configure(void) {
    int configured = 0;

    for (unsigned int i = 0; i < NUM_REDRIVER_BANKS; i++) {
        const redriver_bank_t *bank = &redriver_banks[i];
        HAL_StatusTypeDef status = HAL_ERROR;

        for (int attempt = 0; attempt < REDRIVER_CONFIG_RETRIES && status != HAL_OK; attempt++) {
            if (attempt > 0) {
                HAL_Delay(20);
            }
            status = redriver_configure_bank(bank);
        }
        if (status == HAL_OK) {
            printf("Redriver %s (0x%02X): CTLE index %u, VOD 0x%02X configured\n",
                   bank->name, bank->addr, bank->ctle_index, bank->vod);
            configured++;
        }
    }
    printf("Redrivers configured: %d of %u\n", configured, (unsigned)NUM_REDRIVER_BANKS);
}

static void print_redriver_bank_status(const redriver_bank_t *bank) {
    uint8_t device_id = 0;

    if (redriver_write(bank->addr, SHARE_REG_PAGE_SEL, PAGE_SHARE) != HAL_OK ||
        redriver_read(bank->addr, SHARE_REG_DEVICE_ID, &device_id) != HAL_OK) {
        printf("Redriver %s (0x%02X): no response\n", bank->name, bank->addr);
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
