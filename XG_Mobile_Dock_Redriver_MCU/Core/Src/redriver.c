#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stm32f0xx_hal_i2c.h>

extern I2C_HandleTypeDef hi2c2;

#define CPU_GPU_0_3_ADDR_I2C 0x18
#define CPU_GPU_4_7_ADDR_I2C 0x19
#define GPU_CPU_0_3_ADDR_I2C 0x22
#define GPU_CPU_4_7_ADDR_I2C 0x23
#define REGISTER_READ_MASK 0b00000001

#define CHANNEL_0_REGISTER 0x00
#define CHANNEL_1_REGISTER 0x20
#define CHANNEL_2_REGISTER 0x40
#define CHANNEL_3_REGISTER 0x60
#define CHANNEL_4_REGISTER 0x00
#define CHANNEL_5_REGISTER 0x20
#define CHANNEL_6_REGISTER 0x40
#define CHANNEL_7_REGISTER 0x60

#define RX_DET_COMP_P 0b10000000
#define RX_DET_COMP_N 0b01000000

#define EQ_CONTROL_REGISTER_OFFSET 0x01
#define EQ_GAIN_FLAT_GAIN_REGISTER_OFFSET 0x03


#define RX_DETECT_CONTROL_REGISTER_OFFSET 0x04
#define MR_RX_DET_MAN 0b00000100 //force always detect
#define EN_RX_DET_COUNT 0b00000010 //enable additional rx detect polling
#define SEL_RX_DET_COUNT 0b00000001 //enable additional rx detect 0 = 2 valid detections, 1 = 3 additional valid detections
#define RX_DETECT_WRITE_MASK 0b11111000


#define BIAS_REGISTER_OFFSET 0x06
#define BIAS_CURRENT_2 0b00100000
#define BIAS_CURRENT_1 0b00010000
#define BIAS_CURRENT_0 0b00001000
#define BIAS_WRITE_MASK 0b11000111
// Bias-current field value for reg 0x06[5:3] (the ramp shifts it << 3). 1 = 001b
// (datasheet "best performance") .. 7 = 111b (max drive).
#define BIAS_LEVEL_1 1
#define BIAS_LEVEL_2 2
#define BIAS_LEVEL_3 3
#define BIAS_LEVEL_4 4
#define BIAS_LEVEL_5 5
#define BIAS_LEVEL_6 6
#define BIAS_LEVEL_7 7


#define POWER_DOWN_REGISTER_OFFSET 0x05
#define POWER_UP_CHANNEL 0b01111111
#define POWER_DOWN_CHANNEL 0b10000000


// Flat-gain field, reg 0x03[2:0] (higher = more gain). 5 = 0 dB, 0 = -6 dB.
#define FLAT_GAIN_LEVEL_0  0
#define FLAT_GAIN_LEVEL_1  1
#define FLAT_GAIN_LEVEL_2  2
#define FLAT_GAIN_LEVEL_3  3
#define FLAT_GAIN_LEVEL_4  4
#define FLAT_GAIN_LEVEL_5  5
#define FLAT_GAIN_LEVEL_6  6
#define FLAT_GAIN_LEVEL_7  7


// reg 0x01 stage-1 bypass, bit [7]. Use as the stage-1 "off" value on its own, or OR it onto a
// stage-1 boost value. The ramp masks this bit separately from the boost field [6:3].
#define EQ_STAGE1_BYPASS  (1 << 7)

// EQ levels (DS320PR810 Table 7-1). One EQ<n> per level; levels 3 & 4 do not exist. Each level
// defines a stage-1 (0x01[6:3]), stage-2 (0x01[2:0]) and profile (0x03[6:3]) value, grouped by
// field below. Values are pre-positioned so they OR into the register byte; levels 0/1/2 include
// the stage-1 bypass bit. Pick a level per channel, e.g. .eq_stage1 = EQ7_STAGE1, .eq_stage2 =
// EQ7_STAGE2, .eq_profile = EQ7_PROFILE.

// Stage-1 boost, reg 0x01[6:3] (levels 0/1/2 add EQ_STAGE1_BYPASS).
#define EQ0_STAGE1   ((0  << 3) | EQ_STAGE1_BYPASS)
#define EQ1_STAGE1   ((1  << 3) | EQ_STAGE1_BYPASS)
#define EQ2_STAGE1   ((3  << 3) | EQ_STAGE1_BYPASS)
#define EQ5_STAGE1   (0  << 3)
#define EQ6_STAGE1   (1  << 3)
#define EQ7_STAGE1   (2  << 3)
#define EQ8_STAGE1   (3  << 3)
#define EQ9_STAGE1   (4  << 3)
#define EQ10_STAGE1  (5  << 3)
#define EQ11_STAGE1  (6  << 3)
#define EQ12_STAGE1  (8  << 3)
#define EQ13_STAGE1  (10 << 3)
#define EQ14_STAGE1  (10 << 3)
#define EQ15_STAGE1  (11 << 3)
#define EQ16_STAGE1  (12 << 3)
#define EQ17_STAGE1  (13 << 3)
#define EQ18_STAGE1  (14 << 3)
#define EQ19_STAGE1  (15 << 3)

// Stage-2 boost, reg 0x01[2:0].
#define EQ0_STAGE2   0
#define EQ1_STAGE2   0
#define EQ2_STAGE2   0
#define EQ5_STAGE2   0
#define EQ6_STAGE2   0
#define EQ7_STAGE2   0
#define EQ8_STAGE2   0
#define EQ9_STAGE2   0
#define EQ10_STAGE2  1
#define EQ11_STAGE2  1
#define EQ12_STAGE2  1
#define EQ13_STAGE2  1
#define EQ14_STAGE2  2
#define EQ15_STAGE2  3
#define EQ16_STAGE2  4
#define EQ17_STAGE2  5
#define EQ18_STAGE2  6
#define EQ19_STAGE2  7

// EQ profile, reg 0x03[6:3].
#define EQ0_PROFILE   (0  << 3)
#define EQ1_PROFILE   (0  << 3)
#define EQ2_PROFILE   (0  << 3)
#define EQ5_PROFILE   (1  << 3)
#define EQ6_PROFILE   (1  << 3)
#define EQ7_PROFILE   (1  << 3)
#define EQ8_PROFILE   (3  << 3)
#define EQ9_PROFILE   (3  << 3)
#define EQ10_PROFILE  (7  << 3)
#define EQ11_PROFILE  (7  << 3)
#define EQ12_PROFILE  (7  << 3)
#define EQ13_PROFILE  (7  << 3)
#define EQ14_PROFILE  (15 << 3)
#define EQ15_PROFILE  (15 << 3)
#define EQ16_PROFILE  (15 << 3)
#define EQ17_PROFILE  (15 << 3)
#define EQ18_PROFILE  (15 << 3)
#define EQ19_PROFILE  (15 << 3)

// Logical 0..15 magnitude of a value positioned in reg bits [6:3] (eq_stage1 / eq_profile);
// used by the ramp to walk the field. Ignores the stage-1 bypass bit [7].
#define EQ_MAG_6_3(x)  (((x) >> 3) & 0x0F)


// One redriver operating point: the per-channel register fields as pre-positioned byte
// values (each constant occupies only its own register bits, so the fields OR together).
// The ramp walks each field's magnitude independently from the boot set to the run set;
// the stage-1 bypass bit (EQ_STAGE1_BYPASS, bit [7] of eq_stage1) is discrete - disabled
// while ramping (unless both endpoints bypass), snapped to the run value at the end.
typedef struct {
    uint8_t eq_stage1;   // 0x01[6:3] EQ<n>_STAGE1 (bit 7 = EQ_STAGE1_BYPASS)
    uint8_t eq_stage2;   // 0x01[2:0] EQ<n>_STAGE2
    uint8_t eq_profile;  // 0x03[6:3] EQ<n>_PROFILE
    uint8_t flat_gain;   // 0x03[2:0] FLAT_GAIN_LEVEL_* (5 = 0 dB, 0 = -6 dB)
    uint8_t bias;        // 0x06[5:3] BIAS_LEVEL_* (1..7, written << 3)
} redriver_setting_t;

// ---- Per-channel ramp configuration (data). The ramp engine that consumes this
// ---- table (state, helpers, redriver_ramp_start / _pump) lives further down. ----
typedef struct {
    uint16_t addr;
    uint16_t chan;
    uint8_t  enabled;          // CHANNEL_ENABLED / CHANNEL_DISABLED
    redriver_setting_t boot;   // held through the training window (ramp start point)
    redriver_setting_t run;    // ramped to, field by field, after the boot hold
    uint16_t rampup_step_ms;   // ms between steps (one field value per step)
    uint16_t rampup_delay_ms;  // extra per-lane hold before ramping (stagger lanes)
} ramp_channel_t;

// Every channel holds its boot (training) config this long before the ramp to the
// running state begins. This is the PCIe link-training window.
#define REDRIVER_BOOT_HOLD_MS 5000

// Per-lane ramp stagger: channels begin their boot->running ramp one after another
// (not all at once) after the boot hold. Start order is 0,1,2,3,4,5,7,6 - channel 6
// goes LAST - set by each row's delay = rank * RAMP_STAGGER_MS in the table below.
#define RAMP_STAGGER_MS 500

// ms per ramp step (one sequence entry / one field value applied per tick).
#define RAMP_STEP_MS 25

// ramp_channels[].enabled values.
#define CHANNEL_ENABLED  1
#define CHANNEL_DISABLED 0

// Per-channel setup + ramp endpoints (the single source of truth for redriver config).
// Each channel = a boot set + a run set of redriver_setting_t fields; the ramp walks
// every field independently boot -> run. enabled = CHANNEL_ENABLED / CHANNEL_DISABLED.
// step = RAMP_STEP_MS per walked value; delay = per-lane stagger on top of REDRIVER_BOOT_HOLD_MS
// (rank * RAMP_STAGGER_MS): start order 0,1,2,3,4,5,7,6 so channel 6 ramps LAST.
static ramp_channel_t ramp_channels[] = {
    { GPU_CPU_0_3_ADDR_I2C, CHANNEL_0_REGISTER, CHANNEL_ENABLED,
      { .eq_stage1 = EQ19_STAGE1, .eq_stage2 = EQ19_STAGE2, .eq_profile = EQ19_PROFILE, .flat_gain = FLAT_GAIN_LEVEL_0, .bias = BIAS_LEVEL_7 },
      { .eq_stage1 = EQ0_STAGE1,  .eq_stage2 = EQ0_STAGE2,  .eq_profile = EQ10_PROFILE,  .flat_gain = FLAT_GAIN_LEVEL_0, .bias = BIAS_LEVEL_7 },
      RAMP_STEP_MS, 0 * RAMP_STAGGER_MS },
    { GPU_CPU_0_3_ADDR_I2C, CHANNEL_1_REGISTER, CHANNEL_ENABLED,
      { .eq_stage1 = EQ19_STAGE1, .eq_stage2 = EQ19_STAGE2, .eq_profile = EQ19_PROFILE, .flat_gain = FLAT_GAIN_LEVEL_0, .bias = BIAS_LEVEL_7 },
      { .eq_stage1 = EQ0_STAGE1,  .eq_stage2 = EQ0_STAGE2,  .eq_profile = EQ10_PROFILE,  .flat_gain = FLAT_GAIN_LEVEL_0, .bias = BIAS_LEVEL_7 },
      RAMP_STEP_MS, 1 * RAMP_STAGGER_MS },
    { GPU_CPU_0_3_ADDR_I2C, CHANNEL_2_REGISTER, CHANNEL_ENABLED,
      { .eq_stage1 = EQ19_STAGE1, .eq_stage2 = EQ19_STAGE2, .eq_profile = EQ19_PROFILE, .flat_gain = FLAT_GAIN_LEVEL_0, .bias = BIAS_LEVEL_7 },
      { .eq_stage1 = EQ0_STAGE1,  .eq_stage2 = EQ0_STAGE2,  .eq_profile = EQ10_PROFILE,  .flat_gain = FLAT_GAIN_LEVEL_0, .bias = BIAS_LEVEL_7 },
      RAMP_STEP_MS, 2 * RAMP_STAGGER_MS },
    { GPU_CPU_0_3_ADDR_I2C, CHANNEL_3_REGISTER, CHANNEL_ENABLED,
      { .eq_stage1 = EQ19_STAGE1, .eq_stage2 = EQ19_STAGE2, .eq_profile = EQ19_PROFILE, .flat_gain = FLAT_GAIN_LEVEL_0, .bias = BIAS_LEVEL_7 },
      { .eq_stage1 = EQ0_STAGE1,  .eq_stage2 = EQ0_STAGE2,  .eq_profile = EQ10_PROFILE,  .flat_gain = FLAT_GAIN_LEVEL_0, .bias = BIAS_LEVEL_7 },
      RAMP_STEP_MS, 3 * RAMP_STAGGER_MS },
    { GPU_CPU_4_7_ADDR_I2C, CHANNEL_4_REGISTER, CHANNEL_ENABLED,
      { .eq_stage1 = EQ19_STAGE1, .eq_stage2 = EQ19_STAGE2, .eq_profile = EQ19_PROFILE, .flat_gain = FLAT_GAIN_LEVEL_0, .bias = BIAS_LEVEL_7 },
      { .eq_stage1 = EQ0_STAGE1,  .eq_stage2 = EQ0_STAGE2,  .eq_profile = EQ10_PROFILE,  .flat_gain = FLAT_GAIN_LEVEL_0, .bias = BIAS_LEVEL_7 },
      RAMP_STEP_MS, 4 * RAMP_STAGGER_MS },
    { GPU_CPU_4_7_ADDR_I2C, CHANNEL_5_REGISTER, CHANNEL_ENABLED,
      { .eq_stage1 = EQ19_STAGE1, .eq_stage2 = EQ19_STAGE2, .eq_profile = EQ19_PROFILE, .flat_gain = FLAT_GAIN_LEVEL_0, .bias = BIAS_LEVEL_7 },
      { .eq_stage1 = EQ0_STAGE1,  .eq_stage2 = EQ0_STAGE2,  .eq_profile = EQ10_PROFILE,  .flat_gain = FLAT_GAIN_LEVEL_0, .bias = BIAS_LEVEL_7 },
      RAMP_STEP_MS, 5 * RAMP_STAGGER_MS },
    { GPU_CPU_4_7_ADDR_I2C, CHANNEL_6_REGISTER, CHANNEL_ENABLED,
      { .eq_stage1 = EQ0_STAGE1, .eq_stage2 = EQ0_STAGE2, .eq_profile = EQ19_PROFILE, .flat_gain = FLAT_GAIN_LEVEL_0, .bias = BIAS_LEVEL_7 },
      { .eq_stage1 = EQ0_STAGE1, .eq_stage2 = EQ0_STAGE2, .eq_profile = EQ19_PROFILE, .flat_gain = FLAT_GAIN_LEVEL_0, .bias = BIAS_LEVEL_7 },
      RAMP_STEP_MS, 7 * RAMP_STAGGER_MS },
    { GPU_CPU_4_7_ADDR_I2C, CHANNEL_7_REGISTER, CHANNEL_ENABLED,
      { .eq_stage1 = EQ19_STAGE1, .eq_stage2 = EQ19_STAGE2, .eq_profile = EQ19_PROFILE, .flat_gain = FLAT_GAIN_LEVEL_0, .bias = BIAS_LEVEL_7 },
      { .eq_stage1 = EQ0_STAGE1,  .eq_stage2 = EQ0_STAGE2,  .eq_profile = EQ8_PROFILE,  .flat_gain = FLAT_GAIN_LEVEL_0, .bias = BIAS_LEVEL_7 },
      RAMP_STEP_MS, 6 * RAMP_STAGGER_MS },
    { CPU_GPU_0_3_ADDR_I2C, CHANNEL_0_REGISTER, CHANNEL_ENABLED,
      { .eq_stage1 = EQ7_STAGE1, .eq_stage2 = EQ7_STAGE2, .eq_profile = EQ7_PROFILE, .flat_gain = FLAT_GAIN_LEVEL_5, .bias = BIAS_LEVEL_1 },
      { .eq_stage1 = EQ7_STAGE1, .eq_stage2 = EQ7_STAGE2, .eq_profile = EQ7_PROFILE, .flat_gain = FLAT_GAIN_LEVEL_5, .bias = BIAS_LEVEL_1 },
      RAMP_STEP_MS, 0 * RAMP_STAGGER_MS },
    { CPU_GPU_0_3_ADDR_I2C, CHANNEL_1_REGISTER, CHANNEL_ENABLED,
      { .eq_stage1 = EQ7_STAGE1, .eq_stage2 = EQ7_STAGE2, .eq_profile = EQ7_PROFILE, .flat_gain = FLAT_GAIN_LEVEL_5, .bias = BIAS_LEVEL_1 },
      { .eq_stage1 = EQ7_STAGE1, .eq_stage2 = EQ7_STAGE2, .eq_profile = EQ7_PROFILE, .flat_gain = FLAT_GAIN_LEVEL_5, .bias = BIAS_LEVEL_1 },
      RAMP_STEP_MS, 1 * RAMP_STAGGER_MS },
    { CPU_GPU_0_3_ADDR_I2C, CHANNEL_2_REGISTER, CHANNEL_ENABLED,
      { .eq_stage1 = EQ7_STAGE1, .eq_stage2 = EQ7_STAGE2, .eq_profile = EQ7_PROFILE, .flat_gain = FLAT_GAIN_LEVEL_5, .bias = BIAS_LEVEL_1 },
      { .eq_stage1 = EQ7_STAGE1, .eq_stage2 = EQ7_STAGE2, .eq_profile = EQ7_PROFILE, .flat_gain = FLAT_GAIN_LEVEL_5, .bias = BIAS_LEVEL_1 },
      RAMP_STEP_MS, 2 * RAMP_STAGGER_MS },
    { CPU_GPU_0_3_ADDR_I2C, CHANNEL_3_REGISTER, CHANNEL_ENABLED,
      { .eq_stage1 = EQ7_STAGE1, .eq_stage2 = EQ7_STAGE2, .eq_profile = EQ7_PROFILE, .flat_gain = FLAT_GAIN_LEVEL_5, .bias = BIAS_LEVEL_1 },
      { .eq_stage1 = EQ7_STAGE1, .eq_stage2 = EQ7_STAGE2, .eq_profile = EQ7_PROFILE, .flat_gain = FLAT_GAIN_LEVEL_5, .bias = BIAS_LEVEL_1 },
      RAMP_STEP_MS, 3 * RAMP_STAGGER_MS },
    { CPU_GPU_4_7_ADDR_I2C, CHANNEL_4_REGISTER, CHANNEL_ENABLED,
      { .eq_stage1 = EQ7_STAGE1, .eq_stage2 = EQ7_STAGE2, .eq_profile = EQ7_PROFILE, .flat_gain = FLAT_GAIN_LEVEL_5, .bias = BIAS_LEVEL_1 },
      { .eq_stage1 = EQ7_STAGE1, .eq_stage2 = EQ7_STAGE2, .eq_profile = EQ7_PROFILE, .flat_gain = FLAT_GAIN_LEVEL_5, .bias = BIAS_LEVEL_1 },
      RAMP_STEP_MS, 4 * RAMP_STAGGER_MS },
    { CPU_GPU_4_7_ADDR_I2C, CHANNEL_5_REGISTER, CHANNEL_ENABLED,
      { .eq_stage1 = EQ7_STAGE1, .eq_stage2 = EQ7_STAGE2, .eq_profile = EQ7_PROFILE, .flat_gain = FLAT_GAIN_LEVEL_5, .bias = BIAS_LEVEL_1 },
      { .eq_stage1 = EQ7_STAGE1, .eq_stage2 = EQ7_STAGE2, .eq_profile = EQ7_PROFILE, .flat_gain = FLAT_GAIN_LEVEL_5, .bias = BIAS_LEVEL_1 },
      RAMP_STEP_MS, 5 * RAMP_STAGGER_MS },
    { CPU_GPU_4_7_ADDR_I2C, CHANNEL_6_REGISTER, CHANNEL_ENABLED,
      { .eq_stage1 = EQ7_STAGE1, .eq_stage2 = EQ7_STAGE2, .eq_profile = EQ7_PROFILE, .flat_gain = FLAT_GAIN_LEVEL_5, .bias = BIAS_LEVEL_1 },
      { .eq_stage1 = EQ7_STAGE1, .eq_stage2 = EQ7_STAGE2, .eq_profile = EQ7_PROFILE, .flat_gain = FLAT_GAIN_LEVEL_5, .bias = BIAS_LEVEL_1 },
      RAMP_STEP_MS, 7 * RAMP_STAGGER_MS },
    { CPU_GPU_4_7_ADDR_I2C, CHANNEL_7_REGISTER, CHANNEL_ENABLED,
      { .eq_stage1 = EQ7_STAGE1, .eq_stage2 = EQ7_STAGE2, .eq_profile = EQ7_PROFILE, .flat_gain = FLAT_GAIN_LEVEL_5, .bias = BIAS_LEVEL_1 },
      { .eq_stage1 = EQ7_STAGE1, .eq_stage2 = EQ7_STAGE2, .eq_profile = EQ7_PROFILE, .flat_gain = FLAT_GAIN_LEVEL_5, .bias = BIAS_LEVEL_1 },
      RAMP_STEP_MS, 6 * RAMP_STAGGER_MS },
};
#define NUM_RAMP_CHANNELS (sizeof(ramp_channels) / sizeof(ramp_channels[0]))

static uint8_t register_data;
static uint32_t status_poll_ticks_delay = 0;

static void initialize_channel_eq(uint16_t redriver_address, uint16_t channel_register_address, uint8_t eq_index) {
    register_data = eq_index;
    printf("Init channel eq\n");
    HAL_StatusTypeDef result = HAL_I2C_Mem_Write(&hi2c2, redriver_address << 1, channel_register_address + EQ_CONTROL_REGISTER_OFFSET, I2C_MEMADD_SIZE_8BIT, &register_data, 1, 500);
    printf("Init channel eq done %d\n", result);
}

static void initialize_channel_rx_detect(uint16_t redriver_address, uint16_t channel_register_address, uint8_t rx_detect) {
    printf("Init channel rx\n");
    HAL_I2C_Mem_Read(&hi2c2, (redriver_address << 1) | REGISTER_READ_MASK, channel_register_address + RX_DETECT_CONTROL_REGISTER_OFFSET, I2C_MEMADD_SIZE_8BIT, &register_data, 1, 500);
    register_data = RX_DETECT_WRITE_MASK & register_data;
    register_data = register_data | rx_detect;
    HAL_StatusTypeDef result = HAL_I2C_Mem_Write(&hi2c2, redriver_address << 1, channel_register_address + RX_DETECT_CONTROL_REGISTER_OFFSET, I2C_MEMADD_SIZE_8BIT, &register_data, 1, 500);
    printf("Init channel rx done %d\n", result);
}

//(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
 //                                     uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
static void initialize_channel_bias(uint16_t redriver_address, uint16_t channel_register_address, uint8_t bias) {
    printf("Init channel bias\n");
    HAL_I2C_Mem_Read(&hi2c2, (redriver_address << 1) | REGISTER_READ_MASK, channel_register_address + BIAS_REGISTER_OFFSET, I2C_MEMADD_SIZE_8BIT, &register_data, 1, 500);
    register_data = BIAS_WRITE_MASK & register_data;
    register_data = register_data | bias;
    HAL_StatusTypeDef result = HAL_I2C_Mem_Write(&hi2c2, redriver_address << 1, channel_register_address + BIAS_REGISTER_OFFSET, I2C_MEMADD_SIZE_8BIT, &register_data, 1, 500);
    printf("Init channel bias done %d\n", result);
}

static void initialize_channel_eq_profile(uint16_t redriver_address, uint16_t channel_register_address, uint8_t eq_profile) {
    register_data = eq_profile;
    printf("Init channel eq profile\n");
    HAL_StatusTypeDef result  = HAL_I2C_Mem_Write(&hi2c2, redriver_address << 1, channel_register_address + EQ_GAIN_FLAT_GAIN_REGISTER_OFFSET, I2C_MEMADD_SIZE_8BIT, &register_data, 1, 500);
    printf("Init channel eq profile done %d\n", result);
}

static void initialize_channel_pd(uint16_t redriver_address, uint16_t channel_register_address, uint8_t pd) {
    register_data = pd;
    printf("Init channel power down\n");
    HAL_StatusTypeDef result  = HAL_I2C_Mem_Write(&hi2c2, redriver_address << 1, channel_register_address + POWER_DOWN_REGISTER_OFFSET, I2C_MEMADD_SIZE_8BIT, &register_data, 1, 500);
    printf("Init channel power down done %d\n", result);
}


// Per-channel boot/running register setup is data-driven from the ramp_channels[]
// table below (redriver_apply_boot / redriver_apply_running in the ramp module);
// there are no hand-written per-direction init functions.

void power_up_down_redrivers(GPIO_PinState reset) {
    printf("Enabling redrivers %d if 1, disabling if 0\n", reset == GPIO_PIN_SET);
    reset = (reset == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(GPU_CPU_PD_0_3_GPIO_Port, GPU_CPU_PD_0_3_Pin, reset);
    HAL_GPIO_WritePin(GPU_CPU_PD_4_7_GPIO_Port, GPU_CPU_PD_4_7_Pin, reset);
    HAL_GPIO_WritePin(CPU_GPU_PD_0_3_GPIO_Port, CPU_GPU_PD_0_3_Pin, reset);
    HAL_GPIO_WritePin(CPU_GPU_PD_4_7_GPIO_Port, CPU_GPU_PD_4_7_Pin, reset);
}

void print_redriver_channel_status(uint16_t redriver_address, uint16_t channel_register_address) {
    uint8_t status;
    HAL_I2C_Mem_Read(&hi2c2, (redriver_address << 1) | REGISTER_READ_MASK, channel_register_address, I2C_MEMADD_SIZE_8BIT, &status, 1, 500);
    printf("Redriver 0x%02X channel 0x%02X status: 0x%02X\n", redriver_address, channel_register_address, status);
}

void print_redriver_status(uint16_t redriver_address) {
    print_redriver_channel_status(redriver_address, CHANNEL_0_REGISTER);
    print_redriver_channel_status(redriver_address, CHANNEL_1_REGISTER);
    print_redriver_channel_status(redriver_address, CHANNEL_2_REGISTER);
    print_redriver_channel_status(redriver_address, CHANNEL_3_REGISTER);
    print_redriver_channel_status(redriver_address, CHANNEL_4_REGISTER);
    print_redriver_channel_status(redriver_address, CHANNEL_5_REGISTER);
    print_redriver_channel_status(redriver_address, CHANNEL_6_REGISTER);
    print_redriver_channel_status(redriver_address, CHANNEL_7_REGISTER);
}

void print_redrivers_status() {
    status_poll_ticks_delay++;
    if (status_poll_ticks_delay < 10000) {
        return;
    }
    status_poll_ticks_delay = 0;
    printf("Redriver status:\n");
    print_redriver_status(CPU_GPU_0_3_ADDR_I2C);
    print_redriver_status(CPU_GPU_4_7_ADDR_I2C);
    print_redriver_status(GPU_CPU_0_3_ADDR_I2C);
    print_redriver_status(GPU_CPU_4_7_ADDR_I2C);
}


// ============================================================================
// Boot -> running RAMP  (per-channel, per-field)
//
// Each channel has a boot operating point and a run operating point (redriver_
// setting_t). Every walked field - eq_stage1 / eq_stage2 (reg 0x01), eq_profile
// (0x03[6:3]), flat gain (0x03[2:0]) and bias (0x06[5:3]) - moves independently,
// one value per tick, from its boot value toward its run value. The stage-1 bypass
// bit (eq_stage1[7]) is discrete: disabled while ramping (unless both endpoints
// bypass), then set to the run value by the final snap. A channel is done once its
// furthest-apart field has arrived; then
// redriver_apply_running() re-asserts the exact run point so it matches exactly.
// ============================================================================

// Per-channel ramp progress. step[i]: -1 = idle/done; 1.. = active (number of
// steps taken; a channel finishes when every property has reached its running
// value). next[i] = tick at which channel i applies its next step. Thread only.
static int      redriver_ramp_step[NUM_RAMP_CHANNELS];
static uint32_t redriver_ramp_next[NUM_RAMP_CHANNELS];
static int      redriver_ramp_active = 0;

// Move from 'from' toward 'to' by 'step' units, clamped so it never overshoots.
static int redriver_ramp_walk(int from, int to, int step) {
    if (to >= from) {
        int v = from + step;
        return (v > to) ? to : v;
    }
    int v = from - step;
    return (v < to) ? to : v;
}

// |a - b| for small ints.
static int redriver_ramp_dist(int a, int b) {
    int d = a - b;
    return (d < 0) ? -d : d;
}

// Compose and write one operating point to a channel: 0x01 = eq_stage1 (incl bypass) |
// eq_stage2, 0x03 = eq_profile | flat_gain, 0x06[5:3] = bias. Fields are pre-positioned so
// only the required bits of each register are set.
static void redriver_write_setting(uint16_t addr, uint16_t chan, const redriver_setting_t *s) {
    initialize_channel_eq(addr, chan, (uint8_t)(s->eq_stage1 | s->eq_stage2));
    initialize_channel_eq_profile(addr, chan, (uint8_t)(s->eq_profile | s->flat_gain));
    initialize_channel_bias(addr, chan, (uint8_t)(s->bias << 3));
}

// Largest per-field distance between two operating points = number of ramp steps.
// The stage-1 bypass bit is excluded (discrete: handled separately, not walked).
static int redriver_setting_dist(const redriver_setting_t *a, const redriver_setting_t *b) {
    int d = redriver_ramp_dist(EQ_MAG_6_3(a->eq_stage1), EQ_MAG_6_3(b->eq_stage1));
    int e = redriver_ramp_dist(a->eq_stage2, b->eq_stage2);                       if (e > d) d = e;
    e = redriver_ramp_dist(EQ_MAG_6_3(a->eq_profile), EQ_MAG_6_3(b->eq_profile)); if (e > d) d = e;
    e = redriver_ramp_dist(a->flat_gain, b->flat_gain);                           if (e > d) d = e;
    e = redriver_ramp_dist(a->bias,      b->bias);                                if (e > d) d = e;
    return d;
}

// Power every channel up or down per its .enabled flag, then load each enabled
// channel's BOOT (training) registers: RX detect + the boot operating point.
// This is the starting point the ramp walks from (ramp step 0).
static void redriver_apply_boot(void) {
    for (unsigned int i = 0; i < NUM_RAMP_CHANNELS; i++) {
        ramp_channel_t *c = &ramp_channels[i];
        initialize_channel_pd(c->addr, c->chan, c->enabled ? POWER_UP_CHANNEL : POWER_DOWN_CHANNEL);
    }
    for (unsigned int i = 0; i < NUM_RAMP_CHANNELS; i++) {
        ramp_channel_t *c = &ramp_channels[i];
        if (!c->enabled) {
            continue;
        }
        // mr_rx_det_man forces "RX termination always detected" so the redriver keeps
        // driving even if the far-end detect is marginal; en_rx_det_count left set too.
        initialize_channel_rx_detect(c->addr, c->chan, MR_RX_DET_MAN | EN_RX_DET_COUNT);
        redriver_write_setting(c->addr, c->chan, &c->boot);
    }
}

// Re-assert every enabled channel's exact RUNNING operating point (the ramp endpoint).
static void redriver_apply_running(void) {
    for (unsigned int i = 0; i < NUM_RAMP_CHANNELS; i++) {
        ramp_channel_t *c = &ramp_channels[i];
        if (!c->enabled) {
            continue;
        }
        redriver_write_setting(c->addr, c->chan, &c->run);
    }
}

// Set the redrivers up and arm the boot->running ramp. Waits for the four banks on
// I2C, applies every channel's boot config (power + training EQ = ramp step 0), then
// schedules each enabled channel to hold boot for REDRIVER_BOOT_HOLD_MS (+ its lane
// stagger) before walking to running. Disabled channels are powered down and skipped.
void redriver_ramp_start(void) {
    HAL_I2C_IsDeviceReady(&hi2c2, CPU_GPU_0_3_ADDR_I2C << 1, 1024, 1000);
    HAL_I2C_IsDeviceReady(&hi2c2, CPU_GPU_4_7_ADDR_I2C << 1, 1024, 1000);
    HAL_I2C_IsDeviceReady(&hi2c2, GPU_CPU_0_3_ADDR_I2C << 1, 1024, 1000);
    HAL_I2C_IsDeviceReady(&hi2c2, GPU_CPU_4_7_ADDR_I2C << 1, 1024, 1000);

    redriver_apply_boot();

    uint32_t now = HAL_GetTick();
    for (unsigned int i = 0; i < NUM_RAMP_CHANNELS; i++) {
        if (!ramp_channels[i].enabled) {
            redriver_ramp_step[i] = -1;   // powered down: nothing to ramp
            continue;
        }
        redriver_ramp_step[i] = 1;        // step 0 = boot (just applied); start moving
        redriver_ramp_next[i] = now + REDRIVER_BOOT_HOLD_MS
                                    + ramp_channels[i].rampup_delay_ms
                                    + ramp_channels[i].rampup_step_ms;
    }
    redriver_ramp_active = 1;
    printf("Redriver boot config applied; boot->running ramp armed\n");
}

void redriver_ramp_cancel(void) {
    redriver_ramp_active = 0;
    for (unsigned int i = 0; i < NUM_RAMP_CHANNELS; i++) {
        redriver_ramp_step[i] = -1;
    }
}

// 1 while a boot->running ramp is in progress (keep calling redriver_ramp_pump),
// 0 once it is idle or has finished.
int redriver_ramp_is_active(void) {
    return redriver_ramp_active;
}

// Advance every channel that's due for a step. Returns 1 when the ramp is not in
// progress (already finished this call, or was idle), 0 while it is still ramping.
int redriver_ramp_pump(void) {
    if (!redriver_ramp_active) {
        return 1;                       // nothing in progress
    }

    uint32_t now = HAL_GetTick();
    int any_active = 0;

    for (unsigned int i = 0; i < NUM_RAMP_CHANNELS; i++) {
        int step = redriver_ramp_step[i];
        if (step < 0) {
            continue;                       // channel already finished
        }
        any_active = 1;
        if ((int32_t)(now - redriver_ramp_next[i]) < 0) {
            continue;                       // not time for this channel's next step
        }

        ramp_channel_t *c = &ramp_channels[i];
        const redriver_setting_t *b = &c->boot, *r = &c->run;

        // Each field's magnitude walks boot -> run independently; the channel is done once
        // the furthest-apart field has arrived. Stage-1 bypass is disabled while ramping unless
        // BOTH endpoints are bypassed (so an up-ramp from a bypassed level actually boosts); the
        // final snap applies the exact run bypass.
        int max_dist = redriver_setting_dist(b, r);
        redriver_setting_t cur = {
            .eq_stage1  = (uint8_t)((redriver_ramp_walk(EQ_MAG_6_3(b->eq_stage1), EQ_MAG_6_3(r->eq_stage1), step) << 3)
                                    | (b->eq_stage1 & r->eq_stage1 & EQ_STAGE1_BYPASS)),
            .eq_stage2  = (uint8_t)redriver_ramp_walk(b->eq_stage2, r->eq_stage2, step),
            .eq_profile = (uint8_t)(redriver_ramp_walk(EQ_MAG_6_3(b->eq_profile), EQ_MAG_6_3(r->eq_profile), step) << 3),
            .flat_gain  = (uint8_t)redriver_ramp_walk(b->flat_gain, r->flat_gain, step),
            .bias       = (uint8_t)redriver_ramp_walk(b->bias, r->bias, step),
        };
        redriver_write_setting(c->addr, c->chan, &cur);

        if (step >= max_dist) {
            redriver_ramp_step[i] = -1;     // every property has reached running
        } else {
            redriver_ramp_step[i] = step + 1;
            redriver_ramp_next[i] = redriver_ramp_next[i] + c->rampup_step_ms;
        }
    }

    if (!any_active) {
        // Every channel has landed on its running endpoint; re-assert the exact
        // running config once and stop.
        redriver_apply_running();
        redriver_ramp_active = 0;
        printf("Redriver ramp complete\n");
        return 1;                       // ramp just finished
    }

    return 0;                           // still ramping
}
