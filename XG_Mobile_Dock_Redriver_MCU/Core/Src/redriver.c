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

// Per-EQ-level register values (Table 7-1). One pair per level (levels 3 & 4 don't exist):
//   EQ_CTRL_n    = reg 0x01 = (stage1 << 3) | stage2 | (bypass << 7)
//   EQ_PROFILE_n = reg 0x03[6:3] = (profile << 3)   (flat gain is OR'd in at write time)
#define EQ_CTRL_0     ((0  << 3) | 0 | (1 << 7))
#define EQ_CTRL_1     ((1  << 3) | 0 | (1 << 7))
#define EQ_CTRL_2     ((3  << 3) | 0 | (1 << 7))
#define EQ_CTRL_5     ((0  << 3) | 0 | (0 << 7))
#define EQ_CTRL_6     ((1  << 3) | 0 | (0 << 7))
#define EQ_CTRL_7     ((2  << 3) | 0 | (0 << 7))
#define EQ_CTRL_8     ((3  << 3) | 0 | (0 << 7))
#define EQ_CTRL_9     ((4  << 3) | 0 | (0 << 7))
#define EQ_CTRL_10    ((5  << 3) | 1 | (0 << 7))
#define EQ_CTRL_11    ((6  << 3) | 1 | (0 << 7))
#define EQ_CTRL_12    ((8  << 3) | 1 | (0 << 7))
#define EQ_CTRL_13    ((10 << 3) | 1 | (0 << 7))
#define EQ_CTRL_14    ((10 << 3) | 2 | (0 << 7))
#define EQ_CTRL_15    ((11 << 3) | 3 | (0 << 7))
#define EQ_CTRL_16    ((12 << 3) | 4 | (0 << 7))
#define EQ_CTRL_17    ((13 << 3) | 5 | (0 << 7))
#define EQ_CTRL_18    ((14 << 3) | 6 | (0 << 7))
#define EQ_CTRL_19    ((15 << 3) | 7 | (0 << 7))

#define EQ_PROFILE_0   (0  << 3)
#define EQ_PROFILE_1   (0  << 3)
#define EQ_PROFILE_2   (0  << 3)
#define EQ_PROFILE_5   (1  << 3)
#define EQ_PROFILE_6   (1  << 3)
#define EQ_PROFILE_7   (1  << 3)
#define EQ_PROFILE_8   (3  << 3)
#define EQ_PROFILE_9   (3  << 3)
#define EQ_PROFILE_10  (7  << 3)
#define EQ_PROFILE_11  (7  << 3)
#define EQ_PROFILE_12  (7  << 3)
#define EQ_PROFILE_13  (7  << 3)
#define EQ_PROFILE_14  (15 << 3)
#define EQ_PROFILE_15  (15 << 3)
#define EQ_PROFILE_16  (15 << 3)
#define EQ_PROFILE_17  (15 << 3)
#define EQ_PROFILE_18  (15 << 3)
#define EQ_PROFILE_19  (15 << 3)

// EQ rampup progression: levels ordered LOWEST boost -> HIGHEST. The ramp walks this
// list one entry at a time between a channel's boot and running levels. Each entry
// holds the composed registers for that level (the same values the init uses).
typedef struct {
    uint8_t level;       // Table 7-1 index (identifies boot/running in the sequence)
    uint8_t eq_ctrl;     // reg 0x01
    uint8_t eq_profile;  // reg 0x03[6:3] profile (flat gain OR'd in at write time)
} eq_step_t;

static const eq_step_t eq_rampup_sequence[] = {
    {  0, EQ_CTRL_0,  EQ_PROFILE_0  },
    {  1, EQ_CTRL_1,  EQ_PROFILE_1  },
    {  2, EQ_CTRL_2,  EQ_PROFILE_2  },
    {  5, EQ_CTRL_5,  EQ_PROFILE_5  },
    {  6, EQ_CTRL_6,  EQ_PROFILE_6  },
    {  7, EQ_CTRL_7,  EQ_PROFILE_7  },
    {  8, EQ_CTRL_8,  EQ_PROFILE_8  },
    {  9, EQ_CTRL_9,  EQ_PROFILE_9  },
    { 10, EQ_CTRL_10, EQ_PROFILE_10 },
    { 11, EQ_CTRL_11, EQ_PROFILE_11 },
    { 12, EQ_CTRL_12, EQ_PROFILE_12 },
    { 13, EQ_CTRL_13, EQ_PROFILE_13 },
    { 14, EQ_CTRL_14, EQ_PROFILE_14 },
    { 15, EQ_CTRL_15, EQ_PROFILE_15 },
    { 16, EQ_CTRL_16, EQ_PROFILE_16 },
    { 17, EQ_CTRL_17, EQ_PROFILE_17 },
    { 18, EQ_CTRL_18, EQ_PROFILE_18 },
    { 19, EQ_CTRL_19, EQ_PROFILE_19 },
};
#define NUM_EQ_STEPS (sizeof(eq_rampup_sequence) / sizeof(eq_rampup_sequence[0]))

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
// Boot -> running RAMP  (per-channel, sequence-walked)
//
// The EQ progression is a STATIC ORDERED SEQUENCE of levels (eq_rampup_sequence,
// below), lowest boost -> highest. Each channel names a boot EQ level and a
// running EQ level; the ramp finds both in the sequence and walks it one entry at
// a time between them, applying every level along the way. Flat gain (0x03[2:0])
// and bias (0x06[5:3]) walk their own 0..7 ranges the same way, one value/step.
// Each property advances one step per tick until it reaches its running value;
// the channel is done once all three have arrived, then redriver_apply_running()
// is re-asserted so the end state matches exactly.
// ============================================================================
typedef struct {
    uint16_t addr;
    uint16_t chan;
    uint8_t  enabled;          // 1 = power this channel up + configure + ramp it; 0 = power it down
    uint8_t  boot_eq_level;    // EQ level (Table 7-1 index) held at boot (training)
    uint8_t  run_eq_level;     // EQ level at the running state
    uint8_t  boot_flat_gain;   // 0x03[2:0]  (5 = 0 dB, 0 = -6 dB)
    uint8_t  run_flat_gain;
    uint8_t  boot_bias;        // 0x06[5:3]  (001b..111b)
    uint8_t  run_bias;
    uint16_t rampup_step_ms;   // ms between steps (one sequence entry / value per step)
    uint16_t rampup_delay_ms;  // extra per-lane hold before ramping (on top of the boot hold; stagger lanes)
} ramp_channel_t;

// Every channel holds its boot (training) config this long before the ramp to the
// running state begins. This is the PCIe link-training window.
#define REDRIVER_BOOT_HOLD_MS 5000

// Per-lane ramp stagger: channels begin their boot->running ramp one after another
// (not all at once) after the boot hold. Start order is 0,1,2,3,4,5,7,6 - channel 6
// goes LAST - set by each row's delay = rank * RAMP_STAGGER_MS in the table below.
#define RAMP_STAGGER_MS 500

// Per-channel setup + ramp endpoints (the single source of truth for redriver config).
// en = 1 enables the channel (power up + configure + ramp); 0 powers it down. EQ is a
// level walked through eq_rampup_sequence; flat gain + bias are single fields walked
// over 0..7 (bias 1 = 001b .. 7 = 111b). step_ms = ms per walked value; delay = per-lane
// stagger on top of REDRIVER_BOOT_HOLD_MS (rank * RAMP_STAGGER_MS): start order is
// 0,1,2,3,4,5,7,6 so channel 6 ramps LAST.
//   addr, chan,   en  boot_eq run_eq  boot_fg            run_fg             boot_bias run_bias  step_ms delay
static ramp_channel_t ramp_channels[] = {
    { GPU_CPU_0_3_ADDR_I2C, CHANNEL_0_REGISTER, 1, 19, 0, FLAT_GAIN_LEVEL_5, FLAT_GAIN_LEVEL_0, 1, 7,   25, 0 * RAMP_STAGGER_MS },
    { GPU_CPU_0_3_ADDR_I2C, CHANNEL_1_REGISTER, 1, 19, 0, FLAT_GAIN_LEVEL_5, FLAT_GAIN_LEVEL_0, 1, 7,   25, 1 * RAMP_STAGGER_MS },
    { GPU_CPU_0_3_ADDR_I2C, CHANNEL_2_REGISTER, 1, 19, 0, FLAT_GAIN_LEVEL_5, FLAT_GAIN_LEVEL_0, 1, 7,   25, 2 * RAMP_STAGGER_MS },
    { GPU_CPU_0_3_ADDR_I2C, CHANNEL_3_REGISTER, 1, 19, 0, FLAT_GAIN_LEVEL_5, FLAT_GAIN_LEVEL_0, 1, 7,   25, 3 * RAMP_STAGGER_MS },
    { GPU_CPU_4_7_ADDR_I2C, CHANNEL_4_REGISTER, 1,  0, 0, FLAT_GAIN_LEVEL_5, FLAT_GAIN_LEVEL_0, 1, 7,   25, 4 * RAMP_STAGGER_MS },
    { GPU_CPU_4_7_ADDR_I2C, CHANNEL_5_REGISTER, 1,  0, 0, FLAT_GAIN_LEVEL_5, FLAT_GAIN_LEVEL_0, 1, 7,   25, 5 * RAMP_STAGGER_MS },
    { GPU_CPU_4_7_ADDR_I2C, CHANNEL_6_REGISTER, 1,  0, 0, FLAT_GAIN_LEVEL_5, FLAT_GAIN_LEVEL_0, 1, 7,   25, 7 * RAMP_STAGGER_MS },
    { GPU_CPU_4_7_ADDR_I2C, CHANNEL_7_REGISTER, 1,  0, 0, FLAT_GAIN_LEVEL_5, FLAT_GAIN_LEVEL_0, 1, 7,   25, 6 * RAMP_STAGGER_MS },
    { CPU_GPU_0_3_ADDR_I2C, CHANNEL_0_REGISTER, 1,  6, 6, FLAT_GAIN_LEVEL_5, FLAT_GAIN_LEVEL_5, 1, 1,   25, 0 * RAMP_STAGGER_MS },
    { CPU_GPU_0_3_ADDR_I2C, CHANNEL_1_REGISTER, 1,  6, 6, FLAT_GAIN_LEVEL_5, FLAT_GAIN_LEVEL_5, 1, 1,   25, 1 * RAMP_STAGGER_MS },
    { CPU_GPU_0_3_ADDR_I2C, CHANNEL_2_REGISTER, 1,  6, 6, FLAT_GAIN_LEVEL_5, FLAT_GAIN_LEVEL_5, 1, 1,   25, 2 * RAMP_STAGGER_MS },
    { CPU_GPU_0_3_ADDR_I2C, CHANNEL_3_REGISTER, 1,  6, 6, FLAT_GAIN_LEVEL_5, FLAT_GAIN_LEVEL_5, 1, 1,   25, 3 * RAMP_STAGGER_MS },
    { CPU_GPU_4_7_ADDR_I2C, CHANNEL_4_REGISTER, 1,  6, 6, FLAT_GAIN_LEVEL_5, FLAT_GAIN_LEVEL_5, 1, 1,   25, 4 * RAMP_STAGGER_MS },
    { CPU_GPU_4_7_ADDR_I2C, CHANNEL_5_REGISTER, 1,  6, 6, FLAT_GAIN_LEVEL_5, FLAT_GAIN_LEVEL_5, 1, 1,   25, 5 * RAMP_STAGGER_MS },
    { CPU_GPU_4_7_ADDR_I2C, CHANNEL_6_REGISTER, 1,  6, 6, FLAT_GAIN_LEVEL_5, FLAT_GAIN_LEVEL_5, 1, 1,   25, 7 * RAMP_STAGGER_MS },
    { CPU_GPU_4_7_ADDR_I2C, CHANNEL_7_REGISTER, 1,  6, 6, FLAT_GAIN_LEVEL_5, FLAT_GAIN_LEVEL_5, 1, 1,   25, 6 * RAMP_STAGGER_MS },
};
#define NUM_RAMP_CHANNELS (sizeof(ramp_channels) / sizeof(ramp_channels[0]))

// Per-channel ramp progress. step[i]: -1 = idle/done; 1.. = active (number of
// steps taken; a channel finishes when every property has reached its running
// value). next[i] = tick at which channel i applies its next step. Thread only.
static int      redriver_ramp_step[NUM_RAMP_CHANNELS];
static uint32_t redriver_ramp_next[NUM_RAMP_CHANNELS];
static int      redriver_ramp_active = 0;

// Index of an EQ level within the rampup sequence (falls back to 0 if not found).
static int eq_level_index(uint8_t level) {
    for (unsigned int i = 0; i < NUM_EQ_STEPS; i++) {
        if (eq_rampup_sequence[i].level == level) {
            return (int)i;
        }
    }
    return 0;
}

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

// Power every channel up or down per its .enabled flag, then load each enabled
// channel's BOOT (training) registers: RX detect + boot EQ / flat gain / bias.
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
        const eq_step_t *e = &eq_rampup_sequence[eq_level_index(c->boot_eq_level)];
        initialize_channel_rx_detect(c->addr, c->chan, EN_RX_DET_COUNT);
        initialize_channel_eq(c->addr, c->chan, e->eq_ctrl);
        initialize_channel_eq_profile(c->addr, c->chan, (uint8_t)(e->eq_profile | c->boot_flat_gain));
        initialize_channel_bias(c->addr, c->chan, (uint8_t)(c->boot_bias << 3));
    }
}

// Re-assert every enabled channel's exact RUNNING registers (the ramp endpoint).
static void redriver_apply_running(void) {
    for (unsigned int i = 0; i < NUM_RAMP_CHANNELS; i++) {
        ramp_channel_t *c = &ramp_channels[i];
        if (!c->enabled) {
            continue;
        }
        const eq_step_t *e = &eq_rampup_sequence[eq_level_index(c->run_eq_level)];
        initialize_channel_eq(c->addr, c->chan, e->eq_ctrl);
        initialize_channel_eq_profile(c->addr, c->chan, (uint8_t)(e->eq_profile | c->run_flat_gain));
        initialize_channel_bias(c->addr, c->chan, (uint8_t)(c->run_bias << 3));
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
        int boot_idx = eq_level_index(c->boot_eq_level);
        int run_idx  = eq_level_index(c->run_eq_level);

        // Steps until every property has reached its running value.
        int max_dist = redriver_ramp_dist(run_idx, boot_idx);
        int fg_dist  = redriver_ramp_dist((int)c->run_flat_gain, (int)c->boot_flat_gain);
        int bs_dist  = redriver_ramp_dist((int)c->run_bias,      (int)c->boot_bias);
        if (fg_dist > max_dist) max_dist = fg_dist;
        if (bs_dist > max_dist) max_dist = bs_dist;

        // Where each property sits after 'step' steps toward its running value.
        int cur_idx  = redriver_ramp_walk(boot_idx, run_idx, step);
        uint8_t fg   = (uint8_t)redriver_ramp_walk((int)c->boot_flat_gain, (int)c->run_flat_gain, step);
        uint8_t bias = (uint8_t)redriver_ramp_walk((int)c->boot_bias,      (int)c->run_bias,      step);
        const eq_step_t *e = &eq_rampup_sequence[cur_idx];

        // 0x01 = eq_ctrl (bypass|stage1|stage2); 0x03 = profile | flat gain; 0x06[5:3] = bias.
        initialize_channel_eq(c->addr, c->chan, e->eq_ctrl);
        initialize_channel_eq_profile(c->addr, c->chan, (uint8_t)(e->eq_profile | fg));
        initialize_channel_bias(c->addr, c->chan, (uint8_t)(bias << 3));

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
