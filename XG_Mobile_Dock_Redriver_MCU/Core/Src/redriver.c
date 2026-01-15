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
#define EQ_CONTROL_EQ_STAGE1_BYPASS 0b10000000
#define EQ_CONTROL_EQ_STAGE1_3 0b01000000
#define EQ_CONTROL_EQ_STAGE1_2 0b00100000
#define EQ_CONTROL_EQ_STAGE1_1 0b00010000
#define EQ_CONTROL_EQ_STAGE1_0 0b00001000
#define EQ_CONTROL_EQ_STAGE2_2 0b00000100
#define EQ_CONTROL_EQ_STAGE2_1 0b00000010
#define EQ_CONTROL_EQ_STAGE2_0 0b00000001

#define EQ_GAIN_FLAT_GAIN_REGISTER_OFFSET 0x03
#define EQ_PROFILE_3 0b01000000
#define EQ_PROFILE_2 0b00100000
#define EQ_PROFILE_1 0b00010000
#define EQ_PROFILE_0 0b00001000
#define FLAT_GAIN_2 0b00000100
#define FLAT_GAIN_1 0b00000010
#define FLAT_GAIN_0 0b00000001


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


#define EQ_CONTROL_EQ_STAGE1_INDEX_0 0
#define EQ_CONTROL_EQ_STAGE1_INDEX_1 1
#define EQ_CONTROL_EQ_STAGE1_INDEX_2 3
#define EQ_CONTROL_EQ_STAGE1_INDEX_5 0
#define EQ_CONTROL_EQ_STAGE1_INDEX_6 1
#define EQ_CONTROL_EQ_STAGE1_INDEX_7 2
#define EQ_CONTROL_EQ_STAGE1_INDEX_8 3
#define EQ_CONTROL_EQ_STAGE1_INDEX_9 4
#define EQ_CONTROL_EQ_STAGE1_INDEX_10 5
#define EQ_CONTROL_EQ_STAGE1_INDEX_11 6
#define EQ_CONTROL_EQ_STAGE1_INDEX_12 8
#define EQ_CONTROL_EQ_STAGE1_INDEX_13 10
#define EQ_CONTROL_EQ_STAGE1_INDEX_14 10
#define EQ_CONTROL_EQ_STAGE1_INDEX_15 11
#define EQ_CONTROL_EQ_STAGE1_INDEX_16 12
#define EQ_CONTROL_EQ_STAGE1_INDEX_17 13
#define EQ_CONTROL_EQ_STAGE1_INDEX_18 14
#define EQ_CONTROL_EQ_STAGE1_INDEX_19 15

#define EQ_CONTROL_EQ_STAGE2_INDEX_0 0
#define EQ_CONTROL_EQ_STAGE2_INDEX_1 0
#define EQ_CONTROL_EQ_STAGE2_INDEX_2 0
#define EQ_CONTROL_EQ_STAGE2_INDEX_5 0
#define EQ_CONTROL_EQ_STAGE2_INDEX_6 0
#define EQ_CONTROL_EQ_STAGE2_INDEX_7 0
#define EQ_CONTROL_EQ_STAGE2_INDEX_8 0
#define EQ_CONTROL_EQ_STAGE2_INDEX_9 0
#define EQ_CONTROL_EQ_STAGE2_INDEX_10 1 << 4
#define EQ_CONTROL_EQ_STAGE2_INDEX_11 1 << 4
#define EQ_CONTROL_EQ_STAGE2_INDEX_12 1 << 4
#define EQ_CONTROL_EQ_STAGE2_INDEX_13 1 << 4
#define EQ_CONTROL_EQ_STAGE2_INDEX_14 2 << 4
#define EQ_CONTROL_EQ_STAGE2_INDEX_15 3 << 4
#define EQ_CONTROL_EQ_STAGE2_INDEX_16 4 << 4
#define EQ_CONTROL_EQ_STAGE2_INDEX_17 5 << 4
#define EQ_CONTROL_EQ_STAGE2_INDEX_18 6 << 4
#define EQ_CONTROL_EQ_STAGE2_INDEX_19 7 << 4

#define EQ_PROFILE_INDEX_0 0
#define EQ_PROFILE_INDEX_1 0
#define EQ_PROFILE_INDEX_2 0
#define EQ_PROFILE_INDEX_5 1 << 3
#define EQ_PROFILE_INDEX_6 1 << 3
#define EQ_PROFILE_INDEX_7 1 << 3
#define EQ_PROFILE_INDEX_8 3 << 3
#define EQ_PROFILE_INDEX_9 3 << 3
#define EQ_PROFILE_INDEX_10 7 << 3
#define EQ_PROFILE_INDEX_11 7 << 3
#define EQ_PROFILE_INDEX_12 7 << 3
#define EQ_PROFILE_INDEX_13 7 << 3
#define EQ_PROFILE_INDEX_14 15 << 3
#define EQ_PROFILE_INDEX_15 15 << 3
#define EQ_PROFILE_INDEX_16 15 << 3
#define EQ_PROFILE_INDEX_17 15 << 3
#define EQ_PROFILE_INDEX_18 15 << 3
#define EQ_PROFILE_INDEX_19 15 << 3

#define EQ_STAGE_1_BYPASS_INDEX_0 1 << 7
#define EQ_STAGE_1_BYPASS_INDEX_1 1 << 7
#define EQ_STAGE_1_BYPASS_INDEX_2 1 << 7
#define EQ_STAGE_1_BYPASS_INDEX_5 0
#define EQ_STAGE_1_BYPASS_INDEX_6 0
#define EQ_STAGE_1_BYPASS_INDEX_7 0
#define EQ_STAGE_1_BYPASS_INDEX_8 0
#define EQ_STAGE_1_BYPASS_INDEX_9 0
#define EQ_STAGE_1_BYPASS_INDEX_10 0
#define EQ_STAGE_1_BYPASS_INDEX_11 0
#define EQ_STAGE_1_BYPASS_INDEX_12 0
#define EQ_STAGE_1_BYPASS_INDEX_13 0
#define EQ_STAGE_1_BYPASS_INDEX_14 0
#define EQ_STAGE_1_BYPASS_INDEX_15 0
#define EQ_STAGE_1_BYPASS_INDEX_16 0
#define EQ_STAGE_1_BYPASS_INDEX_17 0
#define EQ_STAGE_1_BYPASS_INDEX_18 0
#define EQ_STAGE_1_BYPASS_INDEX_19 0

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


static void initialize_gpu_cpu_redriver_boot()
{
    printf("Initializing gpu to cpu redriver to boot state\n");

    initialize_channel_pd(GPU_CPU_0_3_ADDR_I2C, CHANNEL_0_REGISTER, POWER_UP_CHANNEL);
    initialize_channel_pd(GPU_CPU_0_3_ADDR_I2C, CHANNEL_1_REGISTER, POWER_UP_CHANNEL);
    initialize_channel_pd(GPU_CPU_0_3_ADDR_I2C, CHANNEL_2_REGISTER, POWER_UP_CHANNEL);
    initialize_channel_pd(GPU_CPU_0_3_ADDR_I2C, CHANNEL_3_REGISTER, POWER_UP_CHANNEL);
    initialize_channel_pd(GPU_CPU_4_7_ADDR_I2C, CHANNEL_4_REGISTER, POWER_UP_CHANNEL);
    initialize_channel_pd(GPU_CPU_4_7_ADDR_I2C, CHANNEL_5_REGISTER, POWER_UP_CHANNEL);
    initialize_channel_pd(GPU_CPU_4_7_ADDR_I2C, CHANNEL_6_REGISTER, POWER_UP_CHANNEL);
    initialize_channel_pd(GPU_CPU_4_7_ADDR_I2C, CHANNEL_7_REGISTER, POWER_UP_CHANNEL);

    initialize_channel_eq(GPU_CPU_0_3_ADDR_I2C, CHANNEL_0_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_19 | EQ_CONTROL_EQ_STAGE2_INDEX_19);
    initialize_channel_eq(GPU_CPU_0_3_ADDR_I2C, CHANNEL_1_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_19 | EQ_CONTROL_EQ_STAGE2_INDEX_19);
    initialize_channel_eq(GPU_CPU_0_3_ADDR_I2C, CHANNEL_2_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_19 | EQ_CONTROL_EQ_STAGE2_INDEX_19);
    initialize_channel_eq(GPU_CPU_0_3_ADDR_I2C, CHANNEL_3_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_19 | EQ_CONTROL_EQ_STAGE2_INDEX_19);
    initialize_channel_eq(GPU_CPU_4_7_ADDR_I2C, CHANNEL_4_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_19 | EQ_CONTROL_EQ_STAGE2_INDEX_19);
    initialize_channel_eq(GPU_CPU_4_7_ADDR_I2C, CHANNEL_5_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_19 | EQ_CONTROL_EQ_STAGE2_INDEX_19);
    initialize_channel_eq(GPU_CPU_4_7_ADDR_I2C, CHANNEL_6_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_19 | EQ_CONTROL_EQ_STAGE2_INDEX_13);
    initialize_channel_eq(GPU_CPU_4_7_ADDR_I2C, CHANNEL_7_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_19 | EQ_CONTROL_EQ_STAGE2_INDEX_16);

    initialize_channel_rx_detect(GPU_CPU_0_3_ADDR_I2C, CHANNEL_0_REGISTER, EN_RX_DET_COUNT);
    initialize_channel_rx_detect(GPU_CPU_0_3_ADDR_I2C, CHANNEL_1_REGISTER, EN_RX_DET_COUNT);
    initialize_channel_rx_detect(GPU_CPU_0_3_ADDR_I2C, CHANNEL_2_REGISTER, EN_RX_DET_COUNT);
    initialize_channel_rx_detect(GPU_CPU_0_3_ADDR_I2C, CHANNEL_3_REGISTER, EN_RX_DET_COUNT);
    initialize_channel_rx_detect(GPU_CPU_4_7_ADDR_I2C, CHANNEL_4_REGISTER, EN_RX_DET_COUNT);
    initialize_channel_rx_detect(GPU_CPU_4_7_ADDR_I2C, CHANNEL_5_REGISTER, EN_RX_DET_COUNT);
    initialize_channel_rx_detect(GPU_CPU_4_7_ADDR_I2C, CHANNEL_6_REGISTER, EN_RX_DET_COUNT);
    initialize_channel_rx_detect(GPU_CPU_4_7_ADDR_I2C, CHANNEL_7_REGISTER, EN_RX_DET_COUNT);

    initialize_channel_bias(GPU_CPU_0_3_ADDR_I2C, CHANNEL_0_REGISTER, BIAS_CURRENT_0);
    initialize_channel_bias(GPU_CPU_0_3_ADDR_I2C, CHANNEL_1_REGISTER, BIAS_CURRENT_0);
    initialize_channel_bias(GPU_CPU_0_3_ADDR_I2C, CHANNEL_2_REGISTER, BIAS_CURRENT_0);
    initialize_channel_bias(GPU_CPU_0_3_ADDR_I2C, CHANNEL_3_REGISTER, BIAS_CURRENT_0);
    initialize_channel_bias(GPU_CPU_4_7_ADDR_I2C, CHANNEL_4_REGISTER, BIAS_CURRENT_0);
    initialize_channel_bias(GPU_CPU_4_7_ADDR_I2C, CHANNEL_5_REGISTER, BIAS_CURRENT_0);
    initialize_channel_bias(GPU_CPU_4_7_ADDR_I2C, CHANNEL_6_REGISTER, BIAS_CURRENT_0);
    initialize_channel_bias(GPU_CPU_4_7_ADDR_I2C, CHANNEL_7_REGISTER, BIAS_CURRENT_0);

    initialize_channel_eq_profile(GPU_CPU_0_3_ADDR_I2C, CHANNEL_0_REGISTER, EQ_PROFILE_INDEX_19);
    initialize_channel_eq_profile(GPU_CPU_0_3_ADDR_I2C, CHANNEL_1_REGISTER, EQ_PROFILE_INDEX_19);
    initialize_channel_eq_profile(GPU_CPU_0_3_ADDR_I2C, CHANNEL_2_REGISTER, EQ_PROFILE_INDEX_19);
    initialize_channel_eq_profile(GPU_CPU_0_3_ADDR_I2C, CHANNEL_3_REGISTER, EQ_PROFILE_INDEX_19);
    initialize_channel_eq_profile(GPU_CPU_4_7_ADDR_I2C, CHANNEL_4_REGISTER, EQ_PROFILE_INDEX_19);
    initialize_channel_eq_profile(GPU_CPU_4_7_ADDR_I2C, CHANNEL_5_REGISTER, EQ_PROFILE_INDEX_19);
    initialize_channel_eq_profile(GPU_CPU_4_7_ADDR_I2C, CHANNEL_6_REGISTER, EQ_PROFILE_INDEX_19);
    initialize_channel_eq_profile(GPU_CPU_4_7_ADDR_I2C, CHANNEL_7_REGISTER, EQ_PROFILE_INDEX_19);
}

static void initialize_cpu_gpu_redriver_boot()
{
    printf("Initializing cpu to gpu redriver to boot state\n");

    initialize_channel_pd(CPU_GPU_0_3_ADDR_I2C, CHANNEL_0_REGISTER, POWER_UP_CHANNEL);
    initialize_channel_pd(CPU_GPU_0_3_ADDR_I2C, CHANNEL_1_REGISTER, POWER_UP_CHANNEL);
    initialize_channel_pd(CPU_GPU_0_3_ADDR_I2C, CHANNEL_2_REGISTER, POWER_UP_CHANNEL);
    initialize_channel_pd(CPU_GPU_0_3_ADDR_I2C, CHANNEL_3_REGISTER, POWER_UP_CHANNEL);
    initialize_channel_pd(CPU_GPU_4_7_ADDR_I2C, CHANNEL_4_REGISTER, POWER_UP_CHANNEL);
    initialize_channel_pd(CPU_GPU_4_7_ADDR_I2C, CHANNEL_5_REGISTER, POWER_UP_CHANNEL);
    initialize_channel_pd(CPU_GPU_4_7_ADDR_I2C, CHANNEL_6_REGISTER, POWER_UP_CHANNEL);
    initialize_channel_pd(CPU_GPU_4_7_ADDR_I2C, CHANNEL_7_REGISTER, POWER_UP_CHANNEL);

    initialize_channel_eq(CPU_GPU_0_3_ADDR_I2C, CHANNEL_0_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_6 | EQ_CONTROL_EQ_STAGE2_INDEX_6);
    initialize_channel_eq(CPU_GPU_0_3_ADDR_I2C, CHANNEL_1_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_6 | EQ_CONTROL_EQ_STAGE2_INDEX_6);
    initialize_channel_eq(CPU_GPU_0_3_ADDR_I2C, CHANNEL_2_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_6 | EQ_CONTROL_EQ_STAGE2_INDEX_6);
    initialize_channel_eq(CPU_GPU_0_3_ADDR_I2C, CHANNEL_3_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_6 | EQ_CONTROL_EQ_STAGE2_INDEX_6);
    initialize_channel_eq(CPU_GPU_4_7_ADDR_I2C, CHANNEL_4_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_6 | EQ_CONTROL_EQ_STAGE2_INDEX_6);
    initialize_channel_eq(CPU_GPU_4_7_ADDR_I2C, CHANNEL_5_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_12 | EQ_CONTROL_EQ_STAGE2_INDEX_12);
    initialize_channel_eq(CPU_GPU_4_7_ADDR_I2C, CHANNEL_6_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_13 | EQ_CONTROL_EQ_STAGE2_INDEX_13);
    initialize_channel_eq(CPU_GPU_4_7_ADDR_I2C, CHANNEL_7_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_6 | EQ_CONTROL_EQ_STAGE2_INDEX_6);

    initialize_channel_rx_detect(CPU_GPU_0_3_ADDR_I2C, CHANNEL_0_REGISTER, EN_RX_DET_COUNT);
    initialize_channel_rx_detect(CPU_GPU_0_3_ADDR_I2C, CHANNEL_1_REGISTER, EN_RX_DET_COUNT);
    initialize_channel_rx_detect(CPU_GPU_0_3_ADDR_I2C, CHANNEL_2_REGISTER, EN_RX_DET_COUNT);
    initialize_channel_rx_detect(CPU_GPU_0_3_ADDR_I2C, CHANNEL_3_REGISTER, EN_RX_DET_COUNT);
    initialize_channel_rx_detect(CPU_GPU_4_7_ADDR_I2C, CHANNEL_4_REGISTER, EN_RX_DET_COUNT);
    initialize_channel_rx_detect(CPU_GPU_4_7_ADDR_I2C, CHANNEL_5_REGISTER, EN_RX_DET_COUNT);
    initialize_channel_rx_detect(CPU_GPU_4_7_ADDR_I2C, CHANNEL_6_REGISTER, EN_RX_DET_COUNT);
    initialize_channel_rx_detect(CPU_GPU_4_7_ADDR_I2C, CHANNEL_7_REGISTER, EN_RX_DET_COUNT);

    initialize_channel_bias(CPU_GPU_0_3_ADDR_I2C, CHANNEL_0_REGISTER, BIAS_CURRENT_0);
    initialize_channel_bias(CPU_GPU_0_3_ADDR_I2C, CHANNEL_1_REGISTER, BIAS_CURRENT_0);
    initialize_channel_bias(CPU_GPU_0_3_ADDR_I2C, CHANNEL_2_REGISTER, BIAS_CURRENT_0);
    initialize_channel_bias(CPU_GPU_0_3_ADDR_I2C, CHANNEL_3_REGISTER, BIAS_CURRENT_0);
    initialize_channel_bias(CPU_GPU_4_7_ADDR_I2C, CHANNEL_4_REGISTER, BIAS_CURRENT_0);
    initialize_channel_bias(CPU_GPU_4_7_ADDR_I2C, CHANNEL_5_REGISTER, BIAS_CURRENT_0);
    initialize_channel_bias(CPU_GPU_4_7_ADDR_I2C, CHANNEL_6_REGISTER, BIAS_CURRENT_0);
    initialize_channel_bias(CPU_GPU_4_7_ADDR_I2C, CHANNEL_7_REGISTER, BIAS_CURRENT_0);

    initialize_channel_eq_profile(CPU_GPU_0_3_ADDR_I2C, CHANNEL_0_REGISTER, EQ_PROFILE_INDEX_6 | FLAT_GAIN_0 | FLAT_GAIN_2);
    initialize_channel_eq_profile(CPU_GPU_0_3_ADDR_I2C, CHANNEL_1_REGISTER, EQ_PROFILE_INDEX_6 | FLAT_GAIN_0 | FLAT_GAIN_2);
    initialize_channel_eq_profile(CPU_GPU_0_3_ADDR_I2C, CHANNEL_2_REGISTER, EQ_PROFILE_INDEX_6 | FLAT_GAIN_0 | FLAT_GAIN_2);
    initialize_channel_eq_profile(CPU_GPU_0_3_ADDR_I2C, CHANNEL_3_REGISTER, EQ_PROFILE_INDEX_6 | FLAT_GAIN_0 | FLAT_GAIN_2);
    initialize_channel_eq_profile(CPU_GPU_4_7_ADDR_I2C, CHANNEL_4_REGISTER, EQ_PROFILE_INDEX_6 | FLAT_GAIN_0 | FLAT_GAIN_2);
    initialize_channel_eq_profile(CPU_GPU_4_7_ADDR_I2C, CHANNEL_5_REGISTER, EQ_PROFILE_INDEX_12 | FLAT_GAIN_0 | FLAT_GAIN_2);
    initialize_channel_eq_profile(CPU_GPU_4_7_ADDR_I2C, CHANNEL_6_REGISTER, EQ_PROFILE_INDEX_13 | FLAT_GAIN_0 | FLAT_GAIN_2);
    initialize_channel_eq_profile(CPU_GPU_4_7_ADDR_I2C, CHANNEL_7_REGISTER, EQ_PROFILE_INDEX_6 | FLAT_GAIN_0 | FLAT_GAIN_2);
}



static void initialize_gpu_cpu_redriver_running()
{
    printf("Initializing gpu to cpu redriver to running state\n");

    initialize_channel_eq(GPU_CPU_0_3_ADDR_I2C, CHANNEL_0_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_12 | EQ_CONTROL_EQ_STAGE2_INDEX_12);
    initialize_channel_eq(GPU_CPU_0_3_ADDR_I2C, CHANNEL_1_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_12 | EQ_CONTROL_EQ_STAGE2_INDEX_12);
    initialize_channel_eq(GPU_CPU_0_3_ADDR_I2C, CHANNEL_2_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_12 | EQ_CONTROL_EQ_STAGE2_INDEX_12);
    initialize_channel_eq(GPU_CPU_0_3_ADDR_I2C, CHANNEL_3_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_12 | EQ_CONTROL_EQ_STAGE2_INDEX_12);
    initialize_channel_eq(GPU_CPU_4_7_ADDR_I2C, CHANNEL_4_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_12 | EQ_CONTROL_EQ_STAGE2_INDEX_12);
    initialize_channel_eq(GPU_CPU_4_7_ADDR_I2C, CHANNEL_5_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_5 | EQ_CONTROL_EQ_STAGE2_INDEX_9);
    initialize_channel_eq(GPU_CPU_4_7_ADDR_I2C, CHANNEL_6_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_5 | EQ_CONTROL_EQ_STAGE2_INDEX_7);
    initialize_channel_eq(GPU_CPU_4_7_ADDR_I2C, CHANNEL_7_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_12 | EQ_CONTROL_EQ_STAGE2_INDEX_12);

    initialize_channel_rx_detect(GPU_CPU_0_3_ADDR_I2C, CHANNEL_0_REGISTER, EN_RX_DET_COUNT | SEL_RX_DET_COUNT);
    initialize_channel_rx_detect(GPU_CPU_0_3_ADDR_I2C, CHANNEL_1_REGISTER, EN_RX_DET_COUNT | SEL_RX_DET_COUNT);
    initialize_channel_rx_detect(GPU_CPU_0_3_ADDR_I2C, CHANNEL_2_REGISTER, EN_RX_DET_COUNT | SEL_RX_DET_COUNT);
    initialize_channel_rx_detect(GPU_CPU_0_3_ADDR_I2C, CHANNEL_3_REGISTER, EN_RX_DET_COUNT | SEL_RX_DET_COUNT);
    initialize_channel_rx_detect(GPU_CPU_4_7_ADDR_I2C, CHANNEL_4_REGISTER, EN_RX_DET_COUNT | SEL_RX_DET_COUNT);
    initialize_channel_rx_detect(GPU_CPU_4_7_ADDR_I2C, CHANNEL_5_REGISTER, EN_RX_DET_COUNT | SEL_RX_DET_COUNT);
    initialize_channel_rx_detect(GPU_CPU_4_7_ADDR_I2C, CHANNEL_6_REGISTER, EN_RX_DET_COUNT | SEL_RX_DET_COUNT);
    initialize_channel_rx_detect(GPU_CPU_4_7_ADDR_I2C, CHANNEL_7_REGISTER, EN_RX_DET_COUNT | SEL_RX_DET_COUNT);

    initialize_channel_bias(GPU_CPU_0_3_ADDR_I2C, CHANNEL_0_REGISTER, BIAS_CURRENT_0);
    initialize_channel_bias(GPU_CPU_0_3_ADDR_I2C, CHANNEL_1_REGISTER, BIAS_CURRENT_0);
    initialize_channel_bias(GPU_CPU_0_3_ADDR_I2C, CHANNEL_2_REGISTER, BIAS_CURRENT_0);
    initialize_channel_bias(GPU_CPU_0_3_ADDR_I2C, CHANNEL_3_REGISTER, BIAS_CURRENT_0);
    initialize_channel_bias(GPU_CPU_4_7_ADDR_I2C, CHANNEL_4_REGISTER, BIAS_CURRENT_0);
    initialize_channel_bias(GPU_CPU_4_7_ADDR_I2C, CHANNEL_5_REGISTER, BIAS_CURRENT_0);
    initialize_channel_bias(GPU_CPU_4_7_ADDR_I2C, CHANNEL_6_REGISTER, BIAS_CURRENT_0);
    initialize_channel_bias(GPU_CPU_4_7_ADDR_I2C, CHANNEL_7_REGISTER, BIAS_CURRENT_0);

    initialize_channel_eq_profile(GPU_CPU_0_3_ADDR_I2C, CHANNEL_0_REGISTER, EQ_PROFILE_INDEX_12 | FLAT_GAIN_1);
    initialize_channel_eq_profile(GPU_CPU_0_3_ADDR_I2C, CHANNEL_1_REGISTER, EQ_PROFILE_INDEX_12 | FLAT_GAIN_1);
    initialize_channel_eq_profile(GPU_CPU_0_3_ADDR_I2C, CHANNEL_2_REGISTER, EQ_PROFILE_INDEX_12 | FLAT_GAIN_1);
    initialize_channel_eq_profile(GPU_CPU_0_3_ADDR_I2C, CHANNEL_3_REGISTER, EQ_PROFILE_INDEX_12 | FLAT_GAIN_1);
    initialize_channel_eq_profile(GPU_CPU_4_7_ADDR_I2C, CHANNEL_4_REGISTER, EQ_PROFILE_INDEX_12 | FLAT_GAIN_1);
    initialize_channel_eq_profile(GPU_CPU_4_7_ADDR_I2C, CHANNEL_5_REGISTER, EQ_PROFILE_INDEX_8 | FLAT_GAIN_0);
    initialize_channel_eq_profile(GPU_CPU_4_7_ADDR_I2C, CHANNEL_6_REGISTER, EQ_PROFILE_INDEX_19 | FLAT_GAIN_0);
    initialize_channel_eq_profile(GPU_CPU_4_7_ADDR_I2C, CHANNEL_7_REGISTER, EQ_PROFILE_INDEX_12 | FLAT_GAIN_1);
}

static void initialize_cpu_gpu_redriver_running()
{
    printf("Initializing cpu to gpu redriver to running state\n");

    initialize_channel_eq(CPU_GPU_0_3_ADDR_I2C, CHANNEL_0_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_6 | EQ_CONTROL_EQ_STAGE2_INDEX_6);
    initialize_channel_eq(CPU_GPU_0_3_ADDR_I2C, CHANNEL_1_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_6 | EQ_CONTROL_EQ_STAGE2_INDEX_6);
    initialize_channel_eq(CPU_GPU_0_3_ADDR_I2C, CHANNEL_2_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_6 | EQ_CONTROL_EQ_STAGE2_INDEX_6);
    initialize_channel_eq(CPU_GPU_0_3_ADDR_I2C, CHANNEL_3_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_6 | EQ_CONTROL_EQ_STAGE2_INDEX_6);
    initialize_channel_eq(CPU_GPU_4_7_ADDR_I2C, CHANNEL_4_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_6 | EQ_CONTROL_EQ_STAGE2_INDEX_6);
    initialize_channel_eq(CPU_GPU_4_7_ADDR_I2C, CHANNEL_5_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_6 | EQ_CONTROL_EQ_STAGE2_INDEX_6);
    initialize_channel_eq(CPU_GPU_4_7_ADDR_I2C, CHANNEL_6_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_6 | EQ_CONTROL_EQ_STAGE2_INDEX_6);
    initialize_channel_eq(CPU_GPU_4_7_ADDR_I2C, CHANNEL_7_REGISTER, EQ_CONTROL_EQ_STAGE1_INDEX_6 | EQ_CONTROL_EQ_STAGE2_INDEX_6);

    initialize_channel_bias(CPU_GPU_0_3_ADDR_I2C, CHANNEL_0_REGISTER, BIAS_CURRENT_0);
    initialize_channel_bias(CPU_GPU_0_3_ADDR_I2C, CHANNEL_1_REGISTER, BIAS_CURRENT_0);
    initialize_channel_bias(CPU_GPU_0_3_ADDR_I2C, CHANNEL_2_REGISTER, BIAS_CURRENT_0);
    initialize_channel_bias(CPU_GPU_0_3_ADDR_I2C, CHANNEL_3_REGISTER, BIAS_CURRENT_0);
    initialize_channel_bias(CPU_GPU_4_7_ADDR_I2C, CHANNEL_4_REGISTER, BIAS_CURRENT_0);
    initialize_channel_bias(CPU_GPU_4_7_ADDR_I2C, CHANNEL_5_REGISTER, BIAS_CURRENT_0);
    initialize_channel_bias(CPU_GPU_4_7_ADDR_I2C, CHANNEL_6_REGISTER, BIAS_CURRENT_0);
    initialize_channel_bias(CPU_GPU_4_7_ADDR_I2C, CHANNEL_7_REGISTER, BIAS_CURRENT_0);

    initialize_channel_eq_profile(CPU_GPU_0_3_ADDR_I2C, CHANNEL_0_REGISTER, EQ_PROFILE_INDEX_6 | FLAT_GAIN_0 | FLAT_GAIN_2);
    initialize_channel_eq_profile(CPU_GPU_0_3_ADDR_I2C, CHANNEL_1_REGISTER, EQ_PROFILE_INDEX_6 | FLAT_GAIN_0 | FLAT_GAIN_2);
    initialize_channel_eq_profile(CPU_GPU_0_3_ADDR_I2C, CHANNEL_2_REGISTER, EQ_PROFILE_INDEX_6 | FLAT_GAIN_0 | FLAT_GAIN_2);
    initialize_channel_eq_profile(CPU_GPU_0_3_ADDR_I2C, CHANNEL_3_REGISTER, EQ_PROFILE_INDEX_6 | FLAT_GAIN_0 | FLAT_GAIN_2);
    initialize_channel_eq_profile(CPU_GPU_4_7_ADDR_I2C, CHANNEL_4_REGISTER, EQ_PROFILE_INDEX_6 | FLAT_GAIN_0 | FLAT_GAIN_2);
    initialize_channel_eq_profile(CPU_GPU_4_7_ADDR_I2C, CHANNEL_5_REGISTER, EQ_PROFILE_INDEX_5 | FLAT_GAIN_0 | FLAT_GAIN_1 | FLAT_GAIN_2);
    initialize_channel_eq_profile(CPU_GPU_4_7_ADDR_I2C, CHANNEL_6_REGISTER, EQ_PROFILE_INDEX_5 | FLAT_GAIN_0 | FLAT_GAIN_1 | FLAT_GAIN_2);
    initialize_channel_eq_profile(CPU_GPU_4_7_ADDR_I2C, CHANNEL_7_REGISTER, EQ_PROFILE_INDEX_6 | FLAT_GAIN_0 | FLAT_GAIN_2);
}

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


void initialize_redrivers_boot() {
    printf("Waiting for the redrivers to be ready\n");
    HAL_I2C_IsDeviceReady(&hi2c2, CPU_GPU_0_3_ADDR_I2C << 1, 1024, 1000);
    HAL_I2C_IsDeviceReady(&hi2c2, CPU_GPU_4_7_ADDR_I2C << 1, 1024, 1000);
    HAL_I2C_IsDeviceReady(&hi2c2, GPU_CPU_0_3_ADDR_I2C << 1, 1024, 1000);
    HAL_I2C_IsDeviceReady(&hi2c2, GPU_CPU_4_7_ADDR_I2C << 1, 1024, 1000);
    printf("Redrivers are ready\n");

    initialize_gpu_cpu_redriver_boot();
    initialize_cpu_gpu_redriver_boot();
}

void initialize_redrivers_running() {
    initialize_gpu_cpu_redriver_running();
    initialize_cpu_gpu_redriver_running();
}
