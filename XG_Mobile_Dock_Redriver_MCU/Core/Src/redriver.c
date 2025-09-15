#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stm32f0xx_hal_i2c.h>

extern I2C_HandleTypeDef hi2c2;

#define CPU_GPU_0_3_ADDR_I2C 0x18
#define CPU_GPU_4_7_ADDR_I2C 0x19
#define GPU_CPU_0_3_ADDR_I2C 0x22
#define GPU_CPU_4_7_ADDR_I2C 0x23

#define CHANNEL_0_REGISTER 0x00
#define CHANNEL_1_REGISTER 0x20
#define CHANNEL_2_REGISTER 0x40
#define CHANNEL_3_REGISTER 0x60
#define CHANNEL_4_REGISTER 0x00
#define CHANNEL_5_REGISTER 0x20
#define CHANNEL_6_REGISTER 0x40
#define CHANNEL_7_REGISTER 0x60

#define EQ_CONTROL_REGISTER_OFFSET 0x01
#define EQ_CONTROL_EQ_STAGE1_BYPASS 0x80
#define EQ_CONTROL_EQ_STAGE1_3 0x40
#define EQ_CONTROL_EQ_STAGE1_2 0x20
#define EQ_CONTROL_EQ_STAGE1_1 0x10
#define EQ_CONTROL_EQ_STAGE1_0 0x08
#define EQ_CONTROL_EQ_STAGE2_2 0x04
#define EQ_CONTROL_EQ_STAGE2_1 0x02
#define EQ_CONTROL_EQ_STAGE2_0 0x01

#define EQ_GAIN_FLAT_GAIN_REGISTER_OFFSET 0x03
#define EQ_PROFILE_3 0x40
#define EQ_PROFILE_2 0x20
#define EQ_PROFILE_1 0x10
#define EQ_PROFILE_0 0x08
#define FLAT_GAIN_2 0x04
#define FLAT_GAIN_1 0x02
#define FLAT_GAIN_0 0x01


#define RX_DETECT_CONTROL_REGISTER_OFFSET 0x04
#define MR_RX_DET_MAN 0x04 //force always detect
#define EN_RX_DET_COUNT 0x02 //enable additional rx detect polling
#define SEL_RX_DET_COUNT 0x01 //enable additional rx detect 0 = 2 valid detections, 1 = 3 additional valid detections


#define BIAS_REGISTER_OFFSET 0x06
#define BIAS_CURRENT_2 0x20
#define BIAS_CURRENT_1 0x10
#define BIAS_CURRENT_0 0x08

static void initialize_channel_eq(uint16_t redriver_address, uint16_t channel_register_address, uint8_t eq_index) {
    HAL_I2C_Mem_Write_IT(&hi2c2, redriver_address, channel_register_address + EQ_CONTROL_REGISTER_OFFSET, I2C_MEMADD_SIZE_8BIT, &eq_index, 1);
}

static void initialize_channel_rx_detect(uint16_t redriver_address, uint16_t channel_register_address, uint8_t rx_detect) {
    HAL_I2C_Mem_Write_IT(&hi2c2, redriver_address, channel_register_address + RX_DETECT_CONTROL_REGISTER_OFFSET, I2C_MEMADD_SIZE_8BIT, &rx_detect, 1);
}

static void initialize_channel_bias(uint16_t redriver_address, uint16_t channel_register_address, uint8_t bias) {
    HAL_I2C_Mem_Write_IT(&hi2c2, redriver_address, channel_register_address + BIAS_REGISTER_OFFSET, I2C_MEMADD_SIZE_8BIT, &bias, 1);
}

static void initialize_channel_eq_profile(uint16_t redriver_address, uint16_t channel_register_address, uint8_t eq_profile) {
    HAL_I2C_Mem_Write_IT(&hi2c2, redriver_address, channel_register_address + EQ_GAIN_FLAT_GAIN_REGISTER_OFFSET, I2C_MEMADD_SIZE_8BIT, &eq_profile, 1);
}


static void initialize_gpu_cpu_redriver()
{
    initialize_channel_eq(GPU_CPU_0_3_ADDR_I2C, CHANNEL_0_REGISTER, EQ_CONTROL_EQ_STAGE2_2 | EQ_CONTROL_EQ_STAGE1_2);
    initialize_channel_eq(GPU_CPU_0_3_ADDR_I2C, CHANNEL_1_REGISTER, EQ_CONTROL_EQ_STAGE2_2 | EQ_CONTROL_EQ_STAGE1_2);
    initialize_channel_eq(GPU_CPU_0_3_ADDR_I2C, CHANNEL_2_REGISTER, EQ_CONTROL_EQ_STAGE2_2 | EQ_CONTROL_EQ_STAGE1_2);
    initialize_channel_eq(GPU_CPU_0_3_ADDR_I2C, CHANNEL_3_REGISTER, EQ_CONTROL_EQ_STAGE2_2 | EQ_CONTROL_EQ_STAGE1_2);
    initialize_channel_eq(GPU_CPU_4_7_ADDR_I2C, CHANNEL_4_REGISTER, EQ_CONTROL_EQ_STAGE2_2 | EQ_CONTROL_EQ_STAGE1_1);
    initialize_channel_eq(GPU_CPU_4_7_ADDR_I2C, CHANNEL_5_REGISTER, EQ_CONTROL_EQ_STAGE2_2 | EQ_CONTROL_EQ_STAGE1_1);
    initialize_channel_eq(GPU_CPU_4_7_ADDR_I2C, CHANNEL_6_REGISTER, EQ_CONTROL_EQ_STAGE2_2 | EQ_CONTROL_EQ_STAGE1_1);
    initialize_channel_eq(GPU_CPU_4_7_ADDR_I2C, CHANNEL_7_REGISTER, EQ_CONTROL_EQ_STAGE2_2 | EQ_CONTROL_EQ_STAGE1_1);

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

    initialize_channel_eq_profile(GPU_CPU_0_3_ADDR_I2C, CHANNEL_0_REGISTER, EQ_PROFILE_3 | FLAT_GAIN_0 | FLAT_GAIN_2);
    initialize_channel_eq_profile(GPU_CPU_0_3_ADDR_I2C, CHANNEL_1_REGISTER, EQ_PROFILE_3 | FLAT_GAIN_0 | FLAT_GAIN_2);
    initialize_channel_eq_profile(GPU_CPU_0_3_ADDR_I2C, CHANNEL_2_REGISTER, EQ_PROFILE_3 | FLAT_GAIN_0 | FLAT_GAIN_2);
    initialize_channel_eq_profile(GPU_CPU_0_3_ADDR_I2C, CHANNEL_3_REGISTER, EQ_PROFILE_3 | FLAT_GAIN_0 | FLAT_GAIN_2);
    initialize_channel_eq_profile(GPU_CPU_4_7_ADDR_I2C, CHANNEL_4_REGISTER, EQ_PROFILE_3 | FLAT_GAIN_0 | FLAT_GAIN_2);
    initialize_channel_eq_profile(GPU_CPU_4_7_ADDR_I2C, CHANNEL_5_REGISTER, EQ_PROFILE_3 | FLAT_GAIN_0 | FLAT_GAIN_2);
    initialize_channel_eq_profile(GPU_CPU_4_7_ADDR_I2C, CHANNEL_6_REGISTER, EQ_PROFILE_3 | FLAT_GAIN_0 | FLAT_GAIN_2);
    initialize_channel_eq_profile(GPU_CPU_4_7_ADDR_I2C, CHANNEL_7_REGISTER, EQ_PROFILE_3 | FLAT_GAIN_0 | FLAT_GAIN_2);
}

static void initialize_cpu_gpu_redriver()
{
    initialize_channel_eq(CPU_GPU_0_3_ADDR_I2C, CHANNEL_0_REGISTER, EQ_CONTROL_EQ_STAGE2_0 | EQ_CONTROL_EQ_STAGE1_BYPASS);
    initialize_channel_eq(CPU_GPU_0_3_ADDR_I2C, CHANNEL_1_REGISTER, EQ_CONTROL_EQ_STAGE2_0 | EQ_CONTROL_EQ_STAGE1_BYPASS);
    initialize_channel_eq(CPU_GPU_0_3_ADDR_I2C, CHANNEL_2_REGISTER, EQ_CONTROL_EQ_STAGE2_0 | EQ_CONTROL_EQ_STAGE1_BYPASS);
    initialize_channel_eq(CPU_GPU_0_3_ADDR_I2C, CHANNEL_3_REGISTER, EQ_CONTROL_EQ_STAGE2_0 | EQ_CONTROL_EQ_STAGE1_BYPASS);
    initialize_channel_eq(CPU_GPU_4_7_ADDR_I2C, CHANNEL_4_REGISTER, EQ_CONTROL_EQ_STAGE2_0 | EQ_CONTROL_EQ_STAGE1_BYPASS);
    initialize_channel_eq(CPU_GPU_4_7_ADDR_I2C, CHANNEL_5_REGISTER, EQ_CONTROL_EQ_STAGE2_0 | EQ_CONTROL_EQ_STAGE1_BYPASS);
    initialize_channel_eq(CPU_GPU_4_7_ADDR_I2C, CHANNEL_6_REGISTER, EQ_CONTROL_EQ_STAGE2_0 | EQ_CONTROL_EQ_STAGE1_BYPASS);
    initialize_channel_eq(CPU_GPU_4_7_ADDR_I2C, CHANNEL_7_REGISTER, EQ_CONTROL_EQ_STAGE2_0 | EQ_CONTROL_EQ_STAGE1_BYPASS);

    initialize_channel_rx_detect(CPU_GPU_0_3_ADDR_I2C, CHANNEL_0_REGISTER, MR_RX_DET_MAN);
    initialize_channel_rx_detect(CPU_GPU_0_3_ADDR_I2C, CHANNEL_1_REGISTER, MR_RX_DET_MAN);
    initialize_channel_rx_detect(CPU_GPU_0_3_ADDR_I2C, CHANNEL_2_REGISTER, MR_RX_DET_MAN);
    initialize_channel_rx_detect(CPU_GPU_0_3_ADDR_I2C, CHANNEL_3_REGISTER, MR_RX_DET_MAN);
    initialize_channel_rx_detect(CPU_GPU_4_7_ADDR_I2C, CHANNEL_4_REGISTER, MR_RX_DET_MAN);
    initialize_channel_rx_detect(CPU_GPU_4_7_ADDR_I2C, CHANNEL_5_REGISTER, MR_RX_DET_MAN);
    initialize_channel_rx_detect(CPU_GPU_4_7_ADDR_I2C, CHANNEL_6_REGISTER, MR_RX_DET_MAN);
    initialize_channel_rx_detect(CPU_GPU_4_7_ADDR_I2C, CHANNEL_7_REGISTER, MR_RX_DET_MAN);

    initialize_channel_bias(CPU_GPU_0_3_ADDR_I2C, CHANNEL_0_REGISTER, BIAS_CURRENT_1);
    initialize_channel_bias(CPU_GPU_0_3_ADDR_I2C, CHANNEL_1_REGISTER, BIAS_CURRENT_1);
    initialize_channel_bias(CPU_GPU_0_3_ADDR_I2C, CHANNEL_2_REGISTER, BIAS_CURRENT_1);
    initialize_channel_bias(CPU_GPU_0_3_ADDR_I2C, CHANNEL_3_REGISTER, BIAS_CURRENT_1);
    initialize_channel_bias(CPU_GPU_4_7_ADDR_I2C, CHANNEL_4_REGISTER, BIAS_CURRENT_1);
    initialize_channel_bias(CPU_GPU_4_7_ADDR_I2C, CHANNEL_5_REGISTER, BIAS_CURRENT_1);
    initialize_channel_bias(CPU_GPU_4_7_ADDR_I2C, CHANNEL_6_REGISTER, BIAS_CURRENT_1);
    initialize_channel_bias(CPU_GPU_4_7_ADDR_I2C, CHANNEL_7_REGISTER, BIAS_CURRENT_1);

    initialize_channel_eq_profile(CPU_GPU_0_3_ADDR_I2C, CHANNEL_0_REGISTER, EQ_PROFILE_0 | FLAT_GAIN_0 | FLAT_GAIN_1);
    initialize_channel_eq_profile(CPU_GPU_0_3_ADDR_I2C, CHANNEL_1_REGISTER, EQ_PROFILE_0 | FLAT_GAIN_0 | FLAT_GAIN_1);
    initialize_channel_eq_profile(CPU_GPU_0_3_ADDR_I2C, CHANNEL_2_REGISTER, EQ_PROFILE_0 | FLAT_GAIN_0 | FLAT_GAIN_1);
    initialize_channel_eq_profile(CPU_GPU_0_3_ADDR_I2C, CHANNEL_3_REGISTER, EQ_PROFILE_0 | FLAT_GAIN_0 | FLAT_GAIN_1);
    initialize_channel_eq_profile(CPU_GPU_4_7_ADDR_I2C, CHANNEL_4_REGISTER, EQ_PROFILE_0 | FLAT_GAIN_0 | FLAT_GAIN_1);
    initialize_channel_eq_profile(CPU_GPU_4_7_ADDR_I2C, CHANNEL_5_REGISTER, EQ_PROFILE_0 | FLAT_GAIN_0 | FLAT_GAIN_1);
    initialize_channel_eq_profile(CPU_GPU_4_7_ADDR_I2C, CHANNEL_6_REGISTER, EQ_PROFILE_0 | FLAT_GAIN_0 | FLAT_GAIN_1);
    initialize_channel_eq_profile(CPU_GPU_4_7_ADDR_I2C, CHANNEL_7_REGISTER, EQ_PROFILE_0 | FLAT_GAIN_0 | FLAT_GAIN_1);
}


void initialize_redrivers() {
    HAL_I2C_IsDeviceReady(&hi2c2, CPU_GPU_0_3_ADDR_I2C, 1024, 1);
    HAL_I2C_IsDeviceReady(&hi2c2, CPU_GPU_4_7_ADDR_I2C, 1024, 1);
    HAL_I2C_IsDeviceReady(&hi2c2, GPU_CPU_0_3_ADDR_I2C, 1024, 1);
    HAL_I2C_IsDeviceReady(&hi2c2, GPU_CPU_0_3_ADDR_I2C, 1024, 1);

    initialize_gpu_cpu_redriver();
    initialize_cpu_gpu_redriver();
}
