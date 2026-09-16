#include "main.h"
#include "stm32f4xx_it.h"

extern DMA_HandleTypeDef hdma_adc;
extern ADC_HandleTypeDef hadc;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;

void NMI_Handler(void) { while (1) {} }
void HardFault_Handler(void) { while (1) {} }
void MemManage_Handler(void) { while (1) {} }
void BusFault_Handler(void) { while (1) {} }
void UsageFault_Handler(void) { while (1) {} }
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}
void SysTick_Handler(void) { HAL_IncTick(); }

void EXTI0_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(CON_DET_Pin); }
void EXTI1_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(PWREN_Pin); }
void EXTI2_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(RST_Pin); }
void EXTI4_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(LOCK_SW_Pin); }

void DMA2_Stream0_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_adc); }
void ADC_IRQHandler(void) { HAL_ADC_IRQHandler(&hadc); }
void TIM1_UP_TIM10_IRQHandler(void) { HAL_TIM_IRQHandler(&htim1); }
void TIM1_CC_IRQHandler(void) { HAL_TIM_IRQHandler(&htim1); }
void TIM3_IRQHandler(void) { HAL_TIM_IRQHandler(&htim3); }
void TIM6_DAC_IRQHandler(void) { HAL_TIM_IRQHandler(&htim6); }
void I2C1_EV_IRQHandler(void) { HAL_I2C_EV_IRQHandler(&hi2c1); }
void I2C1_ER_IRQHandler(void) { HAL_I2C_ER_IRQHandler(&hi2c1); }