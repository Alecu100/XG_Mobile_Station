#include "main.h"
#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim3;

typedef enum {
    MCU_RESET=0,
    DEVICE_IDLE,
    CABLE_DETECT,
    CABLE_LOCK,
    POWER_OFF,
    POWER_ON,
} fsm_state_t;

typedef enum {
    I2C_STATE_IDLE,
    I2C_STATE_ACCESS_REG,
    I2C_STATE_ACCESS_SIZE,
    I2C_STATE_WAIT_DATA,
    I2C_STATE_SEND_DATA,
    I2C_STATE_WAIT_ACK,
} i2c_state_t;

typedef enum {
    NONE,
    WHITE,
    RED,
} led_colour_t;

static const char * const gFSMStateStrings[] = {
    [MCU_RESET] = "MCU_RESET",
    [DEVICE_IDLE] = "DEVICE_IDLE",
    [CABLE_DETECT] = "CABLE_DETECT",
    [CABLE_LOCK] = "CABLE_LOCK",
    [POWER_OFF] = "POWER_OFF",
    [POWER_ON] = "POWER_ON",
};

typedef struct {
    fsm_state_t fsm;
    int debounce_pins;
    volatile int lock_switch;
    volatile int connector_detect;
    volatile int power_enable;
    i2c_state_t i2c;
    unsigned char i2cBuffer[256];
    int i2cCmd;
    int i2cReg;
    int i2cSize;
    volatile unsigned char i2cRegData1;
    volatile int retimer_config_pending;
} state_t;

static state_t gState;

extern void fans_start(void);
extern void fans_stop(void);
extern HAL_StatusTypeDef retimer_configure_x8(void);
extern void retimer_print_status(void);

/* --- Non-blocking UART logging ---------------------------------------------
 * printf() is called from ISR context (the EXTI callback and every I2C slave
 * callback). The old __io_putchar() used HAL_UART_Transmit() with a blocking
 * 0xFFFF timeout, which parks the calling ISR for ~87 us per byte at 115200
 * baud (milliseconds per log line). That stall is what makes the MCU miss
 * I2C/EXTI events. Instead we push bytes into a ring buffer and drain it with
 * interrupt-driven UART TX, so logging never busy-waits on the UART hardware.
 *
 * Behaviour when the buffer is full:
 *   - ISR context   : drop the byte, never block an interrupt handler.
 *   - thread context: wait for the TX ISR to free a slot. Blocking the main
 *                     loop is acceptable and keeps main-context logs lossless,
 *                     matching the previous behaviour.
 */
#define UART_TX_BUF_SIZE 1024u /* must stay a power of two */
static volatile uint8_t  uart_tx_buf[UART_TX_BUF_SIZE];
static volatile uint16_t uart_tx_head;        /* producer index */
static volatile uint16_t uart_tx_tail;        /* consumer index */
static volatile uint16_t uart_tx_sending_len; /* bytes handed to HAL this transfer */
static volatile uint8_t  uart_tx_busy;        /* a HAL_UART_Transmit_IT is in flight */

/* Start the next contiguous chunk if the UART is idle. Must run with interrupts
 * disabled, or from the USART3 ISR where the busy flag serialises starts. */
static void uart_tx_start_locked(void) {
    if (uart_tx_busy) {
        return;
    }
    if (uart_tx_head == uart_tx_tail) {
        return; /* nothing queued */
    }
    uint16_t len;
    if (uart_tx_head > uart_tx_tail) {
        len = uart_tx_head - uart_tx_tail;
    } else {
        len = UART_TX_BUF_SIZE - uart_tx_tail; /* only up to the buffer wrap */
    }
    uart_tx_sending_len = len;
    uart_tx_busy = 1;
    if (HAL_UART_Transmit_IT(&huart3, (uint8_t *)&uart_tx_buf[uart_tx_tail], len) != HAL_OK) {
        uart_tx_busy = 0; /* could not start now; retry on the next kick */
    }
}

static void uart_tx_kick(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uart_tx_start_locked();
    __set_PRIMASK(primask);
}

/* USART3 TX-complete: advance past the chunk we just sent and start the next.
 * Runs in the USART3 ISR. sending_len/tail are stable here because uart_tx_busy
 * stays 1 until we clear it, which blocks any competing start. */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart != &huart3) {
        return;
    }
    uart_tx_tail = (uint16_t)((uart_tx_tail + uart_tx_sending_len) & (UART_TX_BUF_SIZE - 1u));
    uart_tx_busy = 0;
    uart_tx_start_locked();
}

/* USART3 global interrupt. CubeMX did not generate this handler (the UART global
 * interrupt is not enabled in the .ioc), so define it here to override the weak
 * Default_Handler from the startup file. The NVIC line is enabled in main()
 * (USER CODE BEGIN 2). */
void USART3_IRQHandler(void) {
    HAL_UART_IRQHandler(&huart3);
}

/* Enqueue one byte for transmission. Returns 0 on success, -1 if dropped. */
static int uart_tx_putc(uint8_t c) {
    for (;;) {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        uint16_t next = (uint16_t)((uart_tx_head + 1u) & (UART_TX_BUF_SIZE - 1u));
        if (next != uart_tx_tail) {
            uart_tx_buf[uart_tx_head] = c;
            uart_tx_head = next;
            uart_tx_start_locked();
            __set_PRIMASK(primask);
            return 0;
        }
        __set_PRIMASK(primask);
        /* Buffer full. */
        if (__get_IPSR() != 0U) {
            return -1; /* in an ISR: drop, never block */
        }
        /* Thread context: keep the UART draining and spin until a slot frees.
         * Interrupts are enabled here so the USART3 TX ISR can run. */
        uart_tx_kick();
    }
}

int __io_putchar(int ch) {
    if (ch == '\n') {
        uart_tx_putc((uint8_t)'\r');
    }
    uart_tx_putc((uint8_t)ch);
    return ch;
}

// PS_ON is driven through a transistor that pulls PS_ON# low when the MCU pin is high:
//   MCU HIGH -> transistor ON  -> PS_ON# LOW  -> PSU ON
//   MCU LOW  -> transistor OFF -> PS_ON# HIGH -> PSU OFF
void set_psu(int on) {
    HAL_GPIO_WritePin(PSON_GPIO_Port, PSON_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// Downstream PERST# is driven through an inverting transistor (PERST_MCU# -> Q -> PERST#):
//   MCU HIGH -> transistor ON  -> PERST# LOW  (reset asserted)
//   MCU LOW  -> transistor OFF -> PERST# HIGH (reset deasserted)
// host_reset is the active-low RST read from the host (GPIO_PIN_RESET = host asserting reset).
void set_perst(GPIO_PinState host_reset) {
    HAL_GPIO_WritePin(PERST_GPIO_Port, PERST_Pin,
                      host_reset == GPIO_PIN_RESET ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void set_retimer_perst(int asserted) {
    HAL_GPIO_WritePin(RETIMER_PERST_GPIO_Port, RETIMER_PERST_Pin,
                      asserted ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void mirror_clkreq(void) {
    HAL_GPIO_WritePin(RETIMER_CLKREQ_GPIO_Port, RETIMER_CLKREQ_Pin,
                      HAL_GPIO_ReadPin(CLKREQ_GPIO_Port, CLKREQ_Pin));
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == RST_Pin) {
        GPIO_PinState reset = HAL_GPIO_ReadPin(RST_GPIO_Port, RST_Pin);
        if (reset == GPIO_PIN_SET) {
            /* Keep the complete downstream path in reset until the main loop has
             * programmed and verified the retimer width. */
            set_perst(GPIO_PIN_RESET);
            set_retimer_perst(1);
            gState.retimer_config_pending = 1;
        } else {
            set_perst(GPIO_PIN_RESET);
            set_retimer_perst(1);
            gState.retimer_config_pending = 0;
        }
        printf("Pin changed: RST = %d\n", reset == GPIO_PIN_RESET);
    } else if (GPIO_Pin == PWREN_Pin) {
        gState.power_enable = HAL_GPIO_ReadPin(PWREN_GPIO_Port, PWREN_Pin) == GPIO_PIN_RESET;
        printf("Pin changed: PWREN = %d\n", gState.power_enable);
    } else {
        if (gState.debounce_pins) {
            // reset timer, wait for last bounce
            HAL_TIM_Base_Stop_IT(&htim3);
        }
        HAL_TIM_Base_Start_IT(&htim3);
        gState.debounce_pins |= GPIO_Pin;
    }
}

void HAL_TIM3_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if ((gState.debounce_pins & LOCK_SW_Pin) != 0) {
        gState.lock_switch = HAL_GPIO_ReadPin(LOCK_SW_GPIO_Port, LOCK_SW_Pin) == GPIO_PIN_SET;
        printf("Pin changed: LOCK_SW = %d\n", gState.lock_switch);
    }
    if ((gState.debounce_pins & CON_DET_Pin) != 0) {
        gState.connector_detect = HAL_GPIO_ReadPin(CON_DET_GPIO_Port, CON_DET_Pin) == GPIO_PIN_RESET;
        printf("Pin changed: CON_DET = %d\n", gState.connector_detect);
    }
    gState.debounce_pins = 0;
    HAL_TIM_Base_Stop_IT(htim);
}


void init_gpio_state(state_t *state) {
    state->lock_switch = HAL_GPIO_ReadPin(LOCK_SW_GPIO_Port, LOCK_SW_Pin) == GPIO_PIN_SET;
    printf("LOCK_SW = %d, ", state->lock_switch);
    state->connector_detect = HAL_GPIO_ReadPin(CON_DET_GPIO_Port, CON_DET_Pin) == GPIO_PIN_RESET;
    printf("CON_DET = %d, ", state->connector_detect);
    state->power_enable = HAL_GPIO_ReadPin(PWREN_GPIO_Port, PWREN_Pin) == GPIO_PIN_RESET;
    printf("PWREN = %d, ", state->power_enable);
    GPIO_PinState reset = HAL_GPIO_ReadPin(RST_GPIO_Port, RST_Pin);
    printf("RST = %d, ", reset == GPIO_PIN_RESET);
    set_perst(GPIO_PIN_RESET);
    set_retimer_perst(1);
    gState.retimer_config_pending = reset == GPIO_PIN_SET;
}

void update_cable_led(led_colour_t colour) {
    switch (colour) {
        case WHITE: {
            printf("Turn on white LED\n");
            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_WHITE_GPIO_Port, LED_WHITE_Pin, GPIO_PIN_RESET);
            set_psu(1); // cable locked -> spin up PSU
            break;
        }
        case RED: {
            printf("Turn on red LED\n");
            HAL_GPIO_WritePin(LED_WHITE_GPIO_Port, LED_WHITE_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
            // PSU already on from the preceding WHITE state (RED always follows WHITE)
            break;
        }
        case NONE:
        default: {
            printf("Turn off cable LED\n");
            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_WHITE_GPIO_Port, LED_WHITE_Pin, GPIO_PIN_SET);
            set_psu(0); // idle -> PSU off
            break;
        }
    }
}

void update_case_led(int on) {
    if (on) {
        HAL_GPIO_WritePin(CASE_LED_GPIO_Port, CASE_LED_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(CASE_LED_GPIO_Port, CASE_LED_Pin, GPIO_PIN_SET);
    }
}

void turn_power_on() {
    printf("Turning on PCIe power\n");
    HAL_GPIO_WritePin(PCI_12V_EN_GPIO_Port, PCI_12V_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BUCK_ON_GPIO_Port, BUCK_ON_Pin, GPIO_PIN_SET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(PWROK_GPIO_Port, PWROK_Pin, GPIO_PIN_SET);
    if (HAL_GPIO_ReadPin(RST_GPIO_Port, RST_Pin) == GPIO_PIN_SET) {
        gState.retimer_config_pending = 1;
    }
}

void turn_power_off() {
    printf("Turning off PCIe power\n");
    set_perst(GPIO_PIN_RESET);
    set_retimer_perst(1);
    gState.retimer_config_pending = 0;
    HAL_GPIO_WritePin(PWROK_GPIO_Port, PWROK_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BUCK_ON_GPIO_Port, BUCK_ON_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PCI_12V_EN_GPIO_Port, PCI_12V_EN_Pin, GPIO_PIN_RESET);
}

void assert_ec_irq() {
    printf("Asserting EC IRQ\n");
    HAL_GPIO_WritePin(MCU_IRQ_GPIO_Port, MCU_IRQ_Pin, GPIO_PIN_RESET);
}

void clear_ec_irq() {
    printf("Clearing EC IRQ\n");
    HAL_GPIO_WritePin(MCU_IRQ_GPIO_Port, MCU_IRQ_Pin, GPIO_PIN_SET);
}

void assert_lock_det() {
    printf("Asserting LOCK_DET\n");
    HAL_GPIO_WritePin(LOCK_DET_GPIO_Port, LOCK_DET_Pin, GPIO_PIN_SET);
}

void clear_lock_det() {
    printf("Clearing LOCK_DET\n");
    HAL_GPIO_WritePin(LOCK_DET_GPIO_Port, LOCK_DET_Pin, GPIO_PIN_RESET);
}

void transition_state(state_t *state, fsm_state_t next) {
    fsm_state_t prev = state->fsm;

    printf("Transition %s -> %s\n", gFSMStateStrings[prev], gFSMStateStrings[next]);
    // update cable LED
    if (next == CABLE_LOCK || next == POWER_OFF) {
        update_cable_led(WHITE);
        update_case_led(0);
    } else if (next == POWER_ON) {
        update_cable_led(RED);
        update_case_led(1);
    } else {
        update_cable_led(NONE);
        update_case_led(0);
    }
    // LOCK_DET
    if (next > CABLE_LOCK && prev < next) {
        assert_lock_det();
    } else if (next <= CABLE_LOCK && prev > next) {
        clear_lock_det();
    }
    // power off/on
    if (prev > POWER_OFF && next <= POWER_OFF) {
        turn_power_off();
        fans_stop();
    } else if (prev < POWER_ON && next >= POWER_ON) {
        turn_power_on();
        fans_start();
    }
    // send IRQ
    if (prev < CABLE_LOCK && next >= CABLE_LOCK) {
        assert_ec_irq();
    }
    // reset I2C state when unplugged
    if (prev == MCU_RESET || (prev > CABLE_DETECT && next <= CABLE_DETECT)) {
        state->i2cRegData1 = 0xff;
    }
    state->fsm = next;
}

void main_fsm_iteration(void) {
    fsm_state_t prev = gState.fsm;
    static uint32_t last_retimer_attempt;
    static uint32_t last_retimer_status;
    uint32_t now = HAL_GetTick();

    mirror_clkreq();
    if (gState.retimer_config_pending &&
        HAL_GPIO_ReadPin(PGOOD_BUCK_GPIO_Port, PGOOD_BUCK_Pin) == GPIO_PIN_SET &&
        (uint32_t)(now - last_retimer_attempt) >= 1000U) {
        last_retimer_attempt = now;
        if (retimer_configure_x8() == HAL_OK) {
            gState.retimer_config_pending = 0;
            set_retimer_perst(0);
            set_perst(HAL_GPIO_ReadPin(RST_GPIO_Port, RST_Pin));
        } else {
            printf("Retimer configuration failed; retrying with PCIe held in reset\n");
        }
    }
    if (!gState.retimer_config_pending &&
        (uint32_t)(now - last_retimer_status) >= 10000U &&
        HAL_GPIO_ReadPin(PGOOD_BUCK_GPIO_Port, PGOOD_BUCK_Pin) == GPIO_PIN_SET) {
        last_retimer_status = now;
        retimer_print_status();
    }

    //printf("Enter main FSM with state = %s\n", gFSMStateStrings[gState.fsm]);
    switch (prev) {
        case MCU_RESET: {
            init_gpio_state(&gState);
            transition_state(&gState, DEVICE_IDLE);
            HAL_I2C_EnableListen_IT(&hi2c1);
            break;
        }
        case DEVICE_IDLE: {
            if (gState.i2cRegData1 == 0) {
                transition_state(&gState, CABLE_DETECT);
            }
            break;
        }
        case CABLE_DETECT: {
            if (gState.i2cRegData1 != 0) {
                transition_state(&gState, DEVICE_IDLE);
            } else if (gState.connector_detect) {
                transition_state(&gState, CABLE_LOCK);
            }
            break;
        }
        case CABLE_LOCK: {
            if (gState.i2cRegData1 != 0) {
                transition_state(&gState, DEVICE_IDLE);
            } else if (!gState.connector_detect) {
                transition_state(&gState, CABLE_DETECT);
            } else if (gState.lock_switch) {
                transition_state(&gState, POWER_OFF);
            }
            break;
        }
        case POWER_OFF: {
            if (gState.i2cRegData1 != 0) {
                transition_state(&gState, DEVICE_IDLE);
            } else if (!gState.connector_detect) {
                transition_state(&gState, CABLE_DETECT);
            } else if (!gState.lock_switch) {
                transition_state(&gState, CABLE_LOCK);
            } else if (gState.power_enable) {
                transition_state(&gState, POWER_ON);
            }
            break;
        }
        case POWER_ON: {
            if (gState.i2cRegData1 != 0) {
                transition_state(&gState, DEVICE_IDLE);
            } else if (!gState.connector_detect) {
                transition_state(&gState, CABLE_DETECT);
            } else if (!gState.lock_switch) {
                transition_state(&gState, CABLE_LOCK);
            } else if (!gState.power_enable) {
                transition_state(&gState, POWER_OFF);
            }
            break;
        }
        default: {
            printf("Unknown state = %d\n", gState.fsm);
            break;
        }
    }
    if (gState.fsm == prev) {
        __WFI();
    }
}

/* I2C Handling */

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	if(TransferDirection == I2C_DIRECTION_TRANSMIT)  // if the master wants to transmit the data
	{
        //printf("Got I2C transmit!\n");
        gState.i2c = I2C_STATE_IDLE;
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, gState.i2cBuffer, 1, I2C_FIRST_AND_LAST_FRAME);
	}
	else
	{
        if (gState.i2c == I2C_STATE_SEND_DATA) {
            gState.i2c = I2C_STATE_WAIT_ACK;
            HAL_I2C_Slave_Seq_Transmit_IT(hi2c, gState.i2cBuffer, gState.i2cSize, I2C_FIRST_AND_LAST_FRAME);
        } else {
            gState.i2c = I2C_STATE_IDLE;
            printf("I2C: Unexpected read!\n");
            HAL_I2C_Slave_Seq_Transmit_IT(hi2c, NULL, 0, I2C_FIRST_AND_LAST_FRAME);
        }
	}
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (gState.i2c == I2C_STATE_IDLE) {
        gState.i2cCmd = gState.i2cBuffer[0];
        if (gState.i2cCmd == 0xA0) {
            gState.i2c = I2C_STATE_ACCESS_REG;
            HAL_I2C_Slave_Seq_Receive_IT(hi2c, gState.i2cBuffer, 1, I2C_NEXT_FRAME);
        } else if (gState.i2cCmd == 0xA1 || gState.i2cCmd == 0xA2 || gState.i2cCmd == 0xA3) {
            gState.i2cReg = 0;
            if (gState.i2cCmd == 0xA1) {
                clear_ec_irq();
                gState.i2cBuffer[0] = 0xDC;
                gState.i2cSize = 1;
            } else if (gState.i2cCmd == 0xA2) {
                gState.i2cBuffer[0] = gState.i2cRegData1 == 0 ? 2 : 1;
                gState.i2cSize = 1;
            } else if (gState.i2cCmd == 0xA3) {
                gState.i2cBuffer[0] = 1;
                gState.i2cSize = 1;
            }
            gState.i2c = I2C_STATE_SEND_DATA;
        } else if (gState.i2cCmd == 0xE6) {
            gState.i2c = I2C_STATE_IDLE;
            // ignore this, we might get spammed it
        } else {
            gState.i2c = I2C_STATE_IDLE;
            printf("I2C: Unsupported CMD = 0x%02X\n", gState.i2cCmd);
        }
    } else if (gState.i2c == I2C_STATE_ACCESS_REG) {
        gState.i2cReg = gState.i2cBuffer[0];
        gState.i2c = I2C_STATE_ACCESS_SIZE;
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, gState.i2cBuffer, 1, I2C_NEXT_FRAME);
    } else if (gState.i2c == I2C_STATE_ACCESS_SIZE) {
        gState.i2cSize = gState.i2cBuffer[0];
        gState.i2c = I2C_STATE_WAIT_DATA;
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, gState.i2cBuffer, gState.i2cSize, I2C_LAST_FRAME);
    } else if (gState.i2c == I2C_STATE_WAIT_DATA) {
        printf("I2C: CMD = 0x%02X, reg = 0x%02X, size = %d\n", gState.i2cCmd, gState.i2cReg, gState.i2cSize);
        printf("I2C: Data = ");
        for (int i = 0; i < gState.i2cSize; i++) {
            printf("0x%02X ", gState.i2cBuffer[i]);
        }
        printf("\n");
        if (gState.i2cCmd == 0xA0 && gState.i2cSize == 1 && gState.i2cReg == 1) {
            gState.i2cRegData1 = gState.i2cBuffer[0];
        }
        gState.i2c = I2C_STATE_IDLE;
    }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (gState.i2c == I2C_STATE_WAIT_ACK) {
        if (gState.i2cCmd != 0xA3) { // too verbose
            printf("I2C: CMD = 0x%02X, reg = 0x%02X, size = %d\n", gState.i2cCmd, gState.i2cReg, gState.i2cSize);
            printf("I2C: Sent data = ");
            for (int i = 0; i < gState.i2cSize; i++) {
                printf("0x%02X ", gState.i2cBuffer[i]);
            }
            printf("\n");
        }
        gState.i2c = I2C_STATE_IDLE;
    } else {
        printf("I2C: Unexpected TX complete callback!\n");
        gState.i2c = I2C_STATE_IDLE;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    printf("I2C: Bus error seen!\n");
    gState.i2c = I2C_STATE_IDLE;
}
