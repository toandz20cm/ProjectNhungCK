#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

// --- Pin Definitions ---
// LEDs
#define LED_RED_PORT        GPIOB
#define LED_RED_PIN         0
#define LED_YELLOW_PORT     GPIOB
#define LED_YELLOW_PIN      1
#define LED_BLUE_PORT       GPIOB
#define LED_BLUE_PIN        2
#define LED_GREEN_PORT      GPIOB
#define LED_GREEN_PIN       3

// Buttons
#define BUTTON_START_STOP_PORT  GPIOC
#define BUTTON_START_STOP_PIN   13 // PC13 - User Button on Nucleo
#define BUTTON_RESET_PORT       GPIOA
#define BUTTON_RESET_PIN        1

// Buzzer
#define BUZZER_PORT         GPIOB
#define BUZZER_PIN          10

// MQ-2 ADC
#define MQ2_ADC_PORT        GPIOA
#define MQ2_ADC_PIN         0
#define MQ2_ADC_CHANNEL     0

// LCD I2C
#define LCD_I2C_PORT        GPIOB
#define LCD_SCL_PIN         6
#define LCD_SDA_PIN         7
#define LCD_I2C             I2C1
#define LCD_ADDRESS         (0x27 << 1) // Common PCF8574 address (shifted for 7-bit)

// Relay (Optional)
#define RELAY_PORT          GPIOA
#define RELAY_PIN           5

// --- MQ-2 Sensor Parameters ---
#define ADC_MAX_VALUE       4095.0f // 12-bit ADC
#define VREF_VOLTAGE        3.3f    // ADC Reference Voltage
#define MQ2_RL_VALUE        5000.0f // Load resistor in Ohms (typ. 5k for modules)
#define MQ2_RO_VALUE        15000.0f // Calibrated Ro in clean air (NEEDS CALIBRATION!)
                                    // This value is a placeholder and MUST be calibrated
                                    // for your specific sensor and environment.
// LPG Curve Parameters (example, check your MQ-2 datasheet)
// For PPM = A * (Rs/Ro)^B
#define MQ2_LPG_A           570.30f // Example value for LPG
#define MQ2_LPG_B           -2.23f  // Example value for LPG

// --- Gas Thresholds (PPM) ---
#define GAS_THRESHOLD_LOW       300
#define GAS_THRESHOLD_NORMAL    500 // Above this is considered "high"
#define GAS_THRESHOLD_DANGER    1000

// --- System States & Control ---
typedef enum {
    STATE_IDLE,
    STATE_NORMAL,
    STATE_ALARM_LOW,
    STATE_ALARM_HIGH,
    STATE_ALARM_DANGER
} SystemState_t;

volatile SystemState_t current_system_state = STATE_IDLE;
volatile uint8_t system_running = 0; // 0 = stopped, 1 = running
volatile uint32_t systick_ms_count = 0;
volatile uint8_t update_lcd_flag = 0;
volatile uint8_t read_sensor_flag = 0;

// Button press flags
volatile uint8_t start_stop_pressed_flag = 0;
volatile uint8_t reset_state_pressed_flag = 0;
uint32_t last_start_stop_press_time = 0;
uint32_t last_reset_state_press_time = 0;
#define DEBOUNCE_TIME_MS 50

// LED blink control
volatile uint8_t led_blink_active = 0;
volatile uint32_t led_blink_interval_ms = 500; // For 1Hz initially

// --- UART Debug ---
char uart_buffer[100];
void UART2_Config(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // Enable USART2 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // Enable GPIOA clock

    // PA2 (TX), PA3 (RX) for USART2
    GPIOA->MODER &= ~((3U << (2*2)) | (3U << (3*2))); // Clear mode for PA2, PA3
    GPIOA->MODER |=  ((2U << (2*2)) | (2U << (3*2))); // AF mode for PA2, PA3
    GPIOA->AFR[0] &= ~((0xFU << (2*4)) | (0xFU << (3*4))); // Clear AF for PA2, PA3
    GPIOA->AFR[0] |=  ((7U << (2*4)) | (7U << (3*4))); // AF7 (USART2) for PA2, PA3

    // USART2 Configuration (assuming 16MHz APB1 clock, for 84MHz SysClock, APB1 prescaler is /4 -> 21MHz, or /2 -> 42MHz)
    // For 84MHz SysClock, HCLK=84MHz. If APB1 prescaler is /2, PCLK1=42MHz.
    // Baud rate = 9600. USARTDIV = 42MHz / (16 * 9600) = 273.4375
    // BRR = (Mantissa << 4) | Fraction
    // Mantissa = 273, Fraction = 0.4375 * 16 = 7
    // BRR = (273 << 4) | 7 = 0x1117
    USART2->BRR = 0x1117; // For 42MHz PCLK1
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE; // Enable TX, RX
    USART2->CR1 |= USART_CR1_UE; // Enable USART
}

void UART2_SendChar(char c) {
    while (!(USART2->SR & USART_SR_TXE)); // Wait for TXE (Transmit Data Register Empty)
    USART2->DR = c;
    while (!(USART2->SR & USART_SR_TC));  // Wait for TC (Transmission Complete)
}

void UART2_SendString(const char *str) {
    while (*str) {
        UART2_SendChar(*str++);
    }
}

// --- SysTick ---
void SysTick_Config_Custom(uint32_t ticks) {
    SysTick->CTRL = 0;                     // Disable SysTick
    SysTick->LOAD = ticks - 1;             // Set reload register
    NVIC_SetPriority(SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL); // Set Priority for Systick Interrupt
    SysTick->VAL = 0;                      // Reset the SysTick counter value
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;   // Enable SysTick IRQ and SysTick Timer
}

void delay_ms(uint32_t ms) {
    uint32_t start_time = systick_ms_count;
    while ((systick_ms_count - start_time) < ms);
}

// --- GPIO ---
void GPIO_Config(void) {
    // Enable GPIO Clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    // LEDs (PB0-Red, PB1-Yellow, PB2-Blue, PB3-Green) - Output
    LED_RED_PORT->MODER &= ~(3U << (LED_RED_PIN * 2));
    LED_RED_PORT->MODER |=  (1U << (LED_RED_PIN * 2));
    LED_YELLOW_PORT->MODER &= ~(3U << (LED_YELLOW_PIN * 2));
    LED_YELLOW_PORT->MODER |=  (1U << (LED_YELLOW_PIN * 2));
    LED_BLUE_PORT->MODER &= ~(3U << (LED_BLUE_PIN * 2));
    LED_BLUE_PORT->MODER |=  (1U << (LED_BLUE_PIN * 2));
    LED_GREEN_PORT->MODER &= ~(3U << (LED_GREEN_PIN * 2));
    LED_GREEN_PORT->MODER |=  (1U << (LED_GREEN_PIN * 2));

    // Buzzer (PB10) - Output
    BUZZER_PORT->MODER &= ~(3U << (BUZZER_PIN * 2));
    BUZZER_PORT->MODER |=  (1U << (BUZZER_PIN * 2));
    BUZZER_PORT->ODR &= ~(1U << BUZZER_PIN); // Buzzer OFF

    // Relay (PA5) - Output (Optional)
    RELAY_PORT->MODER &= ~(3U << (RELAY_PIN * 2));
    RELAY_PORT->MODER |=  (1U << (RELAY_PIN * 2));
    RELAY_PORT->ODR &= ~(1U << RELAY_PIN); // Relay OFF

    // MQ-2 ADC Pin (PA0) - Analog
    MQ2_ADC_PORT->MODER |= (3U << (MQ2_ADC_PIN * 2));

    // Buttons
    // PC13 (Start/Stop) - Input (already has external pull-up on Nucleo)
    BUTTON_START_STOP_PORT->MODER &= ~(3U << (BUTTON_START_STOP_PIN * 2)); // Input mode
    // PA1 (Reset State) - Input with internal pull-up
    BUTTON_RESET_PORT->MODER &= ~(3U << (BUTTON_RESET_PIN * 2)); // Input mode
    BUTTON_RESET_PORT->PUPDR &= ~(3U << (BUTTON_RESET_PIN * 2));
    BUTTON_RESET_PORT->PUPDR |=  (1U << (BUTTON_RESET_PIN * 2)); // Pull-up
}

// --- EXTI ---
void EXTI_Config(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enable SYSCFG clock

    // PC13 (Start/Stop Button)
    SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13; // Clear EXTI13
    SYSCFG->EXTICR[3] |=  SYSCFG_EXTICR4_EXTI13_PC; // Map PC13 to EXTI13
    EXTI->IMR |= (1U << BUTTON_START_STOP_PIN);    // Unmask EXTI13
    EXTI->RTSR &= ~(1U << BUTTON_START_STOP_PIN);   // Disable rising edge
    EXTI->FTSR |= (1U << BUTTON_START_STOP_PIN);    // Enable falling edge for PC13

    NVIC_SetPriority(EXTI15_10_IRQn, 1);
    NVIC_EnableIRQ(EXTI15_10_IRQn);

    // PA1 (Reset State Button)
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI1;   // Clear EXTI1
    SYSCFG->EXTICR[0] |=  SYSCFG_EXTICR1_EXTI1_PA; // Map PA1 to EXTI1
    EXTI->IMR |= (1U << BUTTON_RESET_PIN);        // Unmask EXTI1
    EXTI->RTSR &= ~(1U << BUTTON_RESET_PIN);       // Disable rising edge
    EXTI->FTSR |= (1U << BUTTON_RESET_PIN);        // Enable falling edge for PA1

    NVIC_SetPriority(EXTI1_IRQn, 1);
    NVIC_EnableIRQ(EXTI1_IRQn);
}

// --- ADC ---
void ADC_Config(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // Enable ADC1 clock
    ADC1->CR1 = 0; // Disable scan mode, interrupts
    ADC1->CR2 = 0; // Single conversion, right alignment
    ADC1->CR2 |= ADC_CR2_ADON; // Turn on ADC
    ADC1->SQR1 = 0; // 1 conversion
    ADC1->SQR3 = MQ2_ADC_CHANNEL; // Select channel 0 (PA0)
    // Sampling time for channel 0 (e.g., 15 cycles)
    ADC1->SMPR2 &= ~(7U << (MQ2_ADC_CHANNEL * 3));
    ADC1->SMPR2 |=  (2U << (MQ2_ADC_CHANNEL * 3)); // 15 cycles
    delay_ms(1); // ADC stabilization time
}

uint16_t ADC_Read(void) {
    ADC1->CR2 |= ADC_CR2_SWSTART; // Start conversion
    while (!(ADC1->SR & ADC_SR_EOC)); // Wait for End Of Conversion
    return ADC1->DR;
}

// --- MQ-2 Functions ---
float ADC_to_Voltage(uint16_t adc_value) {
    return (adc_value / ADC_MAX_VALUE) * VREF_VOLTAGE;
}

int Estimate_Gas_PPM(float voltage_sensor) {
    if (voltage_sensor <= 0.0f) return 0; // Avoid division by zero or log of non-positive
    // V_RL is the voltage across the load resistor RL
    // V_c is the circuit voltage (typically 5V for MQ-2 modules)
    // Rs = (Vc - V_RL) / V_RL * RL
    // Assuming the module outputs V_RL directly or a scaled version.
    // If the PA0 is connected directly to the MQ-2 AO pin, and the module is 5V powered,
    // and the STM32 ADC max is 3.3V, a voltage divider is needed or ensure output < 3.3V.
    // Here, 'voltage_sensor' is assumed to be the actual V_RL.
    float Rs_gas = (5.0f - voltage_sensor) / voltage_sensor * MQ2_RL_VALUE;
    if (Rs_gas < 0) Rs_gas = 0; // Should not happen if V_sensor < 5V

    float ratio = Rs_gas / MQ2_RO_VALUE;
    if (ratio <= 0) return 0;

    // Using LPG curve as an example
    float ppm = MQ2_LPG_A * powf(ratio, MQ2_LPG_B);
    return (int)ppm;
}

// --- I2C and LCD ---
void I2C_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; // Enable I2C1 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // PB6 SCL, PB7 SDA

    // Configure PB6 and PB7 as Alternate Function, Open-Drain, Pull-up
    LCD_I2C_PORT->MODER &= ~((3U << (LCD_SCL_PIN * 2)) | (3U << (LCD_SDA_PIN * 2)));
    LCD_I2C_PORT->MODER |=  ((2U << (LCD_SCL_PIN * 2)) | (2U << (LCD_SDA_PIN * 2))); // AF mode
    LCD_I2C_PORT->OTYPER |= (1U << LCD_SCL_PIN) | (1U << LCD_SDA_PIN);      // Open-drain
    LCD_I2C_PORT->PUPDR &= ~((3U << (LCD_SCL_PIN * 2)) | (3U << (LCD_SDA_PIN * 2)));
    LCD_I2C_PORT->PUPDR |=  ((1U << (LCD_SCL_PIN * 2)) | (1U << (LCD_SDA_PIN * 2))); // Pull-up
    LCD_I2C_PORT->OSPEEDR |= ((3U << (LCD_SCL_PIN*2)) | (3U << (LCD_SDA_PIN*2))); // High speed

    LCD_I2C_PORT->AFR[0] &= ~((0xFU << (LCD_SCL_PIN * 4)) | (0xFU << (LCD_SDA_PIN * 4)));
    LCD_I2C_PORT->AFR[0] |=  ((4U << (LCD_SCL_PIN * 4)) | (4U << (LCD_SDA_PIN * 4))); // AF4 for I2C1

    // Reset I2C
    LCD_I2C->CR1 |= I2C_CR1_SWRST;
    LCD_I2C->CR1 &= ~I2C_CR1_SWRST;

    // Configure I2C (assuming PCLK1 = 42MHz for 100kHz I2C)
    // TPCLK1 = 1/42MHz = 23.8ns
    // For 100kHz: Thigh = Tlow = 5000ns. CCR = 5000ns / 23.8ns = 210
    LCD_I2C->CR2 = 42; // PCLK1 frequency in MHz
    LCD_I2C->CCR = 210; // Standard mode, 100kHz
    LCD_I2C->TRISE = 43; // Max rise time for 100kHz
    LCD_I2C->CR1 |= I2C_CR1_PE; // Enable I2C
}

void I2C_Start(void) {
    LCD_I2C->CR1 |= I2C_CR1_START;
    while (!(LCD_I2C->SR1 & I2C_SR1_SB)); // Wait for start bit generated
}

void I2C_Stop(void) {
    LCD_I2C->CR1 |= I2C_CR1_STOP;
    while (LCD_I2C->SR1 & I2C_SR2_MSL); // Wait until BUSY is cleared
}

void I2C_Write(uint8_t data) {
    while (!(LCD_I2C->SR1 & I2C_SR1_TXE)); // Wait for TXE
    LCD_I2C->DR = data;
    while (!(LCD_I2C->SR1 & I2C_SR1_BTF)); // Wait for byte transfer finished
}

void I2C_Address(uint8_t address) {
    LCD_I2C->DR = address;
    while (!(LCD_I2C->SR1 & I2C_SR1_ADDR)); // Wait for ADDR
    // Clear ADDR flag by reading SR1 then SR2
    (void)LCD_I2C->SR1;
    (void)LCD_I2C->SR2;
}

void LCD_SendCommand(uint8_t cmd) {
    I2C_Start();
    I2C_Address(LCD_ADDRESS);
    I2C_Write(0x00); // Co = 0, RS = 0 for command
    I2C_Write(cmd);
    I2C_Stop();
    delay_ms(2); // Commands need some time
}

void LCD_SendData(uint8_t data) {
    I2C_Start();
    I2C_Address(LCD_ADDRESS);
    I2C_Write(0x40); // Co = 0, RS = 1 for data
    I2C_Write(data);
    I2C_Stop();
    delay_ms(1);
}

// For PCF8574 based I2C LCD adapter, commands are sent nibble by nibble
void LCD_Send(uint8_t value, uint8_t mode) {
    uint8_t high_nibble = value & 0xF0;
    uint8_t low_nibble = (value << 4) & 0xF0;
    uint8_t data_arr[4];

    // For PCF8574, P0-P3 are data D4-D7, P2 is EN, P1 is RW, P0 is RS
    // Backlight is P3 on the PCF8574T module (P7 on PCF8574AT)
    // Assuming P3 is backlight control, always on.
    #define LCD_BACKLIGHT 0x08 // P3 for PCF8574T
    #define LCD_EN 0x04 // P2
    #define LCD_RW 0x02 // P1 (grounded for write)
    #define LCD_RS 0x01 // P0

    uint8_t rs_val = (mode == 1) ? LCD_RS : 0; // 1 for data, 0 for command

    data_arr[0] = high_nibble | rs_val | LCD_BACKLIGHT | LCD_EN;
    data_arr[1] = high_nibble | rs_val | LCD_BACKLIGHT; // EN low
    data_arr[2] = low_nibble  | rs_val | LCD_BACKLIGHT | LCD_EN;
    data_arr[3] = low_nibble  | rs_val | LCD_BACKLIGHT; // EN low

    I2C_Start();
    I2C_Address(LCD_ADDRESS);
    for(int i=0; i<4; i++) {
        I2C_Write(data_arr[i]);
        if (i==0 || i==2) delay_ms(1); // Short pulse for EN
    }
    I2C_Stop();
    if (mode == 0 && (value == 0x01 || value == 0x02)) delay_ms(2); // Clear/Home needs more time
    else delay_ms(1);
}


void LCD_Init(void) {
    delay_ms(50); // Wait for LCD power up
    // 4-bit mode initialization sequence
    LCD_Send(0x30, 0); delay_ms(5);
    LCD_Send(0x30, 0); delay_ms(1);
    LCD_Send(0x30, 0); delay_ms(1);
    LCD_Send(0x20, 0); // Function set: 4-bit mode

    LCD_Send(0x28, 0); // Function set: 4-bit, 2 lines, 5x8 font
    LCD_Send(0x0C, 0); // Display ON, Cursor OFF, Blink OFF
    LCD_Send(0x06, 0); // Entry mode set: Increment cursor, no shift
    LCD_Send(0x01, 0); // Clear display
    delay_ms(2);
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t address;
    switch (row) {
        case 0: address = 0x80 + col; break; // Line 1
        case 1: address = 0xC0 + col; break; // Line 2
        // Add for 20x4 if needed
        // case 2: address = 0x94 + col; break; // Line 3 for 20x4
        // case 3: address = 0xD4 + col; break; // Line 4 for 20x4
        default: address = 0x80 + col; break;
    }
    LCD_Send(address, 0);
}

void LCD_Print(const char* str) {
    while (*str) {
        LCD_Send(*str++, 1);
    }
}

void LCD_Clear(void) {
    LCD_Send(0x01, 0); // Clear display command
    delay_ms(2);       // This command takes longer
}


// --- TIM2 for LED Blinking ---
void TIM2_Config(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2 clock

    // PCLK1 is 42MHz. We want a base tick around 1ms or less for flexibility.
    // E.g., for 100Hz tick (10ms period)
    // TIM2_CLK = 42MHz. Prescaler = 4200-1 => Timer clock = 10kHz
    // Period = 100 => 100 * 0.1ms = 10ms interrupt (100Hz)
    // For 1Hz blink (500ms toggle), we need 50 interrupts.
    // Let's make the timer interrupt more frequent, e.g., every 10ms.
    TIM2->PSC = 4200 - 1;  // 42MHz / 4200 = 10kHz timer clock (0.1ms period)
    TIM2->ARR = 100 - 1;   // 100 * 0.1ms = 10ms interrupt period (100Hz)

    TIM2->DIER |= TIM_DIER_UIE; // Enable update interrupt
    TIM2->CR1 |= TIM_CR1_CEN;   // Enable timer

    NVIC_SetPriority(TIM2_IRQn, 2);
    NVIC_EnableIRQ(TIM2_IRQn);
}

// --- System Clock Config ---
// Configure system clock to 84MHz using HSI (16MHz) and PLL
void SystemClock_Config(void) {
    // Enable HSI
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    // Configure PLL: HSI (16MHz) as source
    // PLLM = 8 (16MHz / 8 = 2MHz VCO input)
    // PLLN = 84 (2MHz * 84 = 168MHz VCO output)
    // PLLP = /2 (168MHz / 2 = 84MHz SYSCLK)
    // PLLQ = /4 (for USB, not strictly needed here if USB not used, but good practice)
    RCC->PLLCFGR = (4 << RCC_PLLCFGR_PLLQ_Pos) |  // PLLQ = 4
                   (0 << RCC_PLLCFGR_PLLSRC_Pos) | // HSI as source
                   ((84/2 -1) << RCC_PLLCFGR_PLLP_Pos) | // PLLP = 0 (/2)
                   (84 << RCC_PLLCFGR_PLLN_Pos) |
                   (8 << RCC_PLLCFGR_PLLM_Pos);

    RCC->CR |= RCC_CR_PLLON; // Enable PLL
    while (!(RCC->CR & RCC_CR_PLLRDY)); // Wait for PLL ready

    // Configure Flash latency, prefetch, caches
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_2WS; // 2 Wait States for 84MHz

    // Select PLL as system clock source
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    // Configure AHB, APB1, APB2 prescalers
    RCC->CFGR &= ~RCC_CFGR_HPRE;  // AHB prescaler = 1 (HCLK = SYSCLK = 84MHz)
    RCC->CFGR &= ~RCC_CFGR_PPRE1;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; // APB1 prescaler = 2 (PCLK1 = HCLK/2 = 42MHz)
    RCC->CFGR &= ~RCC_CFGR_PPRE2; // APB2 prescaler = 1 (PCLK2 = HCLK/1 = 84MHz)

    SystemCoreClockUpdate(); // Update SystemCoreClock variable
}

// --- Control Functions ---
void set_leds_for_state(SystemState_t state) {
    // Turn all LEDs off first
    LED_RED_PORT->ODR &= ~(1U << LED_RED_PIN);
    LED_YELLOW_PORT->ODR &= ~(1U << LED_YELLOW_PIN);
    LED_BLUE_PORT->ODR &= ~(1U << LED_BLUE_PIN);
    LED_GREEN_PORT->ODR &= ~(1U << LED_GREEN_PIN);
    led_blink_active = 0; // Stop blinking by default

    if (!system_running) {
        LED_GREEN_PORT->ODR |= (1U << LED_GREEN_PIN); // Green for idle/stopped
        return;
    }

    switch (state) {
        case STATE_NORMAL:
            LED_BLUE_PORT->ODR |= (1U << LED_BLUE_PIN);
            break;
        case STATE_ALARM_LOW:
            LED_YELLOW_PORT->ODR |= (1U << LED_YELLOW_PIN);
            break;
        case STATE_ALARM_HIGH: // Red 1Hz blink
            led_blink_active = 1;
            led_blink_interval_ms = 500; // 500ms on, 500ms off -> 1Hz
            // TIM2 will handle the blinking of RED LED
            break;
        case STATE_ALARM_DANGER: // Red faster blink
            led_blink_active = 1;
            led_blink_interval_ms = 100; // 100ms on, 100ms off -> 5Hz (adjust for 2-10Hz)
            // TIM2 will handle the blinking of RED LED
            break;
        default: // Also STATE_IDLE if system_running was true (should not happen)
            LED_GREEN_PORT->ODR |= (1U << LED_GREEN_PIN);
            break;
    }
}

void update_system_display(int ppm) {
    char line1_buf[20];
    char line2_buf[20];

    LCD_SetCursor(0, 0);
    if (!system_running) {
        sprintf(line1_buf, "System Stopped  ");
    } else {
        sprintf(line1_buf, "PPM: %-4d       ", ppm);
    }
    LCD_Print(line1_buf);

    LCD_SetCursor(1, 0);
    switch (current_system_state) {
        case STATE_IDLE:    sprintf(line2_buf, "Status: IDLE    "); break;
        case STATE_NORMAL:  sprintf(line2_buf, "Status: NORMAL  "); break;
        case STATE_ALARM_LOW: sprintf(line2_buf, "ALARM: LOW GAS  "); break;
        case STATE_ALARM_HIGH:sprintf(line2_buf, "ALARM: HIGH GAS!"); break;
        case STATE_ALARM_DANGER:sprintf(line2_buf,"DANGER! EVACUATE"); break;
        default:            sprintf(line2_buf, "Status: UNKNOWN "); break;
    }
    LCD_Print(line2_buf);
}


// --- Main ---
int main(void) {
    SystemClock_Config();
    SysTick_Config_Custom(SystemCoreClock / 1000); // 1ms tick

    GPIO_Config();
    ADC_Config();
    EXTI_Config();
    TIM2_Config(); // For LED blinking
    I2C_Init();
    LCD_Init();
    UART2_Config(); // For debug

    UART2_SendString("System Initialized.\r\n");
    sprintf(uart_buffer, "MQ2_RO_VALUE (CALIBRATE THIS!): %.1f Ohms\r\n", MQ2_RO_VALUE);
    UART2_SendString(uart_buffer);


    int current_ppm = 0;
    set_leds_for_state(current_system_state); // Initial LED state
    LCD_Clear();
    update_system_display(0);


    while (1) {
        // --- Button Processing ---
        if (start_stop_pressed_flag) {
            if ((systick_ms_count - last_start_stop_press_time) > DEBOUNCE_TIME_MS) {
                system_running = !system_running;
                if (system_running) {
                    current_system_state = STATE_NORMAL; // Default to normal when starting
                    UART2_SendString("System Started.\r\n");
                } else {
                    current_system_state = STATE_IDLE;
                    BUZZER_PORT->ODR &= ~(1U << BUZZER_PIN); // Turn off buzzer when stopped
                    RELAY_PORT->ODR &= ~(1U << RELAY_PIN);   // Turn off relay
                    UART2_SendString("System Stopped.\r\n");
                }
                set_leds_for_state(current_system_state);
                update_lcd_flag = 1; // Force LCD update
                start_stop_pressed_flag = 0; // Clear flag
            }
        }

        if (reset_state_pressed_flag) {
            if ((systick_ms_count - last_reset_state_press_time) > DEBOUNCE_TIME_MS) {
                if (system_running) { // Only reset state if running
                    current_system_state = STATE_NORMAL;
                    BUZZER_PORT->ODR &= ~(1U << BUZZER_PIN); // Turn off buzzer
                    RELAY_PORT->ODR &= ~(1U << RELAY_PIN);   // Turn off relay
                    set_leds_for_state(current_system_state);
                    update_lcd_flag = 1;
                    UART2_SendString("Alarm State Reset.\r\n");
                }
                reset_state_pressed_flag = 0; // Clear flag
            }
        }

        // --- Sensor Reading and State Update (if running) ---
        if (system_running && read_sensor_flag) {
            read_sensor_flag = 0;
            uint16_t adc_val = ADC_Read();
            float voltage = ADC_to_Voltage(adc_val);
            current_ppm = Estimate_Gas_PPM(voltage);

            sprintf(uart_buffer, "ADC: %d, Voltage: %.2fV, PPM: %d\r\n", adc_val, (double)voltage, current_ppm);
            UART2_SendString(uart_buffer);

            SystemState_t previous_state = current_system_state;

            if (current_ppm > GAS_THRESHOLD_DANGER) {
                current_system_state = STATE_ALARM_DANGER;
            } else if (current_ppm > GAS_THRESHOLD_NORMAL) {
                current_system_state = STATE_ALARM_HIGH;
            } else if (current_ppm > GAS_THRESHOLD_LOW) {
                current_system_state = STATE_ALARM_LOW;
            } else {
                current_system_state = STATE_NORMAL;
            }

            if (current_system_state != previous_state) {
                set_leds_for_state(current_system_state);
                update_lcd_flag = 1;
            }

            // Buzzer and Relay Control
            if (current_system_state == STATE_ALARM_HIGH || current_system_state == STATE_ALARM_DANGER) {
                BUZZER_PORT->ODR |= (1U << BUZZER_PIN); // Buzzer ON
                RELAY_PORT->ODR |= (1U << RELAY_PIN);   // Relay ON (Optional: to shut off gas)
            } else {
                // Buzzer turns off if not high/danger OR if reset button was pressed
                // Relay also turns off if not high/danger
                 if (previous_state == STATE_ALARM_HIGH || previous_state == STATE_ALARM_DANGER) {
                    BUZZER_PORT->ODR &= ~(1U << BUZZER_PIN);
                    RELAY_PORT->ODR &= ~(1U << RELAY_PIN);
                 }
            }
        }

        // --- LCD Update ---
        if (update_lcd_flag) {
            update_lcd_flag = 0;
            update_system_display(current_ppm);
        }
    }
}

// --- Interrupt Handlers ---
void SysTick_Handler(void) {
    systick_ms_count++;
    static uint32_t lcd_update_counter = 0;
    static uint32_t sensor_read_counter = 0;

    lcd_update_counter++;
    sensor_read_counter++;

    if (lcd_update_counter >= 500) { // Update LCD every 500ms
        lcd_update_counter = 0;
        update_lcd_flag = 1;
    }
    if (sensor_read_counter >= 1000) { // Read sensor every 1s
        sensor_read_counter = 0;
        if (system_running) {
            read_sensor_flag = 1;
        }
    }
}

void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) { // Check for update interrupt flag
        TIM2->SR &= ~TIM_SR_UIF;   // Clear flag

        static uint32_t blink_counter = 0;
        if (led_blink_active && system_running) {
            blink_counter++;
            // led_blink_interval_ms is the half-period (time for LED on or LED off)
            // TIM2 fires every 10ms. So, (led_blink_interval_ms / 10) is number of TIM2 ticks.
            if (blink_counter >= (led_blink_interval_ms / 10) ) {
                blink_counter = 0;
                LED_RED_PORT->ODR ^= (1U << LED_RED_PIN); // Toggle Red LED
            }
        } else if (!led_blink_active && (current_system_state != STATE_ALARM_HIGH && current_system_state != STATE_ALARM_DANGER)) {
             LED_RED_PORT->ODR &= ~(1U << LED_RED_PIN); // Ensure red LED is off if not blinking
        }
    }
}

void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR & (1U << BUTTON_START_STOP_PIN)) { // Check for EXTI13 (PC13)
        if ((systick_ms_count - last_start_stop_press_time) > DEBOUNCE_TIME_MS) { // Debounce
            start_stop_pressed_flag = 1;
            last_start_stop_press_time = systick_ms_count;
        }
        EXTI->PR = (1U << BUTTON_START_STOP_PIN); // Clear pending bit
    }
}

void EXTI1_IRQHandler(void) {
    if (EXTI->PR & (1U << BUTTON_RESET_PIN)) { // Check for EXTI1 (PA1)
         if ((systick_ms_count - last_reset_state_press_time) > DEBOUNCE_TIME_MS) { // Debounce
            reset_state_pressed_flag = 1;
            last_reset_state_press_time = systick_ms_count;
        }
        EXTI->PR = (1U << BUTTON_RESET_PIN); // Clear pending bit
    }
}