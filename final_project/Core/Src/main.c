#include <stdio.h>
#include "stm32f4xx.h"


#define TRIG_PIN    9      // PA9
#define ECHO_PIN    10     // PA10

#define LED_Green_PIN     6
#define LED_RED_PIN     7

#define BUTTON_PIN 13  // Nút nhấn nối với PC13
#define LED_PIN 5      // LED nối với PA6

#define IN1_PIN    0  // GPIOA, chân PA0
#define IN2_PIN    1  // GPIOA, chân PA1
#define IN3_PIN    4  // GPIOA, chân PA4
#define IN4_PIN    0  // GPIOB, chân PB0

volatile uint32_t last_button_press = 0;
volatile uint32_t tick_count = 0;  // Bộ đếm millisecond

#define SLAVE_ADDRESS_LCD 0x27

volatile uint32_t tick = 0;
volatile uint8_t system_state = 1;  // 1: Đang đo, 0: Dừng đo

void UART_Init(void);
void UART_SendChar(char c);
void UART_SendString(const char *str);
void GPIO_Trig_Echo_Init(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
void toggleLED(uint32_t ledPin, uint32_t frequency);

void Motor_Init(void);
void TIM2_PWM_Init(void);
void Motor_Forward(uint16_t speedA, uint16_t speedB);
void Motor_Backward(uint16_t speedA, uint16_t speedB);

void Motor_Stop(void);
void Motor_TurnRight(uint16_t speed);
void Motor_TurnLeft(uint16_t speed);

const float speedOfSound = 0.0343 / 2; // cm/µs


// Khởi tạo I2C1
void I2C1_Init(void) {
    RCC->APB1ENR |= (1 << 21);   // Bật clock I2C1
    RCC->AHB1ENR |= (1 << 1);    // Bật clock GPIOB

    // Cấu hình chân PB8, PB9 làm Alternate Function (AF4)
    GPIOB->MODER |= (2 << 16) | (2 << 18);
    GPIOB->OTYPER |= (1 << 8) | (1 << 9);
    GPIOB->OSPEEDR |= (3 << 16) | (3 << 18);
    GPIOB->PUPDR |= (1 << 16) | (1 << 18);
    GPIOB->AFR[1] |= (4 << 0) | (4 << 4);

    // Reset I2C
    I2C1->CR1 |= (1 << 15);
    I2C1->CR1 &= ~(1 << 15);

    // Thiết lập clock I2C
    I2C1->CR2 |= (45 << 0);   // Tần số PCLK1 = 45 MHz
    I2C1->CCR = 225;          // Thiết lập tốc độ truyền
    I2C1->TRISE = 46;         // Thời gian tăng

    // Bật I2C
    I2C1->CR1 |= (1 << 0);
}

// Gửi Start Condition
void I2C_Start(void) {
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
}

// Gửi Stop Condition
void I2C_Stop(void) {
    I2C1->CR1 |= I2C_CR1_STOP;
}

// Gửi địa chỉ thiết bị
void I2C_Address(uint8_t Address) {
    I2C1->DR = Address << 1;  // Địa chỉ + bit Write (0)
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;          // Clear ADDR flag
}

// Gửi dữ liệu
void I2C_Write(uint8_t data) {
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_BTF));
}

// Ghi dữ liệu ra LCD
void LCD_Write(uint8_t Address, uint8_t *Data, int size) {
    I2C_Start();
    I2C_Address(Address);
    for (int i = 0; i < size; i++) {
        I2C_Write(Data[i]);
    }
    I2C_Stop();
}

// Gửi lệnh tới LCD
void lcd_send_cmd(char cmd) {
    char data_u = (cmd & 0xF0);
    char data_l = ((cmd << 4) & 0xF0);
    uint8_t data_t[4] = {
        data_u | 0x0C, // EN=1, RS=0
        data_u | 0x08, // EN=0, RS=0
        data_l | 0x0C, // EN=1, RS=0
        data_l | 0x08  // EN=0, RS=0
    };
    LCD_Write(SLAVE_ADDRESS_LCD, data_t, 4);
}

// Gửi dữ liệu (ký tự) tới LCD
void lcd_send_data(char data) {
    char data_u = (data & 0xF0);
    char data_l = ((data << 4) & 0xF0);
    uint8_t data_t[4] = {
        data_u | 0x0D, // EN=1, RS=1
        data_u | 0x09, // EN=0, RS=1
        data_l | 0x0D, // EN=1, RS=1
        data_l | 0x09  // EN=0, RS=1
    };
    LCD_Write(SLAVE_ADDRESS_LCD, data_t, 4);
}

// Xóa màn hình LCD
void lcd_clear(void) {
    lcd_send_cmd(0x01);  // Lệnh xóa màn hình
}

// Đặt con trỏ tại vị trí (hàng, cột)
void lcd_put_cur(int row, int col) {
    char pos;
    if (row == 0) pos = 0x80 + col;
    else pos = 0xC0 + col;
    lcd_send_cmd(pos);
}

// Khởi tạo LCD
void lcd_init(void) {
    lcd_send_cmd(0x02);  // Return home
    lcd_send_cmd(0x28);  // 4-bit mode, 2 lines
    lcd_send_cmd(0x0C);  // Display ON, cursor OFF
    lcd_send_cmd(0x06);  // Increment cursor
    lcd_send_cmd(0x01);  // Clear display
}

// Hiển thị chuỗi ký tự trên LCD
void lcd_send_string(char *str) {
    while (*str) lcd_send_data(*str++);
}

// For the hcsr04
float measure_distance(void) {
    uint32_t timeout = 0;
    uint32_t numTicks = 0;
    float distance = 0;

    // 1. Đưa TRIG xuống LOW
    GPIOA->ODR &= ~(1 << TRIG_PIN);
    delay_us(2);

    // 2. Đưa TRIG lên HIGH trong 10 µs
    GPIOA->ODR |= (1 << TRIG_PIN);
    delay_us(10);
    GPIOA->ODR &= ~(1 << TRIG_PIN);

    // 3. Chờ ECHO lên HIGH với timeout
    timeout = 100000; // 100 ms timeout
    while (!(GPIOA->IDR & (1 << ECHO_PIN)) && timeout--) {
        delay_us(1);
    }
    if (timeout == 0) return -1; // Timeout error

    // 4. Đo thời gian ECHO ở mức HIGH
    numTicks = 0;
    while (GPIOA->IDR & (1 << ECHO_PIN)) {
        numTicks++;
        delay_us(1);
        if (numTicks > 30000) return -1; // Timeout error
    }

    // 5. Tính khoảng cách
    distance = numTicks * 0.343 / 2; // Sửa công thức: numTicks x tốc độ âm thanh / 2
    return distance;
}

// For the UART
void UART_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // Bật clock GPIOA
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // Bật clock USART2

    // Cấu hình PA2 (TX) và PA3 (RX) cho USART2
    GPIOA->MODER |= (2 << (2 * 2)) | (2 << (3 * 2)); // Chế độ Alternate Function
    GPIOA->AFR[0] |= (7 << (2 * 4)) | (7 << (3 * 4)); // AF7 cho USART2

    // Cấu hình baud rate (9600 với clock APB1 là 16MHz)
    USART2->BRR = 16000000 / 9600;

    // Cấu hình USART
    USART2->CR1 |= USART_CR1_UE;  // Bật USART
    USART2->CR1 |= USART_CR1_TE;  // Bật Transmitter
}

void UART_SendChar(char c) {
    while (!(USART2->SR & USART_SR_TXE)); // Chờ đến khi TXE = 1
    USART2->DR = c;                      // Gửi ký tự
}

void UART_SendString(const char *str) {
    while (*str) {
        UART_SendChar(*str++); // Gửi từng ký tự trong chuỗi
    }
}

// Hàm khởi tạo GPIO cho Trig và Echo
void GPIO_Trig_Echo_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Bật clock GPIOA

    // Cấu hình TRIG (PA9) làm Output
    GPIOA->MODER |= (1 << (TRIG_PIN * 2));       // Chế độ Output
    GPIOA->OTYPER &= ~(1 << TRIG_PIN);          // Push-Pull
    GPIOA->OSPEEDR |= (3 << (TRIG_PIN * 2));    // Tốc độ cao
    GPIOA->PUPDR &= ~(3 << (TRIG_PIN * 2));     // Không dùng Pull-up/Pull-down

    // Cấu hình ECHO (PA10) làm Input
    GPIOA->MODER &= ~(3 << (ECHO_PIN * 2));     // Chế độ Input
    GPIOA->PUPDR &= ~(3 << (ECHO_PIN * 2));     // Không dùng Pull-up/Pull-down
}

//Hàm delay theo micro giây
void delay_us(uint32_t us) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Bật clock cho Timer 3
    TIM3->PSC = 48 - 1;                 // Prescaler để đạt 1 MHz (1 µs)
    TIM3->ARR = us;                     // Giá trị Auto-reload

    TIM3->CNT = 0;                      // Reset Counter
    TIM3->CR1 |= TIM_CR1_CEN;           // Bật Counter

    while (!(TIM3->SR & TIM_SR_UIF));   // Chờ đến khi đếm xong
    TIM3->SR &= ~TIM_SR_UIF;            // Xóa cờ update
    TIM3->CR1 &= ~TIM_CR1_CEN;          // Tắt Counter
}

// Hàm delay theo mili giây
void delay_ms(uint32_t ms) {
    while (ms--) {
        delay_us(1000); // 1 ms = 1000 µs
    }
}



void SysTick_Init(uint32_t sysTickFreq)
{
    // Enable the SysTick timer and interrupt, with a frequency of sysTickFreq Hz
    // Assuming the System Core Clock is already set to the desired frequency (e.g., 72 MHz on STM32F103)

    uint32_t reloadValue = (SystemCoreClock / sysTickFreq) - 1;  // Reload value for desired frequency

    // Set the reload value (24-bit down-counter)
    SysTick->LOAD = reloadValue;

    // Clear the current value register
    SysTick->VAL = 0;

    // Configure the SysTick timer to use the system clock (AHB clock)
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

    // Enable SysTick interrupt and the timer
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

// Trình xử lý ngắt SysTick (tăng tick mỗi 1ms)
void SysTick_Handler(void) {
    tick_count++;  // Tăng bộ đếm millisecond
}

// Hàm trả về số tick hiện tại
uint32_t getTick(void) {
    return tick_count;
}


// Hàm khởi tạo GPIO cho LED và Button
void GPIO_Init(void) {
    // 1. Kích hoạt xung nhịp cho GPIOA và GPIOC
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // Bật clock cho GPIOA
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;  // Bật clock cho GPIOC

    // 2. Cấu hình PA5 (LED) là output mode
    GPIOA->MODER &= ~(3 << (LED_PIN * 2));  // Xóa MODER5
    GPIOA->MODER |= (1 << (LED_PIN * 2));   // Set MODER5 = 01 (Output)
    GPIOA->PUPDR &= ~(1 << 11);
    GPIOA->PUPDR |= (1 << (LED_PIN *2));

    // led đỏ
    GPIOA->MODER &= ~(3 << (LED_RED_PIN * 2));
    GPIOA->MODER |= (1 << (LED_RED_PIN * 2));
    GPIOA->PUPDR &= ~(1 << 15);
    GPIOA->PUPDR |= (1 << (LED_RED_PIN *2));

    // led xanh
    GPIOA->MODER &= ~(3 << (LED_Green_PIN * 2));
    GPIOA->MODER |= (1 << (LED_Green_PIN * 2));
    GPIOA->PUPDR &= ~(1 << 13);
    GPIOA->PUPDR |= (1 << (LED_Green_PIN*2));

    // 3. Cấu hình PC13 (Button) là input mode
    GPIOC->MODER &= ~(3 << (BUTTON_PIN * 2));  // MODER13 = 00 (Input)

    // 4. Bật pull-up cho PC13
    GPIOC->PUPDR &= ~(3 << (BUTTON_PIN * 2));  // Xóa PUPDR13
    GPIOC->PUPDR |= (1 << (BUTTON_PIN * 2));   // PUPDR13 = 01 (Pull-up)
}


// Hàm khởi tạo ngắt EXTI cho nút nhấn
void EXTI_Init(void) {
    // 1. Kích hoạt clock cho SYSCFG
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // 2. Kết nối PC13 với EXTI13 (EXTICR[3])
    SYSCFG->EXTICR[3] &= ~(0xF << 4);  // Xóa cấu hình cho EXTI13
    SYSCFG->EXTICR[3] |= (2 << 4);     // Chọn PC13 (0010)

    // 3. Bật ngắt cho EXTI13
    EXTI->IMR |= (1 << BUTTON_PIN);  // Cho phép ngắt tại line 13
    EXTI->FTSR |= (1 << BUTTON_PIN); // Kích hoạt ngắt cạnh xuống (falling edge)

    // 4. Kích hoạt ngắt EXTI15_10 trong NVIC
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}



void EXTI15_10_IRQHandler(void) {
    uint32_t current_tick = getTick();

    // Chống rung (debounce)
    if ((current_tick - last_button_press) > 50) {  // Debounce 50ms
        system_state = !system_state;  // Đảo trạng thái hệ thống

        // Tắt đèn LED (bật mức thấp)
        GPIOA->ODR &= ~(1 << LED_PIN);

        // Cập nhật thời gian nhấn lần cuối
        last_button_press = current_tick;
    }

    // Xóa cờ pending cho EXTI
    if (EXTI->PR & (1 << BUTTON_PIN)) {
        EXTI->PR |= (1 << BUTTON_PIN);  // Xóa cờ pending
    }
}


void toggleLED(uint32_t ledPin, uint32_t frequency) {
    static uint32_t lastTick = 0;
    static uint8_t ledState = 0;
    uint32_t interval = 1000 / frequency;

    if (getTick() - lastTick >= interval) {
        ledState = !ledState;
        if (ledState) {
            GPIOA->ODR |= ledPin;
        } else {
            GPIOA->ODR &= ~ledPin;
        }
        lastTick = getTick();
    }
}


void TIM2_PWM_Init(void) {
    // Bật clock cho Timer 2
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Cấu hình PA5 (TIM2_CH1)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~(0x3 << (5 * 2)); // Clear mode bits
    GPIOA->MODER |= (0x2 << (5 * 2)); // Alternate Function mode
    GPIOA->AFR[0] |= (0x1 << (5 * 4)); // AF1 cho TIM2_CH1

    // Cấu hình prescaler và ARR để tạo tần số PWM 1 kHz
    TIM2->PSC = 16 - 1;  // Prescaler để timer chạy ở 1 MHz
    TIM2->ARR = 1000;    // Auto-reload cho tần số 1 kHz

    // Đặt CCR1 (Duty cycle ban đầu = 0%)
    TIM2->CCR1 = 0;

    // Cấu hình PWM mode 1 cho TIM2 Channel 1
    TIM2->CCMR1 &= ~(0x7 << TIM_CCMR1_OC1M_Pos);
    TIM2->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos); // PWM mode 1
    TIM2->CCMR1 |= TIM_CCMR1_OC1PE;          // Preload cho CCR1

    // Kích hoạt output cho Channel 1
    TIM2->CCER |= TIM_CCER_CC1E;

    // Bật chế độ tự động tải lại
    TIM2->CR1 |= TIM_CR1_ARPE;

    // Kích hoạt Timer
    TIM2->CR1 |= TIM_CR1_CEN;
}

void TIM4_PWM_Init(void) {
    // Bật clock cho Timer 4
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    // Cấu hình PB7 (TIM4_CH2)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER &= ~(0x3 << (7 * 2)); // Clear mode bits
    GPIOB->MODER |= (0x2 << (7 * 2)); // Alternate Function mode
    GPIOB->AFR[0] |= (0x2 << (7 * 4)); // AF2 cho TIM4_CH2

    // Cấu hình prescaler và ARR để tạo tần số PWM 1 kHz
    TIM4->PSC = 16 - 1;  // Prescaler để timer chạy ở 1 MHz
    TIM4->ARR = 1000;    // Auto-reload cho tần số 1 kHz

    // Đặt CCR2 (Duty cycle ban đầu = 0%)
    TIM4->CCR2 = 0;

    // Cấu hình PWM mode 1 cho TIM4 Channel 2
    TIM4->CCMR1 &= ~(0x7 << TIM_CCMR1_OC2M_Pos);
    TIM4->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos); // PWM mode 1
    TIM4->CCMR1 |= TIM_CCMR1_OC2PE;          // Preload cho CCR2

    // Kích hoạt output cho Channel 2
    TIM4->CCER |= TIM_CCER_CC2E;

    // Bật chế độ tự động tải lại
    TIM4->CR1 |= TIM_CR1_ARPE;

    // Kích hoạt Timer
    TIM4->CR1 |= TIM_CR1_CEN;
}

void Motor_Init(void) {
    // Bật clock cho GPIOA và GPIOB
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

    // Đặt chế độ chân PA0, PA1, PA4 thành Output
    GPIOA->MODER &= ~((0x3 << (IN1_PIN * 2)) | (0x3 << (IN2_PIN * 2)) | (0x3 << (IN3_PIN * 2))); // Clear bits
    GPIOA->MODER |= ((0x1 << (IN1_PIN * 2)) | (0x1 << (IN2_PIN * 2)) | (0x1 << (IN3_PIN * 2)));  // Output mode

    // Đặt chế độ chân PB0 thành Output
    GPIOB->MODER &= ~(0x3 << (IN4_PIN * 2)); // Clear bits
    GPIOB->MODER |= (0x1 << (IN4_PIN * 2)); // Output mode

    // Đặt mức đầu ra ban đầu là 0 (dừng động cơ)
    GPIOA->ODR &= ~((1 << IN1_PIN) | (1 << IN2_PIN) | (1 << IN3_PIN));
    GPIOB->ODR &= ~(1 << IN4_PIN);
}

void Motor_Forward(uint16_t speedA, uint16_t speedB) {
    if (speedA > 1000) speedA = 1000; // Giới hạn tốc độ
    if (speedB > 1000) speedB = 1000;

    // Động cơ 1: IN1 = 1, IN2 = 0
    GPIOA->ODR |= (1 << IN1_PIN);
    GPIOA->ODR &= ~(1 << IN2_PIN);

    // Động cơ 2: IN3 = 1, IN4 = 0
    GPIOA->ODR |= (1 << IN3_PIN);
    GPIOB->ODR &= ~(1 << IN4_PIN);

    // Đặt tốc độ cho PWM
    TIM2->CCR1 = speedA;
    TIM4->CCR2 = speedB;
}

void Motor_Backward(uint16_t speedA, uint16_t speedB) {
    if (speedA > 1000) speedA = 1000; // Giới hạn tốc độ
    if (speedB > 1000) speedB = 1000;

    // Động cơ 1: IN1 = 0, IN2 = 1
    GPIOA->ODR &= ~(1 << IN1_PIN);
    GPIOA->ODR |= (1 << IN2_PIN);

    // Động cơ 2: IN3 = 0, IN4 = 1
    GPIOA->ODR &= ~(1 << IN3_PIN);
    GPIOB->ODR |= (1 << IN4_PIN);

    // Đặt tốc độ cho PWM
    TIM2->CCR1 = speedA;
    TIM4->CCR2 = speedB;
}

void Motor_Stop(void) {
    // Động cơ 1: IN1 = 0, IN2 = 0
    GPIOA->ODR &= ~((1 << IN1_PIN) | (1 << IN2_PIN));

    // Động cơ 2: IN3 = 0, IN4 = 0
    GPIOA->ODR &= ~(1 << IN3_PIN);
    GPIOB->ODR &= ~(1 << IN4_PIN);

    // Đặt tốc độ = 0
    TIM2->CCR1 = 0;
    TIM4->CCR2 = 0;
}

void Motor_TurnRight(uint16_t speed) {
    if (speed > 1000) speed = 1000; // Giới hạn tốc độ

    // Động cơ bên phải (dừng): IN3 = 0, IN4 = 0
    GPIOA->ODR &= ~(1 << IN3_PIN);
    GPIOB->ODR &= ~(1 << IN4_PIN);

    // Động cơ bên trái (tiến): IN1 = 1, IN2 = 0
    GPIOA->ODR |= (1 << IN1_PIN);
    GPIOA->ODR &= ~(1 << IN2_PIN);

    // Đặt tốc độ cho PWM
    TIM2->CCR1 = speed;
    TIM4->CCR2 = 0;
}

void Motor_TurnLeft(uint16_t speed) {
    if (speed > 1000) speed = 1000; // Giới hạn tốc độ

    // Động cơ bên trái (dừng): IN1 = 0, IN2 = 0
    GPIOA->ODR &= ~((1 << IN1_PIN) | (1 << IN2_PIN));

    // Động cơ bên phải (tiến): IN3 = 1, IN4 = 0
    GPIOA->ODR |= (1 << IN3_PIN);
    GPIOB->ODR &= ~(1 << IN4_PIN);

    // Đặt tốc độ cho PWM
    TIM2->CCR1 = 0;
    TIM4->CCR2 = speed;
}

int main(void) {
    // 1. Khởi tạo các chức năng
    GPIO_Trig_Echo_Init();  // Khởi tạo GPIO cho đo khoảng cách
    Motor_Init();
    UART_Init();            // Khởi tạo UART
    I2C1_Init();            // Khởi tạo I2C
    lcd_init();             // Khởi tạo LCD
    TIM2_PWM_Init();
    TIM4_PWM_Init();
    Motor_Stop();
    SystemInit();           // Khởi tạo hệ thống
    SysTick_Init(1000);     // 1 kHz = 1 ms (1000 tick mỗi giây)
    GPIO_Init();            // Khởi tạo GPIO
    EXTI_Init();            // Khởi tạo ngắt EXTI

    // 2. Vòng lặp chính
    while (1) {
        if (system_state == 1) {  // Nếu hệ thống đang hoạt động
            // Đo khoảng cách
            float distance = measure_distance();

            // Gửi kết quả qua UART
            char uart_buffer[50];
            if (distance < 0) {
                sprintf(uart_buffer, "Error: Timeout\r\n");
            } else {
                sprintf(uart_buffer, "Distance: %.2f cm\r\n", distance);
            }
            UART_SendString(uart_buffer);

            // Hiển thị khoảng cách trên LCD
            char lcd_buffer[16];
            lcd_put_cur(0, 0);
            if (distance < 0) {
                lcd_send_string("Error!        ");  // Hiển thị lỗi
            } else if (distance > 1 && distance < 5) {
            	GPIOA->ODR &= ~(1 << 6);
            	toggleLED((1 << 7), 10);             //Gọi hàm toggle nhấp nháy led PA7 với tần số 5hz
                lcd_send_string("sys: 1 - obs: 1   ");
                lcd_put_cur(1, 0);
                sprintf(lcd_buffer, "  %.2f cm", distance);
                lcd_send_string(lcd_buffer);

                Motor_Stop();
                Motor_Backward(100,100);
            }
            else if (distance >=5 && distance < 10) {
            	GPIOA->ODR &= ~(1 << 6);
            	toggleLED((1 << 7), 5);             //Gọi hàm toggle nhấp nháy led PA7 với tần số 5hz
                lcd_send_string("sys: 1 - obs: 1   ");
                lcd_put_cur(1, 0);
                sprintf(lcd_buffer, "  %.2f cm", distance);
                lcd_send_string(lcd_buffer);

                Motor_Stop();
                Motor_Backward(100,100);
            }
            else if (distance >=10 && distance < 20) {
            	GPIOA->ODR &= ~(1 << 6);
            	toggleLED((1 << 7), 1);             //Gọi hàm toggle nhấp nháy led PA7 với tần số 5hz
                lcd_send_string("sys: 1 - obs: 1   ");
                lcd_put_cur(1, 0);
                sprintf(lcd_buffer, "  %.2f cm", distance);
                lcd_send_string(lcd_buffer);

                Motor_Stop();
                Motor_Backward(100,10);
            } else {
                lcd_send_string("sys: 1 - obs: 0   ");
                lcd_put_cur(1, 0);
                sprintf(lcd_buffer, "  %.2f cm", distance);
                lcd_send_string(lcd_buffer);

                Motor_Forward(100,100);
            }
            // 2. Kiểm tra và thay đổi trạng thái LED
            GPIOA->ODR &= ~(1 << 7);
            toggleLED((1 << 6), 1);  // Nhấp nháy LED PA6 với tần số 1 Hz

            // Delay ngắn trước lần đo tiếp theo
            delay_ms(200);
        } else {
            // Hệ thống đang dừng đo
            lcd_put_cur(0, 0);
            lcd_send_string("sys: 0 ");
            Motor_Stop();
        }
    }
}







