#include "stm32f4xx.h"
#include <stdio.h>
#include <math.h>

#define ADC_MAX      4095.0
#define VREF         3.3
#define SCALE_VOLT   100

#define FILTER_SIZE  5

char uart_buffer[64];

void GPIO_Config(void) {
    RCC->AHB1ENR |= (1 << 0);
    GPIOA->MODER |= (3 << (0 * 2)); // PA0 analog
    GPIOA->MODER &= ~(3 << (2 * 2));
    GPIOA->MODER |=  (2 << (2 * 2)); // PA2 AF
    GPIOA->AFR[0]  |= (7 << (2 * 4)); // AF7 for USART2
}

void UART2_Config(void) {
    RCC->APB1ENR |= (1 << 17); // Enable USART2
    USART2->BRR = 16000000 / 9600;
    USART2->CR1 = (1 << 3) | (1 << 13); // TE + UE
}

void UART2_SendChar(char c) {
    while (!(USART2->SR & (1 << 7)));
    USART2->DR = c;
}

void UART2_SendString(char *str) {
    while (*str) UART2_SendChar(*str++);
}

void ADC_Config(void) {
    RCC->APB2ENR |= (1 << 8); // Enable ADC1
    ADC1->CR2 = 0;
    ADC1->SQR3 = 0;
    ADC1->SMPR2 |= (4 << 0);
    ADC1->CR2 |= (1 << 0);
}

uint16_t ADC_Read(void) {
    ADC1->CR2 |= (1 << 30);
    while (!(ADC1->SR & (1 << 1)));
    return ADC1->DR;
}

int ADC_to_mV(uint16_t adc_value) {
    float voltage = (adc_value * VREF) / ADC_MAX;
    return (int)(voltage * SCALE_VOLT);
}

int Estimate_Gas_PPM(int voltage_mV) {
    float V_RL = (float)voltage_mV / SCALE_VOLT;
    float V_c = 5.0f;
    float R_L = 6000.0f; // Điều chỉnh theo mạch
    float R_s = (V_c - V_RL) / V_RL * R_L;
    float R_0 = 65000.0f; // Hiệu chỉnh thực nghiệm
    float ratio = R_s / R_0;
    float A = 200.0f;
    float B = -0.75f;
    float ppm = A * pow(ratio, B);
    return (int)ppm;
}
// Sắp xếp mảng tăng dần
void Sort_Array(int *arr, int size) {
    for (int i = 0; i < size-1; i++) {
        for (int j = i+1; j < size; j++) {
            if (arr[i] > arr[j]) {
                int tmp = arr[i];
                arr[i] = arr[j];
                arr[j] = tmp;
            }
        }
    }
}

// Trả về median từ mảng
int Median_Filter(int *arr, int size) {
    int temp[FILTER_SIZE];
    for (int i = 0; i < size; i++) temp[i] = arr[i];
    Sort_Array(temp, size);
    return temp[size / 2];
}

int main(void) {
    GPIO_Config();
    ADC_Config();
    UART2_Config();

    uint16_t adc_value;
    int voltage_mV;
    int gas_ppm;

    int ppm_buffer[FILTER_SIZE] = {0};
    int index = 0;

    while (1) {
        adc_value = ADC_Read();
        voltage_mV = ADC_to_mV(adc_value);
        gas_ppm = Estimate_Gas_PPM(voltage_mV);
        float V_RL = (float)voltage_mV / SCALE_VOLT;
        float R_s = (5.0 - V_RL) / V_RL * 6000.0;
        int VRL = (int)V_RL;
        int Rs = (int)R_s;
        // Lưu vào mảng vòng tròn
        ppm_buffer[index] = gas_ppm;
        index = (index + 1) % FILTER_SIZE;

        int filtered_ppm = Median_Filter(ppm_buffer, FILTER_SIZE);

        sprintf(uart_buffer, " %d\r\n", filtered_ppm);
        UART2_SendString(uart_buffer);

        for (volatile int i = 0; i < 1000000; i++);
    }
}
