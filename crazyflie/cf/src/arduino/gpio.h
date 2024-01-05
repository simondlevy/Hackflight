#pragma once

#include <stm32fxxx.h>

typedef struct {
    uint32_t periph;
    GPIO_TypeDef* port;
    uint16_t pin;
    int8_t adcCh; // -1 means no ADC available for this pin
} GPIO_Mapping_t;


static const uint32_t LED_PERIPH = RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD;

// Mapping between pin number, real GPIO and ADC channel
static GPIO_Mapping_t GPIOMappings[] = {

    // LEDs
    {.periph= LED_PERIPH, .port= GPIOD, .pin=GPIO_Pin_2,  
        .adcCh=-1},              // LED_BLUE_L
    {.periph= LED_PERIPH, .port= GPIOC, .pin=GPIO_Pin_1,  
        .adcCh=-1},              // LED_GREEN_L
    {.periph= LED_PERIPH, .port= GPIOC, .pin=GPIO_Pin_0,  
        .adcCh=-1},              // LED_RED_L
    {.periph= LED_PERIPH, .port= GPIOC, .pin=GPIO_Pin_2,  
        .adcCh=-1},              // LED_GREEN_R
    {.periph= LED_PERIPH, .port= GPIOC, .pin=GPIO_Pin_3,  
        .adcCh=-1},              // LED_RED_R

    // Decks
    {.periph= RCC_AHB1Periph_GPIOC, .port= GPIOC, .pin=GPIO_Pin_11, 
        .adcCh=-1},             // Deck RX1 
    {.periph= RCC_AHB1Periph_GPIOC, .port= GPIOC, .pin=GPIO_Pin_10, 
        .adcCh=-1},             // Deck TX1 
    {.periph= RCC_AHB1Periph_GPIOB, .port= GPIOB, .pin=GPIO_Pin_7,  
        .adcCh=-1},             // Deck SDA 
    {.periph= RCC_AHB1Periph_GPIOB, .port= GPIOB, .pin=GPIO_Pin_6,  
        .adcCh=-1},             // Deck SCL 
    {.periph= RCC_AHB1Periph_GPIOB, .port= GPIOB, .pin=GPIO_Pin_8,  
        .adcCh=-1},            // Deck IO1 
    {.periph= RCC_AHB1Periph_GPIOB, .port= GPIOB, .pin=GPIO_Pin_5,  
        .adcCh=-1},            // Deck IO2 
    {.periph= RCC_AHB1Periph_GPIOB, .port= GPIOB, .pin=GPIO_Pin_4,  
        .adcCh=-1},            // Deck IO3 
    {.periph= RCC_AHB1Periph_GPIOC, .port= GPIOC, .pin=GPIO_Pin_12, 
        .adcCh=-1},            // Deck IO4 
    {.periph= RCC_AHB1Periph_GPIOA, .port= GPIOA, .pin=GPIO_Pin_2,  
        .adcCh=ADC_Channel_2}, // Deck TX2 
    {.periph= RCC_AHB1Periph_GPIOA, .port= GPIOA, .pin=GPIO_Pin_3,  
        .adcCh=ADC_Channel_3}, // Deck RX2 
    {.periph= RCC_AHB1Periph_GPIOA, .port= GPIOA, .pin=GPIO_Pin_5,  
        .adcCh=ADC_Channel_5}, // Deck SCK 
    {.periph= RCC_AHB1Periph_GPIOA, .port= GPIOA, .pin=GPIO_Pin_6,  
        .adcCh=ADC_Channel_6}, // Deck MISO 
    {.periph= RCC_AHB1Periph_GPIOA, .port= GPIOA, .pin=GPIO_Pin_7,  
        .adcCh=ADC_Channel_7}, // Deck MOSI 

    // Bolt IMU SPI
    {.periph= RCC_AHB1Periph_GPIOB, .port= GPIOB, .pin=GPIO_Pin_0,  
        .adcCh=-1},              // Bolt gyro CS
    {.periph= RCC_AHB1Periph_GPIOB, .port= GPIOB, .pin=GPIO_Pin_1,  
        .adcCh=-1},              // Bolt accel CS

};

