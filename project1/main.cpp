#include"stm32f051x8.h"
#include"system_stm32f0xx.h"
#include"stm32f0xx.h"
#include<cstring>

extern void NVIC_EnableIRQ(IRQn_Type IRQn);         //Enables an interrupt or exception.
extern void NVIC_DisableIRQ(IRQn_Type IRQn);        //Disables an interrupt or exception. 

enum class PinMode : uint32_t {
    INPUT = 0b00,
    OUTPUT = 0b01,
    ALTERNATIVE = 0b10,
    ANALOG = 0b11,
};

enum class SpeedMode : uint32_t {
    LOW = 0b00,
    FAST = 0b01,
    VERYFAST = 0b10,
};

enum class PullMode : uint32_t {
    NOPULL = 0b00,
    PULLUP = 0b01,
    PULLDOWN = 0b10,
};

void configurePin (
    GPIO_TypeDef* GPIOx, uint32_t pin, PinMode pinMode,
    SpeedMode speedMode, PullMode pullMode, uint32_t alternativeFunc) {
    
    GPIOx->MODER &= ~(0b11 << (2 * pin));
    GPIOx->MODER |= uint32_t(pinMode) << (2 * pin);

    GPIOx->OSPEEDR &= ~(0b11 << (2 * pin));
    GPIOx->OSPEEDR |= uint32_t(speedMode) << (2 * pin);

    GPIOx->PUPDR &= ~(0b11 << (2 * pin));
    GPIOx->PUPDR |= uint32_t(pullMode) << (2 * pin);

    GPIOx->AFR[pin / 8] &= ~(0x0ful << ((pin % 8ul) * 4ul));
    GPIOx->AFR[pin / 8] |= uint32_t(alternativeFunc) << ((pin % 8ul) * 4ul); 

}

const uint8_t* str = (const uint8_t*)"I'm your father\n";

void uSART1Init (uint32_t baudRate) {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    NVIC_DisableIRQ(USART1_IRQn); //выключаем на время настройки

    USART1->BRR = 8'000'000 / baudRate; // установили скорость прием-передачи в сек [бод]
    USART1->CR1 |= USART_CR1_TE; // включили передачу
    USART1->CR1 |= USART_CR1_UE; // вклuчили usart

    NVIC_EnableIRQ(USART1_IRQn);
}

extern "C" void USART1_IRQHandler (void) {
    static uint32_t cnt = 0;

    NVIC_DisableIRQ(USART1_IRQn);
    if (cnt < 17) {
        USART1->TDR = str[cnt++];
        USART1->CR1 |= USART_CR1_TXEIE;
    }
    NVIC_EnableIRQ(USART1_IRQn);
}

int main() {
    // включаем GPIOB
    RCC->AHBENR  |= RCC_AHBENR_GPIOBEN;
    uSART1Init(9600);
    configurePin(GPIOB, 6, PinMode::OUTPUT, SpeedMode::LOW, PullMode::NOPULL, 0);
    USART1->CR1 |= USART_CR1_TXEIE;
    while(1){;
    }
}
