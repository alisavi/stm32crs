#include"stm32f051x8.h"
#include"system_stm32f0xx.h"
#include"stm32f0xx.h"

volatile uint32_t milliseconds = 0;

uint32_t CoreFrequency = 8'000'000 / 8;
uint32_t IntFrequency = 10'000;

extern "C" void SysTick_Handler (void) {
    static uint16_t cnt = 0;
    cnt++;

    if (cnt < IntFrequency / 1000) {
        milliseconds++;
        cnt -= IntFrequency / 1000;
    }
}

int main() {
    /***************** конфигурация пина *****************/
    // включаем GPIOA
    RCC->AHBENR  |= RCC_AHBENR_GPIOCEN;
   
    uint32_t pin = 8;

    // 0b00 - INPUT
    // 0b01 - OUTPUT
    // 0b10 - ALTRNATIVE
    // 0b11 - ANALOG
    // Установили режим работы в output
    uint32_t mode = 0b01;

    GPIOC->MODER &= ~(0b11 << (2 * pin));
    GPIOC->MODER |= mode << (2 * pin); 

    // 0b00 - LOW 
    // 0b01 - FAST
    // 0b10 - VERY FAST
    // скорость LOW
    uint32_t speed = 0b00;
    
    GPIOC->OSPEEDR &= ~(0b11 << (2 * pin));
    GPIOC->OSPEEDR |= speed << (2 * pin); 

    // 0b00 - NO PULL UP/DOWN
    // 0b01 - PULL UP
    // 0b10 - PULL DOWN
    // подтяжка NO
    // GPIO_TypeDef*
    uint32_t pull = 0b00;

    GPIOC->PUPDR &= ~(0b11 << (2 * pin));
    GPIOC->PUPDR |= pull << (2 * pin);

    // Альтернативная функция [0...15]
    uint32_t af = 0b00;
    GPIOC->AFR[pin / 8] &= ~(0x0ful << ((pin % 8ul) * 4ul));
    GPIOC->AFR[pin / 8] |= af << ((pin % 8ul) * 4ul);
    
    // включаем 
    GPIOC->ODR |= 1ul << pin;
    
    // /***************** конфигурация TIM6 *****************/
    // // включаем тактирование для 6 таймер
    // RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    // 
    // // устанавливаем макисмальное значение счетчика
    // TIM6->ARR = UINT16_MAX - 1;
    // // обнуляем счетчик таймера 
    // TIM6->CNT = 0;
    // // 8MHz / 8 = 1MHz
    // TIM6->PSC = 8 - 1;
    // 
    // 
    // // ждем когда таймер учтет наши пожелания
    // while (!(TIM6->SR & TIM_SR_UIF)) {}
    
    /***************** конфигурация SysTick *****************/
    // Включим отсчет и прерывания при переполнении
    SysTick->VAL  = CoreFrequency / IntFrequency - 1;
    SysTick->LOAD = CoreFrequency / IntFrequency - 1;
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;

    void initSysTick (uint16_t val) {
        SysTick->VAL  = val;
        SysTick->LOAD = UINT16_MAX - 1;
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    }
    void delaySysTick (uint32_t delayms) {
        uint32_t cntTicks = delayms * 1000;
        uint32_t cntCtrls = 0;
        while (cntCtrls*UINT16_MAX < cntTicks) {
            while (!(SysTick->CTRL &= SysTick_CTRL_COUNTFLAG_Msk)) {
            }
            cntCtrls += 1;
        }
    }

    bool flag = 0;

    while(1){
        uint32_t start = milliseconds;

        GPIOC->ODR &= 0ul << pin;
        GPIOC->ODR |= uint32_t(flag) << pin;

        while (milliseconds - start < 1001);

        flag = !flag;
    }
}
