/**
  ******************************************************************************
  *          F05_RCC.h
  *          A.Chiffa Dev.
  *          Header file for F05_RCC module.
  *          For STM32F05x MCU 
  *          V1.0
  * @attention
  *          Redistributions of source code must retain the above copyright notice,
  *          this list of conditions and the following disclaimer.
  *          Commercial use is possible only by agreement with the developers.
  * @Mail    ArsChiffa@gmail.com
  *
  ******************************************************************************
*/
#ifndef F4_RCC_h
#define F4_RCC_h 

#include "stm32f0xx.h"

uint32_t ahb_clock=8000000;
uint32_t apb_clock=8000000;
uint32_t tim_clock=8000000;

   /** Step 0  - Flash acceleration and latency */
#define FLASH_LATENCY_1WS()    { FLASH->ACR |= FLASH_ACR_LATENCY; }   //Set 1 wait state
#define FLASH_LATENCY_0WS()    { FLASH->ACR &=~FLASH_ACR_LATENCY; }   //Set 0 wait state                                                                      
#define FLASH_PREFETCH_EN()    { FLASH->ACR |= FLASH_ACR_PRFTBE;  }   //Turn On Flash prefetch(12 bytes per read)
#define FLASH_PREFETCH_OFF()   { FLASH->ACR &=~FLASH_ACR_PRFTBE;  }   //Turn Off Flash prefetch

   /** Step II - Turn on oscillator */ 
#define  HSI_START()  { RCC->CR |= RCC_CR_HSION; while(!(RCC->CR & RCC_CR_HSIRDY)); }
#define  HSI_STOP()   { RCC->CR &=~RCC_CR_HSION;                                    }
#define  HSE_START()  { RCC->CR |= RCC_CR_HSEON; while(!(RCC->CR & RCC_CR_HSERDY)); }
#define  HSE_STOP()   { RCC->CR &=~RCC_CR_HSEON; 

   /** Step II - PLL configuration */ 
//Main PLL  multiplication factor should be in range [2;16]
#define  PLL_MUL(MUL)       { MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLMUL, (((MUL-2UL)&0xF)<<RCC_CFGR_PLLMUL_Pos));}
#define  PLLSourse(PLLSRC)  { MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLSRC, PLLSRC);}
//HSE Division factor should be in range [1;16]
#define  HSE_PREDIV(DIV) { MODIFY_REG(RCC->CFGR2, RCC_CFGR2_PREDIV, ((DIV-1UL)&0xF) );}
   /**@Arguments PLLSourse(PLLSRC) */
#define  PLLSRC_HSI_DIV2    RCC_CFGR_PLLSRC_HSI_DIV2
#define  PLLSRC_HSE_PREDIV  RCC_CFGR_PLLSRC_HSE_PREDIV

   /** Step III - PLL Start */ 
#define  PLL_START()     {RCC->CR |= RCC_CR_PLLON; while(!(RCC->CR & RCC_CR_PLLRDY)); }

   /** Step IV - Set Bus divisors */
//Division SYSCLK clock for AHB
#define SET_AHB_PRESCALLER(AHB_DIV)      {MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE,   AHB_DIV); }
//Division AHB clock for APB1/APB2
#define SET_APB_PRESCALLER(APB_DIV)      {MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE,   APB_DIV); }
   /**@Arguments SET_APB_PRESCALLER(APB_DIV) */   
#define APB_DIV1    RCC_CFGR_PPRE_DIV1 
#define APB_DIV2    RCC_CFGR_PPRE_DIV2 
#define APB_DIV4    RCC_CFGR_PPRE_DIV4 
#define APB_DIV8    RCC_CFGR_PPRE_DIV8 
#define APB_DIV16   RCC_CFGR_PPRE_DIV16
   /**@Arguments SET_AHB_PRESCALLER(AHB_DIV) */
#define AHB_DIV1    RCC_CFGR_HPRE_DIV1  
#define AHB_DIV2    RCC_CFGR_HPRE_DIV2  
#define AHB_DIV4    RCC_CFGR_HPRE_DIV4  
#define AHB_DIV8    RCC_CFGR_HPRE_DIV8  
#define AHB_DIV16   RCC_CFGR_HPRE_DIV16 
#define AHB_DIV64   RCC_CFGR_HPRE_DIV64 
#define AHB_DIV128  RCC_CFGR_HPRE_DIV128
#define AHB_DIV256  RCC_CFGR_HPRE_DIV256
#define AHB_DIV512  RCC_CFGR_HPRE_DIV512

   /** Step V - Connect Bus to Clock */
#define SET_SYSCLK_SRC(SysClkSRC)  {MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, SysClkSRC);}
   /**@Arguments SET_SYSCLK_SRC(SysClkSRC) */
#define ClkSourceHSI  RCC_CFGR_SW_HSI 
#define CLKSourceHSE  RCC_CFGR_SW_HSE
#define CLKSourcePLL  RCC_CFGR_SW_PLL



void BUSClockUpdate(void){
uint8_t ppre_bits, hpre_bits, apb_pre, ahb_pre;
 
  SystemCoreClockUpdate();                                  //SYSCLK  freq
  ppre_bits = ((RCC->CFGR & RCC_CFGR_PPRE)>>RCC_CFGR_PPRE_Pos);
  hpre_bits = ((RCC->CFGR & RCC_CFGR_HPRE)>>RCC_CFGR_HPRE_Pos); 
  apb_pre   = (ppre_bits ? (0x2UL<<(ppre_bits&0x3)) : 0x1); //Mask MSB and exp. 2 by the rest 
  ahb_pre   = (hpre_bits ? (0x2UL<<(hpre_bits&0x7)) : 0x1); //Mask MSB and exp. 2 by the rest
  ahb_pre   = ((ahb_pre>0x10) ? (ahb_pre<<1UL) : ahb_pre);  //Take stock doule step of AHB from 16
  ahb_clock = SystemCoreClock/ahb_pre;                      //AHB freq
  apb_clock = ahb_clock/apb_pre;                            //APB freq
  tim_clock = ((apb_pre==0x1) ? apb_clock : apb_clock*0x2); //Timers freq
}


void RCC_48HSI(void){
  FLASH_LATENCY_1WS();
  FLASH_PREFETCH_EN();
  HSI_START();
  PLL_MUL(12);
  PLLSourse(PLLSRC_HSI_DIV2);
  PLL_START();   
  SET_AHB_PRESCALLER(AHB_DIV1);
  SET_APB_PRESCALLER(APB_DIV1);      
  SET_SYSCLK_SRC(CLKSourcePLL);    
  BUSClockUpdate();
}


void RCC_TimClock( TIM_TypeDef *TIMx ){
  
 switch((uint32_t)TIMx){  
   case (uint32_t)TIM1  :  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN  ; break;
   case (uint32_t)TIM2  :  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN  ; break;
   case (uint32_t)TIM3  :  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN  ; break;
   case (uint32_t)TIM6  :  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN  ; break;
   case (uint32_t)TIM14 :  RCC->APB1ENR |= RCC_APB1ENR_TIM14EN ; break;
   case (uint32_t)TIM15 :  RCC->APB2ENR |= RCC_APB2ENR_TIM15EN ; break;
   case (uint32_t)TIM16 :  RCC->APB2ENR |= RCC_APB2ENR_TIM16EN ; break;
   case (uint32_t)TIM17 :  RCC->APB2ENR |= RCC_APB2ENR_TIM17EN ; break;
  } 
}



#endif