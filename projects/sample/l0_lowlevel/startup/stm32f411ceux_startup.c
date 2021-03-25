#include <stdint.h>

#include "stm32f411xe.h"
#include "clock.h"

extern void main(void);

void Default_Handler(void);
void Reset_Handler(void);

void NMI_Handler 					            (void) __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler 				        (void) __attribute__ ((weak, alias("Default_Handler")));
void MemManage_Handler 				        (void) __attribute__ ((weak, alias("Default_Handler")));
void BusFault_Handler 				        (void) __attribute__ ((weak, alias("Default_Handler")));
void UsageFault_Handler 			        (void) __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler 					            (void) __attribute__ ((weak, alias("Default_Handler")));
void DebugMon_Handler 				        (void) __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler   				        (void) __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler  				        (void) __attribute__ ((weak, alias("Default_Handler")));
void WWDG_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void PVD_IRQHandler                   (void) __attribute__ ((weak, alias("Default_Handler")));
void TAMP_STAMP_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void RTC_WKUP_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void FLASH_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void RCC_IRQHandler                   (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI0_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI1_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI2_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI3_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI4_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream0_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream1_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream2_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream3_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream4_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream5_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream6_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void ADC_IRQHandler                   (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI9_5_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_BRK_TIM9_IRQHandler         (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_UP_TIM10_IRQHandler         (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_TRG_COM_TIM11_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_CC_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM2_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM3_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM4_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C1_EV_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C1_ER_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C2_EV_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C2_ER_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI1_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI2_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void USART1_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void USART2_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI15_10_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void RTC_Alarm_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void OTG_FS_WKUP_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream7_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void SDIO_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM5_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI3_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream0_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream1_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream2_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream3_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream4_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void OTG_FS_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream5_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream6_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream7_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void USART6_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C3_EV_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C3_ER_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void FPU_IRQHandler                   (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI4_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI5_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));

extern void* __base_stack;

void* interrupt_vector_table[] __attribute__((section(".interrupt_vector_table"))) = {
  (void*)&__base_stack,
  (void*)Reset_Handler,
  (void*)NMI_Handler,
  (void*)HardFault_Handler,
  (void*)MemManage_Handler,
  (void*)BusFault_Handler,
  (void*)UsageFault_Handler,
  0,
  0,
  0,
  0,
  (void*)SVC_Handler,
  (void*)DebugMon_Handler,
  0,
  (void*)PendSV_Handler,
  (void*)SysTick_Handler,
  (void*)WWDG_IRQHandler,
  (void*)PVD_IRQHandler,
  (void*)TAMP_STAMP_IRQHandler,
  (void*)RTC_WKUP_IRQHandler,
  (void*)FLASH_IRQHandler,
  (void*)RCC_IRQHandler,
  (void*)EXTI0_IRQHandler,
  (void*)EXTI1_IRQHandler,
  (void*)EXTI2_IRQHandler,
  (void*)EXTI3_IRQHandler,
  (void*)EXTI4_IRQHandler,
  (void*)DMA1_Stream0_IRQHandler,
  (void*)DMA1_Stream1_IRQHandler,
  (void*)DMA1_Stream2_IRQHandler,
  (void*)DMA1_Stream3_IRQHandler,
  (void*)DMA1_Stream4_IRQHandler,
  (void*)DMA1_Stream5_IRQHandler,
  (void*)DMA1_Stream6_IRQHandler,
  (void*)ADC_IRQHandler,
  0,
  0,
  0,
  0,
  (void*)EXTI9_5_IRQHandler,
  (void*)TIM1_BRK_TIM9_IRQHandler,
  (void*)TIM1_UP_TIM10_IRQHandler,
  (void*)TIM1_TRG_COM_TIM11_IRQHandler,
  (void*)TIM1_CC_IRQHandler,
  (void*)TIM2_IRQHandler,
  (void*)TIM3_IRQHandler,
  (void*)TIM4_IRQHandler,
  (void*)I2C1_EV_IRQHandler,
  (void*)I2C1_ER_IRQHandler,
  (void*)I2C2_EV_IRQHandler,
  (void*)I2C2_ER_IRQHandler,
  (void*)SPI1_IRQHandler,
  (void*)SPI2_IRQHandler,
  (void*)USART1_IRQHandler,
  (void*)USART2_IRQHandler,
  0,
  (void*)EXTI15_10_IRQHandler,
  (void*)RTC_Alarm_IRQHandler,
  (void*)OTG_FS_WKUP_IRQHandler,
  0,
  0,
  0,
  0,
  (void*)DMA1_Stream7_IRQHandler,
  0,
  (void*)SDIO_IRQHandler,
  (void*)TIM5_IRQHandler,
  (void*)SPI3_IRQHandler,
  0,
  0,
  0,
  0,
  (void*)DMA2_Stream0_IRQHandler,
  (void*)DMA2_Stream1_IRQHandler,
  (void*)DMA2_Stream2_IRQHandler,
  (void*)DMA2_Stream3_IRQHandler,
  (void*)DMA2_Stream4_IRQHandler,
  0,
  0,
  0,
  0,
  0,
  0,
  (void*)OTG_FS_IRQHandler,
  (void*)DMA2_Stream5_IRQHandler,
  (void*)DMA2_Stream6_IRQHandler,
  (void*)DMA2_Stream7_IRQHandler,
  (void*)USART6_IRQHandler,
  (void*)I2C3_EV_IRQHandler,
  (void*)I2C3_ER_IRQHandler,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  (void*)FPU_IRQHandler,
  0,
  0,
  (void*)SPI4_IRQHandler,
  (void*)SPI5_IRQHandler,
};

static void startup__init_data_sram(void) {
  extern uint32_t __text_end__;
  extern uint32_t __data_end__;
  extern uint32_t __data_start__;

  uint8_t *src_flash = (uint8_t*)&__text_end__;
  uint8_t *dest_sram = (uint8_t*)&__data_start__;

  while (dest_sram < (uint8_t*)&__data_end__) {
    *dest_sram++ = *src_flash++;
  }
}

static void startup__init_bss_sram(void) {
  extern uint32_t __bss_start__;
  extern uint32_t __bss_end__;

  uint8_t *bss_sram_start = (uint8_t*)&__bss_start__;
  while (bss_sram_start < (uint8_t*)&__bss_end__) {
    *bss_sram_start++ = 0U;
  }
}

void Reset_Handler(void) {
  startup__init_data_sram();
  startup__init_bss_sram();
  
  clock__init_system_clock();
  
  main();
}

void Default_Handler(void) {
  while (1);
}