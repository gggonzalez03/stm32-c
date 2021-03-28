#include "clock.h"
#include "stm32f411xe.h"

/**
 * Enable High Speed External (HSE)
 **/
static void clock__enable_hse()
{
  uint32_t cr_hse_on = (1UL << 16);
  uint32_t cr_hse_ready = (1UL << 17);

  RCC->CR |= cr_hse_on;
  while (!(RCC->CR & cr_hse_ready));
}

/**
 * Turn on power to APB1
 **/
static void clock__power_on_apb1() {
  RCC->APB1ENR |= (1UL << 28);
}

/**
 * Set APB1 Prescaler
 * WARNING: APB1 should have a prescaler such that the resulting
 * frequency is less than or equal to 50MHz. This is a low speed
 * peripheral bus.
 **/
static void clock__set_apb1_prescaler(uint8_t prescaler) {
  RCC->CFGR &= ~(7UL << 10);
  RCC->CFGR |= (prescaler << 10);
}

/**
 * Set APB2 Prescaler
 **/
static void clock__set_apb2_prescaler(uint8_t prescaler) {
  RCC->CFGR &= ~(7UL << 13);
  RCC->CFGR |= (prescaler << 13);
}

/**
 * Set AHB Prescaler
 **/ 
static void clock__set_ahb_prescaler(uint8_t prescaler) {
  RCC->CFGR &= ~(0xF << 4);
  RCC->CFGR |= (prescaler << 4);
}

/**
 * Enable PLL to get 100MHz clock source
 * NOTE: If this frequency is used, make sure to change the value of VOS[1:0]
 * Refer to Chapter 5.4.1
 * Also change clock__get_core_clock_frq() return value
 **/ 
static void clock__enable_pll_100MHz()
{
  /**
   * f_pll_input = 25MHz (External Crystal)
   * f_pll_desired = 100MHz
   * 
   * f_vco_input = f_pll_input / M              => 1MHz = 25MHz / 25MHz
   * f_vco_clk = f_pll_input * (PLLN /  PLLM)   => 200MHz = f_pll_input * (200 / 25)
   * f_pll_output = f_vco_clk / (PLLP)          => f_pll_desired = 200MHz / (2)
   * f_usb_otg_fs_and_sdio = f_vco_clk / PLLQ   => 40MHz = f_vco_clk / 5
   **/
  uint32_t cfgr_pll_divider_m = (25UL << 0);
  uint32_t cfgr_pll_divider_n = (200UL << 6);
  uint32_t cfgr_pll_divider_p = (0UL << 16);
  uint32_t cfgr_pll_divider_q = (5UL << 24);
  uint32_t cfgr_pll_src_hse = (1UL << 22);

  uint32_t cr_pll_on = (1UL << 24);
  uint32_t cr_pll_ready = (1UL << 25);

  RCC->PLLCFGR &= ~(0xFFFFFFFF);
  RCC->PLLCFGR |= (cfgr_pll_src_hse | cfgr_pll_divider_m | cfgr_pll_divider_n | cfgr_pll_divider_p | cfgr_pll_divider_q);

  RCC->CR |= cr_pll_on;
  while (!(RCC->CR & cr_pll_ready));
}

/**
 * Enable PLL to get 84MHz clock source
 * NOTE:  84MHz is chosen because
 *        1. VOS reset value will not need to be changed (See Chapter 5.4.1)
 *        2. More importantly, SysTick calibration value is fixed to 10500 (See Chapter 10.1.2)
 *           This allows us to have 1ms SysTick interrupt with no additional configurations.
 **/ 
static void clock__enable_pll_84MHz()
{
  /**
   * f_pll_input = 25MHz (External Crystal)
   * f_pll_desired = 84MHz
   * 
   * f_vco_input = f_pll_input / M              => 1MHz = 25MHz / 25MHz
   * f_vco_clk = f_pll_input * (PLLN /  PLLM)   => 168MHz = f_pll_input * (168 / 25)
   * f_pll_output = f_vco_clk / (PLLP)          => f_pll_desired = 168MHz / (2)
   * f_usb_otg_fs_and_sdio = f_vco_clk / PLLQ   => 42MHz = f_vco_clk / 4
   **/
  uint32_t cfgr_pll_divider_m = (25UL << 0);
  uint32_t cfgr_pll_divider_n = (168UL << 6);
  uint32_t cfgr_pll_divider_p = (0UL << 16);
  uint32_t cfgr_pll_divider_q = (4UL << 24);
  uint32_t cfgr_pll_src_hse = (1UL << 22);

  uint32_t cr_pll_on = (1UL << 24);
  uint32_t cr_pll_ready = (1UL << 25);

  RCC->PLLCFGR &= ~(0xFFFFFFFF);
  RCC->PLLCFGR |= (cfgr_pll_src_hse | cfgr_pll_divider_m | cfgr_pll_divider_n | cfgr_pll_divider_p | cfgr_pll_divider_q);

  RCC->CR |= cr_pll_on;
  while (!(RCC->CR & cr_pll_ready));
}

/**
 * Select PLL output as the clock source for the system
 **/
static void clock__select_pll_as_source()
{
  uint32_t cfgr_system_clock_sw_mask = (3UL << 0);
  uint32_t cfgr_system_clock_select_pll = (2UL << 0);
  uint32_t cfgr_system_clock_pll_is_selected = (2UL << 2);
  
  RCC->CFGR &= ~cfgr_system_clock_sw_mask;
  RCC->CFGR |= cfgr_system_clock_select_pll;

  while (!(RCC->CFGR & cfgr_system_clock_pll_is_selected));
}

/**
 * Initialize system clock
 **/ 
void clock__init_system_clock()
{
  clock__enable_hse();
  clock__power_on_apb1();
  clock__set_apb1_prescaler(4U);
  clock__set_apb2_prescaler(0U);
  clock__set_ahb_prescaler(0U);
  clock__enable_pll_84MHz();
  clock__select_pll_as_source();
}

/**
 * Get the core clock in Hertz
 * @return uint32_t clock frequency in Hertz
 **/
uint32_t clock__get_core_clock_frq()
{
  return 84 * 1000 * 1000;
}