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
 * Enable PLL to get 100MHz clock source
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
  uint32_t cfgr_pll_divider_p = (2UL << 16);
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
  clock__enable_pll_100MHz();
  clock__select_pll_as_source();
}

/**
 * Get the core clock in Hertz
 * @return uint32_t clock frequency in Hertz
 **/
uint32_t clock__get_core_clock_frq()
{
  return 100 * 1000 * 1000;
}