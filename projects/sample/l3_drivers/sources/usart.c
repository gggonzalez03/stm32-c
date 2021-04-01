#include <stdlib.h>

#include "usart.h"
#include "stm32f411xe.h"

#include "stm_peripherals.h"

typedef USART_TypeDef usart_td;
typedef IRQn_Type irq_number_td;

typedef struct
{
  stm_peripheral_e usart;
  usart_td *registers;
  irq_number_td irq_number;
  bool low_power_mode;
} usart_s;

/*************************************************************************
 * 
 *                        PRIVATE DATA DEFINITIONS
 *
 *************************************************************************/

static const usart_s usarts[] = {
  { STM_PERIPHERAL_USART1, USART1, USART1_IRQn },
  { STM_PERIPHERAL_USART2, USART2, USART2_IRQn },
  { STM_PERIPHERAL_USART6, USART6, USART6_IRQn }
};

 /*************************************************************************
 * 
 *                      PRIVATE FUNCTION DEFINITIONS
 *
 *************************************************************************/


/*************************************************************************
 * 
 *                        PRIVATE DATA DEFINITIONS
 *
 *************************************************************************/

/*************************************************************************
 * 
 *                        PUBLIC DATA DEFINITIONS
 *
 *************************************************************************/

bool usart__init(usart_e usart_id, uint32_t peripheral_clock, uint32_t baud_rate, bool low_power_mode)
{
  usart_s usart = usarts[usart_id];
  stm_peripheral__power_on_peripheral(usart.usart, low_power_mode);

  uint32_t receiver_enable = (1UL << 2);
  uint32_t transmitter_enable = (1UL << 3);
  uint32_t usart_enable = (1UL << 13);

  /**
   * Example: peripheral_clock = 84Mhz, baud_rate = 115200
   *
   * 84000000 / (16 * 115200) = 45.5729166667
   * 45 => 0x02D
   * .5729166667*16 => 9.17 => 9 => 0x9
   * BRR[15:0] = 0x02D9
   *
   * See Chapter 19.3.4 for the formula and some examples
   **/
  float usart_div = peripheral_clock / (16 * baud_rate);
  uint32_t mantissa = (uint32_t)usart_div;
  uint32_t fractional_div = (uint32_t)((usart_div - (float)mantissa) * 16);
  uint32_t final_div = ((mantissa << 4) & 0xFFF) | ((fractional_div << 0) & 0xF);

  usart.registers->BRR = (final_div & 0xFFFF);
  usart.registers->CR1 |= (transmitter_enable);
  usart.registers->CR1 |= (receiver_enable);
  usart.registers->CR1 |= usart_enable;

  return true;
}

bool usart__polled_receive(usart_e usart_id, char* const byte)
{
  usart_s usart = usarts[usart_id];

  while (!(usart.registers->SR & (1UL << 5)));
  *byte = (char)usart.registers->DR;

  return true;
}

bool usart__polled_transmit(usart_e usart_id, char byte)
{
  usart_s usart = usarts[usart_id];

  usart.registers->DR = byte & 0xFF;
  while (!(usart.registers->SR & (1UL << 6)));

  return true;
}

void usart__enable_rx_interrrupt(usart_e usart_id)
{
  usart_s usart = usarts[usart_id];

  uint32_t rx_interrupt_enable = (1UL << 5);

  usart.registers->CR1 |= rx_interrupt_enable;
  NVIC_EnableIRQ(usart.irq_number);
}

void usart__enable_tx_interrrupt(usart_e usart_id)
{
  usart_s usart = usarts[usart_id];

  uint32_t tx_interrupt_enable = (1UL << 7);

  usart.registers->CR1 |= tx_interrupt_enable;
  NVIC_EnableIRQ(usart.irq_number);
}