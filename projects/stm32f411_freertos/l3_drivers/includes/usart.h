#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef USE_FREERTOS
#include "FreeRTOS.h"
#include "queue.h"
#endif

typedef enum
{
  USART__1 = 0,
  USART__2,
  USART__6
} usart_e;

/**
 * Initialize a USART peripheral
 * @param usart_id is the USART peripheral to initialize
 * @param peripheral_clock is the clock frequency in Hz that is fed into the USART
 * @param baud_rate is the desire baud rate
 * @param power_off_on_sleep whether to turn off the usart when in sleep mode or not
 * @return whether init is successful or not
 **/
bool usart__init(usart_e usart_id, uint32_t peripheral_clock, uint32_t baud_rate, bool power_off_on_sleep);

/**
 * Receive a character from the USART in blocking mode
 * @param usart_id is the USART peripheral to receive character from
 * @param byte is the pointer to where the character will be stored
 * @return whether receive is successful or not
 **/
bool usart__polled_receive(usart_e usart_id, char* const byte);

/**
 * Transmit a character over USART
 * @param usart_id is the USART peripheral to be used for transmit
 * @param byte is the character to transmit
 * @return whether transmit is successful or not
 **/
bool usart__polled_transmit(usart_e usart_id, char byte);

/**
 * Enable receive interrupt
 * @param usart_id is the USART peripheral to enable receive interrupt for
 **/
void usart__enable_rx_interrrupt(usart_e usart_id);

/**
 * @param usart_id is the USART peripheral to enable transmit interrupt for
 **/
void usart__enable_tx_interrrupt(usart_e usart_id);

#ifdef USE_FREERTOS
bool usart__enable_queues(usart_e usart_id, QueueHandle_t rx_queue, QueueHandle_t tx_queue);
bool usart__queued_receive(usart_e usart_id, char* const byte, uint32_t timeout);
bool usart__queued_transmit(usart_e usart_id, char byte, uint32_t timeout);
#endif