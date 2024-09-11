/*
 * USB Serial
 * 
 * Copyright (c) 2020 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 * 
 * Hardware defintions
 */

#pragma once

// --- USB pins and clocks

#define USB_DP_PORT GPIOA
#define USB_DP_PIN GPIO12
#define USB_PORT_RCC RCC_GPIOA

// --- USART pins and clocks

#define USART USART2
#define USART_RX_DATA_REG USART2_RDR
#define USART_TX_DATA_REG USART2_TDR
#define USART_PORT GPIOA
#define USART_TX_GPIO GPIO2
#define USART_RX_GPIO GPIO3
#define USART_RCC RCC_USART2

// --- USART DMA channels and clocks

#define USART_DMA DMA1
#define USART_DMA_TX_CHAN 4
#define USART_DMA_RX_CHAN 5
#define USART_DMA_RCC RCC_DMA