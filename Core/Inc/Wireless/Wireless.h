/*
 * Wireless.h
 *
 *  Created on: 6 feb. 2019
 *      Author: Cas Doornkamp
 * 
 * Implements functionality for main to communicate with the SX1280
 */

#ifndef WIRELESS_WIRELESS_H_
#define WIRELESS_WIRELESS_H_

#include <stdbool.h>
#include "SX1280_Constants.h"
#include "SX1280.h"
#include "main.h"

#define MAX_BUF_LENGTH 128

// SX1280 Supports ranges between 2.4GHz and 2.5 GHz, with steps of 1 MHz
// Page 20 Table 10-2 - Synthesizer frequency range
#define WIRELESS_FEEDBACK_CHANNEL ((float)( 50.0 /*  +50 MHz */ ))  // 2.450 GHz
#define WIRELESS_COMMAND_CHANNEL  ((float)(100.0 /* +100 MHz */ ))  // 2.500 GHz

SX1280 SX1280_TX_struct;
SX1280 SX1280_RX_struct;
SX1280 * SX_TX; // pointer to the datastruct for SX TX module
SX1280 * SX_RX; // pointer to the datastruct for SX RX module

// Public Functions
SX1280 * Wireless_Init(float channel, SPI_HandleTypeDef * WirelessSpi, uint8_t mode); // mode=0 -> TX, mode=1 -> RX
void SendPacket(SX1280* SX, uint8_t * data, uint8_t Nbytes);
void ReceivePacket(SX1280* SX);
void Wireless_IRQ_Handler(SX1280* SX, uint8_t * data, uint8_t Nbytes);
void Wireless_DMA_Handler(SX1280* SX);

#endif /* WIRELESS_WIRELESS_H_ */
