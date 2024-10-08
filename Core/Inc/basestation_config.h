#include "basestation.h"
#include "main.h"
#include "Wireless.h"
#include "logging.h"
#include "packet_buffers.h"
#include "FT812Q_Drawing.h"

#include "REM_BaseTypes.h"
#include "REM_BasestationConfiguration.h"
#include "REM_RobotCommand.h"
#include "REM_RobotFeedback.h"
#include "REM_RobotStateInfo.h"
#include "REM_RobotSetPIDGains.h"
#include "REM_RobotMusicCommand.h"
#include "REM_Packet.h"
#include "REM_SX1280Filler.h"
#include "REM_RobotKillCommand.h"

#include "CircularBuffer.h"
#include <usbd_def.h>
extern USBD_HandleTypeDef hUsbDeviceFS;


/* Counters, tracking the number of packets handled */
volatile uint32_t packet_counter_in[REM_TOTAL_NUMBER_OF_PACKETS];
volatile uint32_t packet_counter_out[REM_TOTAL_NUMBER_OF_PACKETS]; 

/* Import hardware handles from main.c */
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim1;

/* Screen variables */
DISPLAY_STATES displayState = DISPLAY_STATE_DEINITIALIZED;
uint16_t touchPoint[2] = {-1, -1}; // Initialize touchPoint outside of screen, meaning TOUCH_STATE_RELEASED
TouchState touchState; // TODO check default initialization. What is touchState->state? Compiler dependent?

// Based on Wireless.c:SX1280_Settings.TXoffset
// Currently, we're splitting the SX1280 256 byte buffer in half. 128 for sending, 128 for receiving
// Set to 127, because that's the max value as defined in the datasheet
// Table 14-38: Payload Length Definition in FLRC Packet, page 124

/* SX data */
extern SX1280_Settings SX1280_DEFAULT_SETTINGS;
extern SX1280_Settings SX1280_RX_SETTINGS;
extern SX1280_Settings SX1280_TX_SETTINGS;

static Wireless SX1280_TX = {0};
static Wireless SX1280_RX = {0};
static Wireless* SX_TX = &SX1280_TX;
static Wireless* SX_RX = &SX1280_RX;
static uint8_t SXTX_TX_buffer[MAX_PAYLOAD_SIZE + 3] __attribute__((aligned(4))) = {0};
static uint8_t SXTX_RX_buffer[MAX_PAYLOAD_SIZE + 3] __attribute__((aligned(4))) = {0};
static uint8_t SXRX_TX_buffer[MAX_PAYLOAD_SIZE + 3] __attribute__((aligned(4))) = {0};
static uint8_t SXRX_RX_buffer[MAX_PAYLOAD_SIZE + 3] __attribute__((aligned(4))) = {0};

static Wireless_Packet txPacket;
static Wireless_Packet rxPacket;

// The pins cannot be set at this point as they are not "const" enough for the compiler, so set them in the init
SX1280_Interface SX_TX_Interface = {.SPI= &hspi1, .TXbuf= SXTX_TX_buffer, .RXbuf= SXTX_RX_buffer, /*.logger=LOG_printf*/};
SX1280_Interface SX_RX_Interface = {.SPI= &hspi2, .TXbuf= SXRX_TX_buffer, .RXbuf= SXRX_RX_buffer, /*.logger=LOG_printf*/};