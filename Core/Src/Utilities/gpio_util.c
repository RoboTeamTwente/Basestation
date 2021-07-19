/*
 * gpio_util.c
 *
 *  Created on: 26 okt. 2018
 *      Author: Cas Doornkamp
 */

#include "gpio_util.h"
// List known GPIO pins

GPIO_Pin SX_TX_IRQ 			= { SX_TX_IRQ_GPIO_Port			, SX_TX_IRQ_Pin			};
GPIO_Pin SX_TX_RST			= { SX_TX_RST_GPIO_Port			, SX_TX_RST_Pin			};
GPIO_Pin SX_TX_BUSY 		= { SX_TX_BUSY_GPIO_Port		, SX_TX_BUSY_Pin		};
GPIO_Pin SX_TX_CS 			= { SX_TX_CS_GPIO_Port			, SX_TX_CS_Pin			};

GPIO_Pin SX_RX_IRQ 			= { SX_RX_IRQ_GPIO_Port			, SX_RX_IRQ_Pin			};
GPIO_Pin SX_RX_RST			= { SX_RX_RST_GPIO_Port			, SX_RX_RST_Pin			};
GPIO_Pin SX_RX_BUSY 		= { SX_RX_BUSY_GPIO_Port		, SX_RX_BUSY_Pin		};
GPIO_Pin SX_RX_CS 			= { SX_RX_CS_GPIO_Port			, SX_RX_CS_Pin			};

GPIO_Pin LD_ACTIVE 			= { LED_ACTIVE_GPIO_Port		, LED_ACTIVE_Pin		};
GPIO_Pin LD_USB 			= { LED_USB_GPIO_Port			, LED_USB_Pin			};
GPIO_Pin LD_LED1 			= { LED_LED1_GPIO_Port			, LED_LED1_Pin			};
GPIO_Pin LD_LED2 			= { LED_LED2_GPIO_Port			, LED_LED2_Pin			};
GPIO_Pin LD_LED3 			= { LED_LED3_GPIO_Port			, LED_LED3_Pin			};
GPIO_Pin LD_TX 				= { LED_TX_GPIO_Port			, LED_TX_Pin			};
GPIO_Pin LD_RX 				= { LED_RX_GPIO_Port			, LED_RX_Pin			};

GPIO_Pin USB_RST            = { USB_RST_GPIO_Port           , USB_RST_Pin           };

GPIO_Pin SW1                = { SW1_GPIO_Port               , SW1_Pin               };
GPIO_Pin SW2                = { SW2_GPIO_Port               , SW2_Pin               };
GPIO_Pin SW3                = { SW3_GPIO_Port               , SW3_Pin               };
GPIO_Pin SW4                = { SW4_GPIO_Port               , SW4_Pin               };