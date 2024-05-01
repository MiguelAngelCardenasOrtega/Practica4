/*
 * lin_driver_test_main.c
 * Created on: Sep 15, 2018
 *     Author: Nico
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "fsl_uart_freertos.h"
#include "fsl_uart.h"

#include "pin_mux.h"
#include "clock_config.h"
#include <lin1d3_driver.h>
#include "FreeRTOSConfig.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
//#define DEVICE_IS_MASTER
//#define DEVICE_IS_SLAVE_A
#define DEVICE_IS_SLAVE_B
//#define DEVICE_IS_SLAVE_C
//#define DEVICE_IS_SLAVE_D

/* UART instance and clock */
#define MASTER_UART UART3
#define MASTER_UART_CLKSRC UART3_CLK_SRC
#define MASTER_UART_CLK_FREQ CLOCK_GetFreq(UART3_CLK_SRC)
#define MASTER_UART_RX_TX_IRQn UART3_RX_TX_IRQn

/* UART instance and clock */
#define SLAVE_UART UART3
#define SLAVE_UART_CLKSRC UART3_CLK_SRC
#define SLAVE_UART_CLK_FREQ CLOCK_GetFreq(UART3_CLK_SRC)
#define SLAVE_UART_RX_TX_IRQn UART3_RX_TX_IRQn

/* Task priorities. */
#define init_task_PRIORITY (configMAX_PRIORITIES - 2)
#define test_task_heap_size_d	(192)

#define app_message_id_1_d (message_size_2_bytes_d<<4|0x01)
#define app_message_id_2_d (message_size_2_bytes_d<<4|0x02)
#define app_message_id_3_d (message_size_2_bytes_d<<4|0x03)
#define app_message_id_4_d (message_size_2_bytes_d<<4|0x04)



/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void test_task(void *pvParameters);

static void	message_1_transmit_callback(void* message);
static void	message_1_receive_callback(void* message);
static void	message_2_transmit_callback(void* message);
static void	message_2_receive_callback(void* message);
static void	message_3_transmit_callback(void* message);
static void	message_3_receive_callback(void* message);
static void	message_4_transmit_callback(void* message);
static void	message_4_receive_callback(void* message);
/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Application entry point.
 */
int main(void)
{
	/* Define the init structure for the input switch pin */
	gpio_pin_config_t sw_config = {
		kGPIO_DigitalInput,
		0,
	};

	/* Define the init structure for the output LED pin */
	gpio_pin_config_t led_config = {
		kGPIO_DigitalOutput,
		1,
	};

    /* Init board hardware. */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    NVIC_SetPriority(MASTER_UART_RX_TX_IRQn, 5);
    NVIC_SetPriority(SLAVE_UART_RX_TX_IRQn, 5);

    /* Init input SW2 GPIO. */
	GPIO_PinInit(BOARD_SW2_GPIO, BOARD_SW2_GPIO_PIN, &sw_config);

	/* Init input SW2 GPIO. */
	GPIO_PinInit(BOARD_SW3_GPIO, BOARD_SW3_GPIO_PIN, &sw_config);

	/* Init output RED LED GPIO. */
	GPIO_PinInit(BOARD_LED_RED_GPIO, BOARD_LED_RED_GPIO_PIN, &led_config);

	/* Init output BLUE LED GPIO. */
	GPIO_PinInit(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_GPIO_PIN, &led_config);

	/* Init output GREEN LED GPIO. */
	GPIO_PinInit(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_GPIO_PIN, &led_config);


    if (xTaskCreate(test_task, "test_task", test_task_heap_size_d, NULL, init_task_PRIORITY, NULL) != pdPASS)
    {
        PRINTF("Init Task creation failed!.\r\n");
        while (1)
            ;
    }
    PRINTF(" *** LIN driver demo v1.40***\r\n");
    vTaskStartScheduler();
    for (;;)
        ;
}

/*!
 * @brief Task responsible for loopback.
 */
static void test_task(void *pvParameters)
{
	int error;
	int msg_count = 0;
	lin1d3_nodeConfig_t node_config;
	lin1d3_handle_t* master_handle;
	lin1d3_handle_t* slave_handle;

#ifdef DEVICE_IS_MASTER
	/* Set Master Config */
	node_config.type = lin1d3_master_nodeType;
	node_config.bitrate = 9600;
	node_config.uartBase = MASTER_UART;
	node_config.srcclk = MASTER_UART_CLK_FREQ;
	node_config.skip_uart_init = 0;
	memset(node_config.messageTable,0, (sizeof(node_config.messageTable[0])*lin1d3_max_supported_messages_per_node_cfg_d));
	/* Init Master node */
	master_handle = lin1d3_InitNode(node_config);
#endif

	/* Set Slave Config */
	node_config.type = lin1d3_slave_nodeType;
	node_config.bitrate = 9600;
	node_config.uartBase = SLAVE_UART;
	node_config.srcclk = SLAVE_UART_CLK_FREQ;
#ifdef DEVICE_IS_MASTER
	node_config.skip_uart_init = 1;
	node_config.uart_rtos_handle = master_handle->uart_rtos_handle;
#else
	node_config.skip_uart_init = 0;
#endif
	memset(node_config.messageTable,0, (sizeof(node_config.messageTable[0])*lin1d3_max_supported_messages_per_node_cfg_d));
#ifdef DEVICE_IS_SLAVE_A
	node_config.messageTable[0].ID = app_message_id_1_d;
	node_config.messageTable[0].rx = 0;
	node_config.messageTable[0].handler = message_1_transmit_callback;
	node_config.messageTable[1].ID = app_message_id_2_d;
	node_config.messageTable[1].rx = 1;
	node_config.messageTable[1].handler = message_2_receive_callback;
	node_config.messageTable[2].ID = app_message_id_3_d;
	node_config.messageTable[2].rx = 1;
	node_config.messageTable[2].handler = message_3_receive_callback;
	node_config.messageTable[3].ID = app_message_id_4_d;
	node_config.messageTable[3].rx = 1;
	node_config.messageTable[3].handler = message_4_receive_callback;
#endif
#ifdef DEVICE_IS_SLAVE_B
	node_config.messageTable[0].ID = app_message_id_1_d;
	node_config.messageTable[0].rx = 1;
	node_config.messageTable[0].handler = message_1_receive_callback;
	node_config.messageTable[1].ID = app_message_id_2_d;
	node_config.messageTable[1].rx = 0;
	node_config.messageTable[1].handler = message_2_transmit_callback;
#endif
#ifdef DEVICE_IS_SLAVE_C
	node_config.messageTable[0].ID = app_message_id_1_d;
	node_config.messageTable[0].rx = 1;
	node_config.messageTable[0].handler = message_1_receive_callback;
	node_config.messageTable[1].ID = app_message_id_3_d;
	node_config.messageTable[1].rx = 0;
	node_config.messageTable[1].handler = message_3_transmit_callback;
#endif
#ifdef DEVICE_IS_SLAVE_D
	node_config.messageTable[0].ID = app_message_id_1_d;
	node_config.messageTable[0].rx = 1;
	node_config.messageTable[0].handler = message_1_receive_callback;
	node_config.messageTable[1].ID = app_message_id_4_d;
	node_config.messageTable[1].rx = 0;
	node_config.messageTable[1].handler = message_4_transmit_callback;
#endif
	/* Init Slave Node*/
	slave_handle = lin1d3_InitNode(node_config);


	if((NULL == slave_handle)
#ifdef DEVICE_IS_MASTER
		|| (NULL == master_handle)
#endif
	   ){
		PRINTF(" Init failed!! \r\n");
		error = kStatus_Fail;
	}
	else {
		error = kStatus_Success;
	}

	while (kStatus_Success == error)
    {
#ifdef DEVICE_IS_MASTER
		vTaskDelay(pdMS_TO_TICKS(25));
    	lin1d3_masterSendMessage(master_handle, app_message_id_2_d);
    	vTaskDelay(pdMS_TO_TICKS(25));
		//lin1d3_masterSendMessage(master_handle, app_message_id_3_d);
		vTaskDelay(pdMS_TO_TICKS(25));
		//lin1d3_masterSendMessage(master_handle, app_message_id_4_d);
		vTaskDelay(pdMS_TO_TICKS(25));
		msg_count++;
		if (msg_count == 10)
		{
			msg_count = 0;
			lin1d3_masterSendMessage(master_handle, app_message_id_1_d);
		}
#else
    	vTaskDelay(pdMS_TO_TICKS(1000));
#endif
    }

    vTaskSuspend(NULL);
}

static void	message_1_transmit_callback(void* message)
{
	static uint8_t count = 0;
	uint8_t* message_data = (uint8_t*)message;

	PRINTF("Slave got message 1 transmit request\r\n");

	message_data[0] = count;
	count++;
	count %= 4;
}

static void	message_1_receive_callback(void* message)
{
	uint8_t* message_data = (uint8_t*)message;

	PRINTF("Slave received message 1\r\n");

	if (message_data[0] == 0)
	{
		// CLEAR ALL RGB LEDs
		GPIO_PortSet(BOARD_LED_RED_GPIO, 1U << BOARD_LED_RED_GPIO_PIN);
		GPIO_PortSet(BOARD_LED_GREEN_GPIO, 1U << BOARD_LED_GREEN_GPIO_PIN);
		GPIO_PortSet(BOARD_LED_BLUE_GPIO, 1U << BOARD_LED_BLUE_GPIO_PIN);
	}
	else if (message_data[0] == 1)
	{
		// CLEAR ALL G & B LEDs
		GPIO_PortSet(BOARD_LED_GREEN_GPIO, 1U << BOARD_LED_GREEN_GPIO_PIN);
		GPIO_PortSet(BOARD_LED_BLUE_GPIO, 1U << BOARD_LED_BLUE_GPIO_PIN);

		//TURN ON RED LED
		GPIO_PortClear(BOARD_LED_RED_GPIO, 1U << BOARD_LED_RED_GPIO_PIN);
	}
	else if (message_data[0] == 2)
	{
		// CLEAR ALL R & B LEDs
		GPIO_PortSet(BOARD_LED_RED_GPIO, 1U << BOARD_LED_RED_GPIO_PIN);
		GPIO_PortSet(BOARD_LED_BLUE_GPIO, 1U << BOARD_LED_BLUE_GPIO_PIN);

		//TURN ON GREEN LED
		GPIO_PortClear(BOARD_LED_GREEN_GPIO, 1U << BOARD_LED_GREEN_GPIO_PIN);
	}
	else if (message_data[0] == 3)
	{
		// CLEAR ALL R & G LEDs
		GPIO_PortSet(BOARD_LED_RED_GPIO, 1U << BOARD_LED_RED_GPIO_PIN);
		GPIO_PortSet(BOARD_LED_GREEN_GPIO, 1U << BOARD_LED_GREEN_GPIO_PIN);

		//TURN ON RED LED
		GPIO_PortClear(BOARD_LED_BLUE_GPIO, 1U << BOARD_LED_BLUE_GPIO_PIN);
	}
	else
	{
		// CLEAR ALL RGB LEDs
		GPIO_PortSet(BOARD_LED_RED_GPIO, 1U << BOARD_LED_RED_GPIO_PIN);
		GPIO_PortSet(BOARD_LED_GREEN_GPIO, 1U << BOARD_LED_GREEN_GPIO_PIN);
		GPIO_PortSet(BOARD_LED_BLUE_GPIO, 1U << BOARD_LED_BLUE_GPIO_PIN);
		PRINTF("INVALID MSG 1 LED COLOR!\r\n");
	}
}

static void	message_2_transmit_callback(void* message)
{
	uint8_t* message_data = (uint8_t*)message;

	PRINTF("Slave got message 2 transmit request\r\n");

	if (GPIO_PinRead(BOARD_SW2_GPIO, BOARD_SW2_GPIO_PIN) == 0)
	{
		message_data[0] |= 0x1;
	}

	if (GPIO_PinRead(BOARD_SW3_GPIO, BOARD_SW3_GPIO_PIN) == 0)
	{
		message_data[0] |= 0x2;
	}
}

static void	message_2_receive_callback(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Slave received message 2\r\n");

	if (message_data[0] != 0)
	{
		//TURN ON RED LED
		GPIO_PortClear(BOARD_LED_RED_GPIO, 1U << BOARD_LED_RED_GPIO_PIN);
	}
	else
	{
		//TURN OFF RED LED
		GPIO_PortSet(BOARD_LED_RED_GPIO, 1U << BOARD_LED_RED_GPIO_PIN);
	}
}

static void	message_3_transmit_callback(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Slave got message 3 transmit request\r\n");

	if (GPIO_PinRead(BOARD_SW2_GPIO, BOARD_SW2_GPIO_PIN) == 0)
	{
		message_data[0] |= 0x1;
	}

	if (GPIO_PinRead(BOARD_SW3_GPIO, BOARD_SW3_GPIO_PIN) == 0)
	{
		message_data[0] |= 0x2;
	}
}

static void	message_3_receive_callback(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Slave received message 3\r\n");

	if (message_data[0] != 0)
	{
		//TURN ON GREEN LED
		GPIO_PortClear(BOARD_LED_GREEN_GPIO, 1U << BOARD_LED_GREEN_GPIO_PIN);
	}
	else
	{
		//TURN OFF GREEN LED
		GPIO_PortSet(BOARD_LED_GREEN_GPIO, 1U << BOARD_LED_GREEN_GPIO_PIN);
	}
}

static void	message_4_transmit_callback(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Slave got message 4 transmit request\r\n");

	if (GPIO_PinRead(BOARD_SW2_GPIO, BOARD_SW2_GPIO_PIN) == 0)
	{
		message_data[0] |= 0x1;
	}

	if (GPIO_PinRead(BOARD_SW3_GPIO, BOARD_SW3_GPIO_PIN) == 0)
	{
		message_data[0] |= 0x2;
	}
}

static void	message_4_receive_callback(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Slave received message 4\r\n");

	if (message_data[0] != 0)
	{
		//TURN ON BLUE LED
		GPIO_PortClear(BOARD_LED_BLUE_GPIO, 1U << BOARD_LED_BLUE_GPIO_PIN);
	}
	else
	{
		//TURN OFF BLUE LED
		GPIO_PortSet(BOARD_LED_BLUE_GPIO, 1U << BOARD_LED_BLUE_GPIO_PIN);
	}
}



