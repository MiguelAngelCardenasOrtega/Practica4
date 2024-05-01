/*
 * lin1d3_driver.c
 *
 *  Created on: Sep 14, 2018
 *      Author: Nico
 */
#include "lin1d3_driver.h"
#include <string.h>
#include <fsl_debug_console.h>

#define master_stack_size_d	(256)
#define master_task_priority (configMAX_PRIORITIES - 1)
#define master_queue_size_d	(8)

#define slave_stack_size_d	(256)
#define slave_task_priority (configMAX_PRIORITIES - 1)

#define size_of_uart_buffer	(10)

#define size_of_lin_header_d (2)

#define ID0_BIT_MASK		 (0x01)
#define ID1_BIT_MASK		 (0x02)
#define ID2_BIT_MASK		 (0x04)
#define ID3_BIT_MASK		 (0x08)
#define ID4_BIT_MASK		 (0x10)
#define ID5_BIT_MASK		 (0x20)
#define EVEN_PARITY_BIT_MASK (0x40)
#define ODD_PARITY_BIT_MASK  (0x80)

/*Static function prototypes */
static void master_task(void *pvParameters);
static void slave_task(void *pvParameters);
static void calculate_parity_bits(uint8_t* header);
static bool check_parity_bits(uint8_t* header);
static void calculate_checksum(uint8_t* message, uint8_t message_size);
static bool check_checksum(uint8_t* message, uint8_t message_size);


/******************************************************************************
 * Public functions
 *
 *****************************************************************************/

/*
 * Init a LIN node
 * */
lin1d3_handle_t* lin1d3_InitNode(lin1d3_nodeConfig_t config)
{
	lin1d3_handle_t* handle = NULL;
	static uint8_t node_idx = 0;
	char master_task_name[] = "linMaster0";
	char slave_task_name[] = "linSlave0";
	/* Check init parameters */
	if(config.type >= lin1d3_max_nodeType) {
		return NULL;
	}

	/* Create the handle structure and */
	handle = (lin1d3_handle_t*)pvPortMalloc(sizeof(lin1d3_handle_t));
	if(handle ==  NULL) {
		/* Failed to allocate memory for the node handle */
		return NULL;
	}
	/* Init the handle structure with 0s */
	memset(handle, 0, sizeof(lin1d3_handle_t));
	/* Copy the config */
	memcpy(&(handle->config), &config, sizeof(lin1d3_nodeConfig_t));

	/* Init/Configure the UART */
	handle->uart_config.base = handle->config.uartBase;
	handle->uart_config.srcclk = handle->config.srcclk;
	handle->uart_config.baudrate = handle->config.bitrate;
	handle->uart_config.parity = kUART_ParityDisabled;
	handle->uart_config.stopbits = kUART_OneStopBit;
	handle->uart_config.buffer = pvPortMalloc(size_of_uart_buffer);
	handle->uart_config.buffer_size = size_of_uart_buffer;
	if(handle->uart_config.buffer == NULL){
		return NULL;
	}

	if(config.skip_uart_init == 0) {
		/* Create the handle UART handle structures */
		handle->uart_rtos_handle = (uart_rtos_handle_t*)pvPortMalloc(sizeof(uart_rtos_handle_t));
		if(handle->uart_rtos_handle ==  NULL) {
			/* Failed to allocate memory for the node handle */
			return NULL;
		}

		handle->uart_handle = (uart_handle_t*)pvPortMalloc(sizeof(uart_handle_t));
		if(handle->uart_handle ==  NULL) {
			/* Failed to allocate memory for the node handle */
			return NULL;
		}

		if (0 > UART_RTOS_Init(handle->uart_rtos_handle, handle->uart_handle, &(handle->uart_config)))
		{
			return NULL;
		}
	}
	else {
		handle->uart_rtos_handle = config.uart_rtos_handle;
	}
	/* Create the Node Task */
	if(lin1d3_master_nodeType == config.type) {
		/* Create a queue for User message requests */
		handle->node_queue = xQueueCreate( master_queue_size_d, sizeof(uint8_t));
		if(handle->node_queue == NULL){
			vPortFree(handle);
			return NULL;
		}
		/* Create a task for the node */
		master_task_name[strlen(master_task_name)-1] += node_idx++;
		if (xTaskCreate(master_task, master_task_name, master_stack_size_d, handle, master_task_priority, &(handle->task_handle)) != pdPASS) {
			vPortFree(handle);
			return NULL;
		}
	}
	else if(lin1d3_slave_nodeType == config.type) {
		/* Create a task for the node */
		slave_task_name[strlen(slave_task_name)-1] += node_idx++;
		if (xTaskCreate(slave_task, slave_task_name, slave_stack_size_d, handle, slave_task_priority, &(handle->task_handle)) != pdPASS) {
			vPortFree(handle);
			return NULL;
		}
	}

	return handle;
}

/*
 * Send a message frame from a LIN Master node
 * */
uint32_t lin1d3_masterSendMessage(lin1d3_handle_t* handle, uint8_t ID)
{
	if(handle !=  NULL) {
		/* Put the requested ID on the master queue */
		xQueueSend( handle->node_queue, &ID, ( TickType_t ) 0 );
	}
	return 0;
}

/******************************************************************************
 * Static functions
 *
 *****************************************************************************/
static void master_task(void *pvParameters)
{
	lin1d3_handle_t* handle = (lin1d3_handle_t*)pvParameters;
	uint8_t  ID;
	uint8_t  lin1p3_header[] = {0x55, 0x00};
	uint8_t  lin1p3_message[size_of_uart_buffer];
	uint8_t  message_size = 0;

	if(handle == NULL) {
		vTaskSuspend(NULL);
	}

    while(1) {
    	/* Wait for messages on the Queue */
        if(xQueueReceive(handle->node_queue, &ID, portMAX_DELAY)){
        	/* Build and send the LIN Header */
        	/* Put the ID into the header */
        	lin1p3_header[1] = ID;
        	// Add the parity bits
        	calculate_parity_bits(lin1p3_header);
        	/* Init the message recevie buffer */
        	memset(lin1p3_message, 0, size_of_uart_buffer);
        	/* Calc the message size */
        	switch(ID&0x30) {
        		case 0x00: message_size = 2;
        		break;
        		case 0x10: message_size = 2;
        		break;
        		case 0x20: message_size = 4;
        		break;
        		case 0x30: message_size = 8;
        		break;
        	}
        	message_size+=1;

        	/* Configure 13bit break transmission */
        	handle->uart_config.base->S2 |= (1<<2);

        	/* Send the break signal */
        	handle->uart_config.base->C2 |= 0x01;
        	handle->uart_config.base->C2 &= 0xFE;

        	vTaskDelay(1);

        	/* Send the header */
        	UART_RTOS_Send(handle->uart_rtos_handle, (uint8_t *)lin1p3_header, size_of_lin_header_d);
        	vTaskDelay(1);
        }
    }
}

static void slave_task(void *pvParameters)
{
	lin1d3_handle_t* handle = (lin1d3_handle_t*)pvParameters;
	uint8_t  ID;
	uint8_t  lin1p3_header[size_of_lin_header_d];
	uint8_t  lin1p3_message[size_of_uart_buffer];
	uint8_t  message_size = 0;
	size_t n;
	uint8_t  msg_idx;

	if(handle == NULL) {
		vTaskSuspend(NULL);
	}

    while(1) {
    	/* Init the message header buffer */
    	memset(lin1p3_header, 0, size_of_lin_header_d);

    	/* Wait for break */
		DisableIRQ(UART3_RX_TX_IRQn); //Disable RX interrupt so the break won't mess with the UART_RTOS driver
		handle->uart_config.base->S2 |= 0x01<<7; //Clear the LIN Break Detect Interrupt Flag
		handle->uart_config.base->S2 |= 0x01<<1; //Enable LIN Break Detection
		while((handle->uart_config.base->S2 &  0x01<<7) == 0x00) vTaskDelay(1); //Wait for the flag to be set
		handle->uart_config.base->S2 &= ~(0x01<<1); //Disable LIN Break Detection
		handle->uart_config.base->S2 |= 0x01<<7; //Clear the LIN Break Detect Interrupt Flag
		EnableIRQ(UART3_RX_TX_IRQn); //Enable RX interrupt so the UART_RTOS driver works again

    	/* Wait for header on the UART */
    	UART_RTOS_Receive(handle->uart_rtos_handle, lin1p3_header, size_of_lin_header_d, &n);
    	/* Check header */
    	if((lin1p3_header[0] != 0x55) && (check_parity_bits(lin1p3_header) == true)) {
    		/* Header is not correct we are ignoring the header */
    		continue;
    	}
    	/* Get the message ID */
    	ID = (lin1p3_header[1] & 0x3F);
    	/* If the header is correct, check if the message is in the table */
    	msg_idx = 0;
    	/*Look for the ID in the message table */
    	while(msg_idx < lin1d3_max_supported_messages_per_node_cfg_d) {
    		if(handle->config.messageTable[msg_idx].ID == ID) {
    			break;
    		}
    		msg_idx++;
    	}
    	/* If the message ID was not found then ignore it */
    	if(msg_idx == lin1d3_max_supported_messages_per_node_cfg_d) continue;

    	/* Calc the message size */
    	switch(ID&0x30) {
    		case 0x00: message_size = 2;
    		break;
    		case 0x10: message_size = 2;
    		break;
    		case 0x20: message_size = 4;
    		break;
    		case 0x30: message_size = 8;
    		break;
    	}

    	message_size+=1;
    	/* Init the message transmit buffer */
    	memset(lin1p3_message, 0, size_of_uart_buffer);

    	if(handle->config.messageTable[msg_idx].rx == 0) {
        	/*If the message is in the table call the message callback */
    		/* User shall fill the message */
        	handle->config.messageTable[msg_idx].handler((void*)lin1p3_message);
        	/* Add the checksum to the message */
        	calculate_checksum(lin1p3_message, message_size);
        	/* Send the message data */
        	UART_RTOS_Send(handle->uart_rtos_handle, (uint8_t *)lin1p3_message, message_size);
    	}
    	else {
        	/* Wait for Response on the UART */
        	UART_RTOS_Receive(handle->uart_rtos_handle, lin1p3_message, message_size, &n);
        	if (check_checksum(lin1p3_message, message_size) == true)
        	{
        		/*If the message is in the table call the message callback */
        		handle->config.messageTable[msg_idx].handler((void*)lin1p3_message);
        	}
    	}
    }
}

static void calculate_parity_bits(uint8_t* header)
{
	uint8_t id0_bit = (header[1] & ID0_BIT_MASK);
	uint8_t id1_bit = (header[1] & ID1_BIT_MASK) >> 1;
	uint8_t id2_bit = (header[1] & ID2_BIT_MASK) >> 2;
	uint8_t id3_bit = (header[1] & ID3_BIT_MASK) >> 3;
	uint8_t id4_bit = (header[1] & ID4_BIT_MASK) >> 4;
	uint8_t id5_bit = (header[1] & ID5_BIT_MASK) >> 5;

	uint8_t even_parity = (id0_bit ^ id1_bit) ^ (id2_bit ^ id4_bit);
	uint8_t odd_parity =  !((id1_bit ^ id3_bit) ^ (id4_bit ^ id5_bit));

	header[1] |= (even_parity << 6);
	header[1] |= (odd_parity << 7);
}

static bool check_parity_bits(uint8_t* header)
{
	uint8_t id0_bit = (header[1] & ID0_BIT_MASK);
	uint8_t id1_bit = (header[1] & ID1_BIT_MASK) >> 1;
	uint8_t id2_bit = (header[1] & ID2_BIT_MASK) >> 2;
	uint8_t id3_bit = (header[1] & ID3_BIT_MASK) >> 3;
	uint8_t id4_bit = (header[1] & ID4_BIT_MASK) >> 4;
	uint8_t id5_bit = (header[1] & ID5_BIT_MASK) >> 5;

	uint8_t even_parity = (id0_bit ^ id1_bit) ^ (id2_bit ^ id4_bit);
	uint8_t odd_parity =  !((id1_bit ^ id3_bit) ^ (id4_bit ^ id5_bit));

	if ((((header[1] & EVEN_PARITY_BIT_MASK) >> 6) == even_parity) && (((header[1] & ODD_PARITY_BIT_MASK) >> 7) == odd_parity))
	{
		return true;
	}
	else
	{
		PRINTF("Parity bits check error!\r\n");
		return false;
	}
}

static void calculate_checksum(uint8_t* message, uint8_t message_size)
{
	uint8_t checksum = 0;

	for (uint8_t idx = 0; idx < message_size-1; idx++)
	{
		checksum = (checksum + message[idx]) % 0xFF;
	}

	message[message_size-1] = (uint8_t)~checksum;
}

static bool check_checksum(uint8_t* message, uint8_t message_size)
{
	uint8_t checksum = 0;

	for (uint8_t idx = 0; idx < message_size-1; idx++)
	{
		checksum = (checksum + message[idx]) % 0xFF;
	}

	if ((checksum + message[message_size-1]) == 0xFF)
	{
		return true;
	}
	else
	{
		PRINTF("MSG: 0x%x, 0x%x, 0x%x\r\n", message[0], message[1], message[2]);
		PRINTF("Checksum check error!\r\n");
		return false;
	}
}
