#include <stdio.h>
#include <string.h>

#include "apps_common.h"
#include "apps_utilities.h"
#include "lr11xx_radio.h"
#include "lr11xx_regmem.h"
#include "lr11xx_system.h"
#include "main.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_dbg_trace.h"
#include "uart_init.h"
#include "stm32l4xx_ll_utils.h"

#define IRQ_MASK                                                                                               \
    ( LR11XX_SYSTEM_IRQ_TX_DONE | LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_TIMEOUT |                      \
      LR11XX_SYSTEM_IRQ_PREAMBLE_DETECTED | LR11XX_SYSTEM_IRQ_HEADER_ERROR | LR11XX_SYSTEM_IRQ_FSK_LEN_ERROR | \
      LR11XX_SYSTEM_IRQ_CRC_ERROR )


static lr11xx_hal_context_t* context;

static uint8_t buffer[PAYLOAD_LENGTH];

static uint32_t rx_timeout = RX_TIMEOUT_VALUE;

// UART receive buffer
static uint8_t uart_rx_buffer[PAYLOAD_LENGTH];
static uint8_t uart_rx_index = 0;
static bool uart_data_ready = false;

static void per_reception_failure_handling( uint16_t* failure_counter );
static void uart_rx_callback(uint8_t data);
static void send_data_to_mos(uint8_t* data, uint8_t length);
static void send_string_to_mos(const char* str);

int main( void )
{
    smtc_hal_mcu_init( );
    apps_common_shield_init( );
    
    // Initialize UART and set receive callback function
    uart_init_with_rx_callback(uart_rx_callback);

    HAL_DBG_TRACE_INFO( "===== LoRa example -- work with MOS4=====\n\n");
    apps_common_print_sdk_driver_version( );

    context = apps_common_lr11xx_get_context( );

    apps_common_lr11xx_system_init( ( void* ) context );
    apps_common_lr11xx_fetch_and_print_version( ( void* ) context );
    apps_common_lr11xx_radio_init( ( void* ) context );

    ASSERT_LR11XX_RC( lr11xx_system_set_dio_irq_params( context, IRQ_MASK, 0 ) );
    ASSERT_LR11XX_RC( lr11xx_system_clear_irq_status( context, LR11XX_SYSTEM_IRQ_ALL_MASK ) );

    // Initialize transmit buffer
    for( int i = 1; i < PAYLOAD_LENGTH; i++ )
    {
        buffer[i] = i;
    }
    
    HAL_DBG_TRACE_INFO("Waiting for UART data...\n");
    
    while (1)
    {
        // Process interrupts
        apps_common_lr11xx_irq_process( context, IRQ_MASK );
        
        // Check if there is UART data to send
        if (uart_data_ready)
        {
            HAL_DBG_TRACE_INFO("UART data received, preparing to send...\n");
            
            send_string_to_mos(uart_rx_buffer);
            // Reset UART receive status
            uart_rx_index = 0;
            uart_data_ready = false;
        }
    }
}

static void uart_rx_callback(uint8_t data)
{
    // Store received data
    if (uart_rx_index < PAYLOAD_LENGTH)
    {
        uart_rx_buffer[uart_rx_index++] = data;
        
        // If carriage return or buffer full, mark data ready
        if (data == '\r' || data == '\n' || uart_rx_index >= PAYLOAD_LENGTH)
        {
            uart_data_ready = true;
        }
    }
    else
    {
        // Buffer full, mark data ready
        uart_data_ready = true;
    }
}

static void send_string_to_mos(const char* str)
{
    uint8_t length = strlen(str);
    
    if (length > PAYLOAD_LENGTH)
    {
        length = PAYLOAD_LENGTH;
    }

    memcpy(buffer, str, length);

    HAL_DBG_TRACE_INFO("Sending string: ");
    for (int i = 0; i < length; i++)
    {
        HAL_DBG_TRACE_INFO("%c", buffer[i]);
    }
    HAL_DBG_TRACE_INFO("\n");

    if (length < PAYLOAD_LENGTH)
    {
        memset(&buffer[length], 0, PAYLOAD_LENGTH - length);
    }

    ASSERT_LR11XX_RC( lr11xx_regmem_write_buffer8( context, buffer, PAYLOAD_LENGTH ) );

    apps_common_lr11xx_handle_pre_tx();
    ASSERT_LR11XX_RC( lr11xx_radio_set_tx(context, 0) );
}

void on_tx_done( void )
{
    //apps_common_lr11xx_handle_post_tx( );

    HAL_DBG_TRACE_INFO( "TX completed\n" );
    //LL_mDelay( 200 );
    apps_common_lr11xx_handle_pre_rx( );
    ASSERT_LR11XX_RC( lr11xx_radio_set_rx( context, rx_timeout ) );
}

void on_rx_done( void )
{
    uint8_t size;

    apps_common_lr11xx_handle_post_rx( );

    apps_common_lr11xx_receive( context, buffer, PAYLOAD_LENGTH, &size );

    // 打印接收到的内容
    HAL_DBG_TRACE_INFO("Receive success!\n");
    HAL_DBG_TRACE_INFO("[SX1278] Data:\t\t");

    for (uint8_t i = 0; i < size; i++) {
        HAL_DBG_TRACE_PRINTF("%c", buffer[i]);  // 打印字符
    }
    HAL_DBG_TRACE_INFO("\n");
    
    //apps_common_lr11xx_handle_pre_rx( );
    //ASSERT_LR11XX_RC( lr11xx_radio_set_rx( context, rx_timeout ) );
    apps_common_lr11xx_handle_post_tx( );
    HAL_DBG_TRACE_INFO("Waiting for UART data...\n");
}

void on_rx_timeout( void )
{
    apps_common_lr11xx_handle_pre_rx( );
    ASSERT_LR11XX_RC( lr11xx_radio_set_rx( context, rx_timeout ) );
}

void on_rx_crc_error( void )
{
    HAL_DBG_TRACE_INFO("on_rx_crc_error\n");
}

void on_fsk_len_error( void )
{
    HAL_DBG_TRACE_INFO("on_fsk_len_error\n");
}
