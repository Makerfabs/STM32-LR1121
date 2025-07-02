#include <stdio.h>
#include <string.h>

#include "apps_common.h"
#include "apps_utilities.h"
#include "lr11xx_radio.h"
#include "lr11xx_regmem.h"
#include "lr11xx_system.h"
#include "main_per.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_dbg_trace.h"
#include "uart_init.h"
#include "stm32l4xx_ll_utils.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief LR11xx interrupt mask used by the application
 */
#define IRQ_MASK                                                                                               \
    ( LR11XX_SYSTEM_IRQ_TX_DONE | LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_TIMEOUT |                      \
      LR11XX_SYSTEM_IRQ_PREAMBLE_DETECTED | LR11XX_SYSTEM_IRQ_HEADER_ERROR | LR11XX_SYSTEM_IRQ_FSK_LEN_ERROR | \
      LR11XX_SYSTEM_IRQ_CRC_ERROR )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#if( RECEIVER == 1 )
const char* mode = "Receiver";
#else
const char* mode = "Transmitter";
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static lr11xx_hal_context_t* context;

static uint8_t buffer[PAYLOAD_LENGTH];

static uint16_t nb_ok            = 0;
static uint16_t nb_rx_timeout    = 0;
static uint16_t nb_rx_error      = 0;
static uint16_t nb_fsk_len_error = 0;

static uint8_t rolling_counter = 0;

static uint16_t per_index      = 0;
static bool     first_pkt_flag = false;

static uint8_t  per_msg[PAYLOAD_LENGTH];
static uint32_t rx_timeout = RX_TIMEOUT_VALUE;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Handle reception failure for PER example
 *
 * @param [in] failure_counter pointer to the counter for each type of reception failure
 */
static void per_reception_failure_handling( uint16_t* failure_counter );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * @brief Main application entry point.
 */
int main( void )
{
    smtc_hal_mcu_init( );
    apps_common_shield_init( );
    uart_init();

    HAL_DBG_TRACE_INFO( "===== LoRa example - %s =====\n\n", mode );
    apps_common_print_sdk_driver_version( );

    context = apps_common_lr11xx_get_context( );

    apps_common_lr11xx_system_init( ( void* ) context );
    apps_common_lr11xx_fetch_and_print_version( ( void* ) context );
    apps_common_lr11xx_radio_init( ( void* ) context );

    ASSERT_LR11XX_RC( lr11xx_system_set_dio_irq_params( context, IRQ_MASK, 0 ) );
    ASSERT_LR11XX_RC( lr11xx_system_clear_irq_status( context, LR11XX_SYSTEM_IRQ_ALL_MASK ) );

    for( int i = 1; i < PAYLOAD_LENGTH; i++ )
    {
        buffer[i] = i;
    }
    rx_timeout += get_time_on_air_in_ms( );
#if RECEIVER == 1
    apps_common_lr11xx_handle_pre_rx( );
    ASSERT_LR11XX_RC( lr11xx_radio_set_rx( context, rx_timeout ) );
    memcpy( per_msg, &buffer[1], PAYLOAD_LENGTH - 1 );
#else
    buffer[0] = 0;
    ASSERT_LR11XX_RC( lr11xx_regmem_write_buffer8( context, buffer, PAYLOAD_LENGTH ) );
    apps_common_lr11xx_handle_pre_tx( );
    ASSERT_LR11XX_RC( lr11xx_radio_set_tx( context, 0 ) );
#endif

    //while( per_index < NB_FRAME )
		while (1)
    {
        apps_common_lr11xx_irq_process( context, IRQ_MASK );
    }

    /*if( per_index > NB_FRAME )
    {
        nb_ok--;
    }
    HAL_DBG_TRACE_PRINTF( "PER = %d \n", 100 - ( ( nb_ok * 100 ) / NB_FRAME ) );

    HAL_DBG_TRACE_PRINTF( "Final PER index: %d \n", per_index );
    HAL_DBG_TRACE_PRINTF( "Valid reception amount: %d \n", nb_ok );
    HAL_DBG_TRACE_PRINTF( "Timeout reception amount: %d \n", nb_rx_timeout );
    HAL_DBG_TRACE_PRINTF( "CRC Error reception amount: %d \n", nb_rx_error );
#if( PACKET_TYPE == LR11XX_RADIO_PKT_TYPE_GFSK )
    HAL_DBG_TRACE_PRINTF( "FSK Length Error reception amount: %d \n", nb_fsk_len_error );
#endif

    while( 1 )
    {
    }*/
}

/*!
 * @brief TX done interrupt handler
 */
void on_tx_done( void )
{
    apps_common_lr11xx_handle_post_tx( );

    LL_mDelay( TX_TO_TX_DELAY_IN_MS );

    buffer[0]++;
    HAL_DBG_TRACE_INFO( "Counter value: %d\n", buffer[0] );
    ASSERT_LR11XX_RC( lr11xx_regmem_write_buffer8( context, buffer, PAYLOAD_LENGTH ) );

    apps_common_lr11xx_handle_pre_tx( );
    ASSERT_LR11XX_RC( lr11xx_radio_set_tx( context, 0 ) );
}

/*!
 * @brief RX done interrupt handler
 */
void on_rx_done( void )
{
    uint8_t size;

    apps_common_lr11xx_handle_post_rx( );

    apps_common_lr11xx_receive( context, buffer, PAYLOAD_LENGTH, &size );

    if( memcmp( &buffer[1], per_msg, PAYLOAD_LENGTH - 1 ) == 0 )
    {
        if( first_pkt_flag == true )
        {
            uint8_t rolling_counter_gap = ( uint8_t ) ( buffer[0] - rolling_counter );
            nb_ok++;
            per_index += rolling_counter_gap;
            if( rolling_counter_gap > 1 )
            {
                HAL_DBG_TRACE_WARNING( "%d packet(s) missed\n", ( rolling_counter_gap - 1 ) );
            }
            rolling_counter = buffer[0];
        }
        else
        {
            first_pkt_flag  = true;
            rolling_counter = buffer[0];
        }
        HAL_DBG_TRACE_INFO( "Counter value: %d, PER index: %d\n", buffer[0], per_index );
    }
    //if( per_index < NB_FRAME )  // Re-start Rx only if the expected number of frames is not reached
    //{
        apps_common_lr11xx_handle_pre_rx( );
        ASSERT_LR11XX_RC( lr11xx_radio_set_rx( context, rx_timeout ) );
    //}
}

/*!
 * @brief RX timeout interrupt handler
 */
void on_rx_timeout( void )
{
    per_reception_failure_handling( &nb_rx_timeout );
}

/*!
 * @brief CRC error interrupt handler
 */
void on_rx_crc_error( void )
{
    per_reception_failure_handling( &nb_rx_error );
}

/*!
 * @brief FSK length error interrupt handler
 */
void on_fsk_len_error( void )
{
    per_reception_failure_handling( &nb_fsk_len_error );
}

/*!
 * @brief Reception failure handling function
 * @param failure_counter Pointer to the specific failure type counter
 */
static void per_reception_failure_handling( uint16_t* failure_counter )
{
    apps_common_lr11xx_handle_post_rx( );

    // Let's start counting after the first received packet
    if( first_pkt_flag == true )
    {
        ( *failure_counter )++;
    }

    apps_common_lr11xx_handle_pre_rx( );
    ASSERT_LR11XX_RC( lr11xx_radio_set_rx( context, rx_timeout ) );
}