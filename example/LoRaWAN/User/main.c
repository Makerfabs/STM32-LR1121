#include <stdio.h>
#include <stdint.h>
#include "lorawan_commissioning.h"
#include "lr1121_modem_board.h"
#include "apps_utilities.h"
#include "lr1121_modem_helper.h"
#include "lr1121_modem_system_types.h"
#include "dht11.h"
#include "delay.h"
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*!
 * @brief Stringify constants
 */
#define xstr( a ) str( a )
#define str( a ) #a

/*!
 * @brief Helper macro that returned a human-friendly message if a command does not return LR1121_MODEM_RESPONSE_CODE_OK
 *
 * @remark The macro is implemented to be used with functions returning a @ref lr1121_modem_return_code_t
 *
 * @param[in] rc  Return code
 */

#define ASSERT_SMTC_MODEM_RC( rc_func )                                                        \
    do                                                                                         \
    {                                                                                          \
        lr1121_modem_response_code_t rc = rc_func;                                             \
        if( rc == LR1121_MODEM_RESPONSE_CODE_NOT_INITIALIZED )                                 \
        {                                                                                      \
            HAL_DBG_TRACE_ERROR( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__,   \
                                 xstr( LR1121_MODEM_RESPONSE_CODE_NOT_INITIALIZED ) );         \
        }                                                                                      \
        else if( rc == LR1121_MODEM_RESPONSE_CODE_INVALID )                                    \
        {                                                                                      \
            HAL_DBG_TRACE_ERROR( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__,   \
                                 xstr( LR1121_MODEM_RESPONSE_CODE_INVALID ) );                 \
        }                                                                                      \
        else if( rc == LR1121_MODEM_RESPONSE_CODE_BUSY )                                       \
        {                                                                                      \
            HAL_DBG_TRACE_ERROR( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__,   \
                                 xstr( LR1121_MODEM_RESPONSE_CODE_BUSY ) );                    \
        }                                                                                      \
        else if( rc == LR1121_MODEM_RESPONSE_CODE_FAIL )                                       \
        {                                                                                      \
            HAL_DBG_TRACE_ERROR( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__,   \
                                 xstr( LR1121_MODEM_RESPONSE_CODE_FAIL ) );                    \
        }                                                                                      \
        else if( rc == LR1121_MODEM_RESPONSE_CODE_NO_TIME )                                    \
        {                                                                                      \
            HAL_DBG_TRACE_WARNING( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__, \
                                   xstr( LR1121_MODEM_RESPONSE_CODE_NO_TIME ) );               \
        }                                                                                      \
        else if( rc == LR1121_MODEM_RESPONSE_CODE_NO_EVENT )                                   \
        {                                                                                      \
            HAL_DBG_TRACE_INFO( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__,    \
                                xstr( LR1121_MODEM_RESPONSE_CODE_NO_EVENT ) );                 \
        }                                                                                      \
    } while( 0 )

/**
 * @brief Watchdog counter reload value during sleep (The period must be lower than MCU watchdog period (here 20s))
 */
#define WATCHDOG_RELOAD_PERIOD_MS 20000

/**
 * @brief Periodical uplink alarm delay in seconds
 */
#define PERIODICAL_UPLINK_DELAY_S 300

#define EXTI_BUTTON PC_13

/*!
 * @brief User application data buffer size
 */
#define LORAWAN_APP_DATA_MAX_SIZE 242

/*!
 * @brief LoRaWAN regulatory region.
 * One of:
 * LR1121_LORAWAN_REGION_AS923_GRP1
 * LR1121_LORAWAN_REGION_AS923_GRP2
 * LR1121_LORAWAN_REGION_AS923_GRP3
 * LR1121_LORAWAN_REGION_AS923_GRP4
 * LR1121_LORAWAN_REGION_AU915
 * LR1121_LORAWAN_REGION_CN470
 * LR1121_LORAWAN_REGION_EU868
 * LR1121_LORAWAN_REGION_IN865
 * LR1121_LORAWAN_REGION_KR920
 * LR1121_LORAWAN_REGION_RU864
 * LR1121_LORAWAN_REGION_US915
 */
#define LORAWAN_REGION_USED LR1121_LORAWAN_REGION_EU868

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#if( !USE_LR11XX_CREDENTIALS )
/**
 * @brief Stack credentials
 */
static const uint8_t user_join_eui[8] = LORAWAN_JOIN_EUI;
static const uint8_t user_dev_eui[8]  = LORAWAN_DEVICE_EUI;
static const uint8_t user_nwk_key[16] = LORAWAN_NWK_KEY;
static const uint8_t user_app_key[16] = LORAWAN_APP_KEY;
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

#if( USE_LR11XX_CREDENTIALS )
/**
 * @brief Internal credentials
 */
static uint8_t chip_eui[8] = { 0 };
static uint8_t chip_pin[4] = { 0 };
#endif

extern lr1121_t lr1121;

static volatile bool user_button_is_press = false;  // Flag indicating if the button is pressed
static uint16_t      uplink_counter       = 0;      // Counter for uplinks sent
static uint16_t      confirmed_counter    = 0;      // Counter for confirmed uplinks
static bool          uplink_sending       = false;  // Flag indicating an uplink is requested but not yet sent

extern uint8_t Data[5];
extern int flag;
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief User callback for button EXTI
 *
 * @param context Define by the user at the init
 */
static void user_button_callback( void* context );

/**
 * @brief Send the 32bits uplink counter and 32bits confirmed counter on chosen port
 */
static void send_uplinks_counter_on_port( uint8_t port );

/**
 * @brief Send tx_frame_buffer on choosen port
 *
 */
static lr1121_modem_response_code_t send_frame( const uint8_t* tx_frame_buffer, const uint8_t tx_frame_buffer_size,
                                                uint8_t port, const lr1121_modem_uplink_type_t tx_confirmed );

/**
 * @brief Process received events
 *
 */
static void event_process( void* context );
void read_DHT11_data();
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

int main( void )
{
    // Configure all the microprocessor peripherals (clock, gpio, timer, ...)
    hal_mcu_init( );
    hal_mcu_init_periph( );

    leds_blink( LED_ALL_MASK, 250, 4, true );

    HAL_DBG_TRACE_MSG( "\n\n" );
	HAL_DBG_TRACE_INFO( "===== LoRaWAN example =====\n\n" );

    // Disable IRQ to avoid unwanted behavior during init
    hal_mcu_disable_irq( );

    // Configure Nucleo blue button as EXTI
    hal_gpio_irq_t nucleo_blue_button = {
        .pin      = EXTI_BUTTON,
        .context  = NULL,                  // context passed to the callback - not used in this example
        .callback = user_button_callback,  // callback called when EXTI is triggered
    };
    hal_gpio_init_in( EXTI_BUTTON, HAL_GPIO_PULL_MODE_NONE, HAL_GPIO_IRQ_MODE_FALLING, &nucleo_blue_button );

    // Configure event callback on interrupt
    hal_gpio_irq_t event_callback = {
        .pin      = lr1121.event.pin,
        .context  = &lr1121,        // context passed to the callback
        .callback = event_process,  // callback called when event pin is triggered
    };
    hal_gpio_init_in( lr1121.event.pin, HAL_GPIO_PULL_MODE_NONE, HAL_GPIO_IRQ_MODE_RISING, &event_callback );

    // Flush events before enabling irq
    lr1121_modem_board_event_flush( &lr1121 );

    // Init done: enable interruption
    hal_mcu_enable_irq( );

    /* Board is initialized */
    leds_blink( LED_TX_MASK, 100, 20, true );
    HAL_DBG_TRACE_MSG( "Initialization done\n\n" );
    lr1121_modem_system_reboot( &lr1121, false );

    while( 1 )
    {
        // Check button
        if( user_button_is_press == true )
        {
            user_button_is_press = false;
            HAL_DBG_TRACE_MSG( "Button pushed\n\n" );
            lr1121_modem_lorawan_status_t modem_status;
            lr1121_modem_get_status( &lr1121, &modem_status );
            // Check if the device has already joined a network
            if( ( modem_status & LR1121_LORAWAN_JOINED ) == LR1121_LORAWAN_JOINED )
            {
                if( uplink_sending == false )
                {
                    // Send the uplink counter on port 102
                    send_uplinks_counter_on_port( 102 );
                }
                else
                {
                    HAL_DBG_TRACE_PRINTF(
                        "An uplink has already been requested, please wait for this uplink to be sent before "
                        "requesting another one\n" );
                }
            }
            else
            {
                HAL_DBG_TRACE_PRINTF( "The device has not yet joined the network.\n" );
            }
        }

        hal_mcu_disable_irq( );
        if( ( user_button_is_press == false ) )
        {
            hal_watchdog_reload( );
            hal_mcu_set_sleep_for_ms( WATCHDOG_RELOAD_PERIOD_MS );
        }
        hal_watchdog_reload( );
        hal_mcu_enable_irq( );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void event_process( void* context )
{
    // Continue to read modem events until all of them have been processed.
    lr1121_modem_response_code_t rc_event = LR1121_MODEM_RESPONSE_CODE_OK;
    do
    {
        // Read modem event
        lr1121_modem_event_fields_t current_event;
        rc_event = lr1121_modem_get_event( context, &current_event );
        if( rc_event == LR1121_MODEM_RESPONSE_CODE_OK )
        {
            switch( current_event.event_type )
            {
            case LR1121_MODEM_LORAWAN_EVENT_RESET:

                HAL_DBG_TRACE_MSG_COLOR( "Event received: RESET\n\n", HAL_DBG_TRACE_COLOR_BLUE );

                ASSERT_SMTC_MODEM_RC( lr1121_modem_system_cfg_lfclk( context, LR1121_MODEM_SYSTEM_LFCLK_XTAL, true ) );
                ASSERT_SMTC_MODEM_RC( lr1121_modem_set_crystal_error( context, 50 ) );
                get_and_print_crashlog( context );
#if( !USE_LR11XX_CREDENTIALS )
                // Set user credentials
                HAL_DBG_TRACE_INFO( "###### ===== LR1121 SET EUI and KEYS ==== ######\n\n" );
                ASSERT_SMTC_MODEM_RC( lr1121_modem_set_dev_eui( context, user_dev_eui ) );
                ASSERT_SMTC_MODEM_RC( lr1121_modem_set_join_eui( context, user_join_eui ) );
                ASSERT_SMTC_MODEM_RC( lr1121_modem_set_app_key( context, user_app_key ) );
                ASSERT_SMTC_MODEM_RC( lr1121_modem_set_nwk_key( context, user_nwk_key ) );
                uint8_t tmp_pin[4] = { 0 };  // The chip_pin is not used if we use custom credentials
                print_lorawan_credentials( user_dev_eui, user_join_eui, tmp_pin, USE_LR11XX_CREDENTIALS );
#else
                // Get internal credentials
                uint8_t tmp_join_eui[8] = { 0 };
                ASSERT_SMTC_MODEM_RC( lr1121_modem_system_read_uid( context, chip_eui ) );
                ASSERT_SMTC_MODEM_RC( lr1121_modem_system_read_pin( context, chip_pin ) );
                ASSERT_SMTC_MODEM_RC( lr1121_modem_get_join_eui( context, tmp_join_eui ) );
                print_lorawan_credentials( chip_eui, tmp_join_eui, chip_pin, USE_LR11XX_CREDENTIALS );
#endif

                // Set user region
                ASSERT_SMTC_MODEM_RC( lr1121_modem_set_region( context, LORAWAN_REGION_USED ) );
                print_lorawan_region( LORAWAN_REGION_USED );
                // print user uplink delay
				HAL_DBG_TRACE_INFO( "Periodical uplink (%d sec) \n\n\n", PERIODICAL_UPLINK_DELAY_S );
                // Schedule a LoRaWAN network JoinRequest.
                ASSERT_SMTC_MODEM_RC( lr1121_modem_join( context ) );
                HAL_DBG_TRACE_INFO( "###### ===== JOINING ==== ######\n\n\n" );
                break;

            case LR1121_MODEM_LORAWAN_EVENT_ALARM:
                HAL_DBG_TRACE_MSG_COLOR( "Event received: ALARM\n\n", HAL_DBG_TRACE_COLOR_BLUE );
                // Send periodical uplink on port 101
                send_uplinks_counter_on_port( 101 );
                // Restart periodical uplink alarm
                ASSERT_SMTC_MODEM_RC( lr1121_modem_set_alarm_timer( context, PERIODICAL_UPLINK_DELAY_S ) );
                break;

            case LR1121_MODEM_LORAWAN_EVENT_JOINED:
                HAL_DBG_TRACE_MSG_COLOR( "Event received: JOINED\n", HAL_DBG_TRACE_COLOR_BLUE );
                HAL_DBG_TRACE_INFO( "Modem is now joined \n\n" );

                uint8_t adr_custom_list[16] = { 0 };
                ASSERT_SMTC_MODEM_RC( lr1121_modem_set_adr_profile(
                    context, LR1121_MODEM_ADR_PROFILE_NETWORK_SERVER_CONTROLLED, adr_custom_list ) );

                // Send first periodical uplink on port 101
                send_uplinks_counter_on_port( 101 );
                // start periodical uplink alarm
                ASSERT_SMTC_MODEM_RC( lr1121_modem_set_alarm_timer( context, PERIODICAL_UPLINK_DELAY_S ) );
                break;

            case LR1121_MODEM_LORAWAN_EVENT_TX_DONE:
            {
                const lr1121_modem_tx_done_event_t tx_done_event_data =
                    ( lr1121_modem_tx_done_event_t )( current_event.data >> 8 );
                HAL_DBG_TRACE_MSG_COLOR( "Event received: TXDONE\n\n", HAL_DBG_TRACE_COLOR_BLUE );

                HAL_DBG_TRACE_MSG( "TX DATA     : " );

                switch( tx_done_event_data )
                {
                case LR1121_MODEM_TX_NOT_SENT:
                {
                    HAL_DBG_TRACE_PRINTF( " NOT SENT" );
                    uplink_counter--;
                    break;
                }
                case LR1121_MODEM_CONFIRMED_TX:
                {
                    HAL_DBG_TRACE_PRINTF( " CONFIRMED - ACK" );
                    confirmed_counter++;
                    break;
                }
                case LR1121_MODEM_UNCONFIRMED_TX:
                {
                    HAL_DBG_TRACE_MSG( " UNCONFIRMED\n\n" );
                    break;
                }
                default:
                {
                    HAL_DBG_TRACE_PRINTF( " unknown value (%02x)\n\n", tx_done_event_data );
                }
                }
                HAL_DBG_TRACE_MSG( "\n\n" );

                HAL_DBG_TRACE_INFO( "Transmission done \n" );
                uplink_sending = false;  // Reset flag indicating an uplink request has been processed
                break;
            }

            case LR1121_MODEM_LORAWAN_EVENT_DOWN_DATA:
                HAL_DBG_TRACE_MSG_COLOR( "Event received: DOWNDATA\n\n", HAL_DBG_TRACE_COLOR_BLUE );
                uint8_t rx_payload[LORAWAN_APP_DATA_MAX_SIZE] = { 0 };  // Buffer for rx payload
                uint8_t rx_payload_size                       = 0;      // Size of the payload in the rx_payload buffer
                lr1121_modem_downlink_metadata_t rx_metadata  = { 0 };  // Metadata of downlink
                uint8_t                          rx_remaining = 0;      // Remaining downlink payload in modem
                // Get downlink data
                ASSERT_SMTC_MODEM_RC( lr1121_modem_get_downlink_data_size( context, &rx_payload_size, &rx_remaining ) );
                ASSERT_SMTC_MODEM_RC( lr1121_modem_get_downlink_data( context, rx_payload, rx_payload_size ) );
                ASSERT_SMTC_MODEM_RC( lr1121_modem_get_downlink_metadata( context, &rx_metadata ) );
                HAL_DBG_TRACE_PRINTF( "Data received on port %u\n", rx_metadata.fport );
                HAL_DBG_TRACE_ARRAY( "Received payload", rx_payload, rx_payload_size );
                break;

            case LR1121_MODEM_LORAWAN_EVENT_JOIN_FAIL:
                HAL_DBG_TRACE_MSG_COLOR( "Event received: JOINFAIL\n\n", HAL_DBG_TRACE_COLOR_BLUE );
                break;

            case LR1121_MODEM_LORAWAN_EVENT_LINK_CHECK:
                HAL_DBG_TRACE_MSG_COLOR( "Event received: LINK_CHECK\n\n", HAL_DBG_TRACE_COLOR_BLUE );
                break;

            case LR1121_MODEM_LORAWAN_EVENT_CLASS_B_PING_SLOT_INFO:
                HAL_DBG_TRACE_MSG_COLOR( "Event received: CLASS_B_PING_SLOT_INFO\n\n", HAL_DBG_TRACE_COLOR_BLUE );
                break;

            case LR1121_MODEM_LORAWAN_EVENT_CLASS_B_STATUS:
                HAL_DBG_TRACE_MSG_COLOR( "Event received: CLASS_B_STATUS\n\n", HAL_DBG_TRACE_COLOR_BLUE );
                break;

            case LR1121_MODEM_LORAWAN_EVENT_LORAWAN_MAC_TIME:
                HAL_DBG_TRACE_MSG_COLOR( "Event received: LORAWAN MAC TIME\n\n", HAL_DBG_TRACE_COLOR_BLUE );
                break;

            case LR1121_MODEM_LORAWAN_EVENT_NEW_MULTICAST_SESSION_CLASS_C:
                HAL_DBG_TRACE_MSG_COLOR( "Event received: MULTICAST CLASS_C STOP\n\n", HAL_DBG_TRACE_COLOR_BLUE );
                break;

            case LR1121_MODEM_LORAWAN_EVENT_NEW_MULTICAST_SESSION_CLASS_B:
                HAL_DBG_TRACE_MSG_COLOR( "Event received: MULTICAST CLASS_B STOP\n\n", HAL_DBG_TRACE_COLOR_BLUE );
                break;

            case LR1121_MODEM_LORAWAN_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_C:
                HAL_DBG_TRACE_MSG_COLOR( "Event received: New MULTICAST CLASS_C\n\n", HAL_DBG_TRACE_COLOR_BLUE );
                break;

            case LR1121_MODEM_LORAWAN_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_B:
                HAL_DBG_TRACE_MSG_COLOR( "Event received: New MULTICAST CLASS_B\n\n", HAL_DBG_TRACE_COLOR_BLUE );
                break;
            default:
                HAL_DBG_TRACE_INFO( "Event not handled 0x%02x\n", current_event.event_type );
                break;
            }
        }
    } while( rc_event != LR1121_MODEM_RESPONSE_CODE_NO_EVENT );
}

static void user_button_callback( void* context )
{
    ( void ) context;  // Not used in the example - avoid warning

    static uint32_t last_press_timestamp_ms = 0;

    // Debounce the button press, avoid multiple triggers
    if( ( int32_t )( hal_rtc_get_time_ms( ) - last_press_timestamp_ms ) > 500 )
    {
        last_press_timestamp_ms = hal_rtc_get_time_ms( );
        user_button_is_press    = true;
    }
}

static void send_uplinks_counter_on_port( uint8_t port )
{
    read_DHT11_data();
    // Send uplink and confirmed counter
    uint8_t buff[6] = { 0 };
    buff[0]         = ( uplink_counter >> 8 ) & 0xFF;
    buff[1]         = ( uplink_counter & 0xFF );
    buff[2] = Data[0];
    buff[3] = Data[1];
    buff[4] = Data[2];
    buff[5] = Data[3];
    ASSERT_SMTC_MODEM_RC( send_frame( buff, 6, port, true ) );
    uplink_counter++;  // Increment uplink counter
    uplink_sending = true;
}

static lr1121_modem_response_code_t send_frame( const uint8_t* tx_frame_buffer, const uint8_t tx_frame_buffer_size,
                                                uint8_t port, const lr1121_modem_uplink_type_t tx_confirmed )
{
    lr1121_modem_response_code_t modem_response_code = LR1121_MODEM_RESPONSE_CODE_OK;
    uint8_t                      tx_max_payload;
    int32_t                      duty_cycle;

    lr1121_modem_get_duty_cycle_status( &lr1121, &duty_cycle );

    if( duty_cycle < 0 )
    {
        HAL_DBG_TRACE_INFO( "DUTY CYCLE, NEXT UPLINK AVAILABLE in %d milliseconds \n\n\n", -duty_cycle );
        return modem_response_code;
    }

    modem_response_code = lr1121_modem_get_next_tx_max_payload( &lr1121, &tx_max_payload );
    if( modem_response_code != LR1121_MODEM_RESPONSE_CODE_OK )
    {
        HAL_DBG_TRACE_ERROR( "\n\n lr1121_modem_get_next_tx_max_payload RC : %d \n\n", modem_response_code );
    }

    if( tx_frame_buffer_size > tx_max_payload )
    {
        /* Send empty frame in order to flush MAC commands */
        HAL_DBG_TRACE_PRINTF( "\n\n APP DATA > MAX PAYLOAD AVAILABLE (%d bytes) \n\n", tx_max_payload );
        modem_response_code = lr1121_modem_request_tx( &lr1121, port, tx_confirmed, NULL, 0 );
    }
    else
    {
        modem_response_code =
            lr1121_modem_request_tx( &lr1121, port, tx_confirmed, tx_frame_buffer, tx_frame_buffer_size );
    }

    if( modem_response_code == LR1121_MODEM_RESPONSE_CODE_OK )
    {
        HAL_DBG_TRACE_INFO( "lr1121 MODEM-E REQUEST TX \n\n" );
        HAL_DBG_TRACE_MSG( "TX DATA     : " );
        print_hex_buffer( tx_frame_buffer, tx_frame_buffer_size );
        HAL_DBG_TRACE_MSG( "\n\n\n" );
    }
    else
    {
        HAL_DBG_TRACE_ERROR( "lr1121 MODEM-E REQUEST TX ERROR CMD, modem_response_code : %d \n\n\n",
                             modem_response_code );
    }
    return modem_response_code;
}

void read_DHT11_data()
{
    uint8_t retry_count = 0;
    const uint8_t max_retries = 3;

    while(retry_count < max_retries && flag != 1)
    {
        DHT_Read();
        if(flag != 1)
        {
            // 读取失败，等待短暂时间后重试
            HAL_DBG_TRACE_MSG("read fail, retry...");
            delay_ms(100);
            retry_count++;
        }
    }
    
    if(flag == 1)
    {
        char temp_str[10], humi_str[10];
        sprintf(temp_str, "%d.%d", Data[2], Data[3]);
        sprintf(humi_str, "%d.%d", Data[0], Data[1]);
        
        HAL_DBG_TRACE_INFO("read ok! temp: %sC, humi: %s%%\n", temp_str, humi_str);

        flag = 0;
    }
    else
    {
        HAL_DBG_TRACE_INFO("read fail, retry max times: %d\n", max_retries);
    }
}

void my_test()
{
    uint8_t retry_count = 0;
    const uint8_t max_retries = 3;

    while(retry_count < max_retries && flag != 1)
    {
        DHT_Read();
        if(flag != 1)
        {
            // 读取失败，等待短暂时间后重试
            HAL_Delay(100);
            retry_count++;
        }
    }
      
    if(flag == 1)
    {
        char temp_str[10], humi_str[10];
        sprintf(temp_str, "%d.%d", Data[2], Data[3]);
        sprintf(humi_str, "%d.%d", Data[0], Data[1]);
        
        HAL_DBG_TRACE_INFO("temp: %s, humi: %s\n", temp_str, humi_str);
        flag = 0; // 重置flag
    }
    else
    {
        HAL_DBG_TRACE_INFO("read err\n");
    }
    HAL_Delay(2000);
}
/* --- EOF ------------------------------------------------------------------ */
