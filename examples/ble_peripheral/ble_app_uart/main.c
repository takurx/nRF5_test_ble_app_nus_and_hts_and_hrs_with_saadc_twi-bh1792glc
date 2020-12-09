/*
 *  main.c
 *
 *  Copyright (C) 2019 HERBIO Co., Ltd.
 *  Copyright (C) 2019 takurx, Yoshihiro Nakagawa <taku3wzr@gmail.com>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "boards.h"
#include "nrf_delay.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_hrs.h"
#include "ble_hts.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "sensorsim.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include <bh1792.h>
#include <hr_bh1792.h>
#include "bsp_btn_ble.h"
#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_drv_clock.h"

#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"



//#define DEBUG_MEAS_INTERVAL             1
//#define TWI_DISABLE                     1

#ifndef TWI_DISABLE 
#define FIRMWARE_VERSION                "2p2p0"                                  /* Firmware version, 'ver' command on NUS, :'major'p'minor'p'revision'*/
#define DEVICE_NAME                     "Herbio+"                                /**< Name of device. Will be included in the advertising data. */
#else
#define FIRMWARE_VERSION                "1p2p0"                                  /* Firmware version, 'ver' command on NUS, :'major'p'minor'p'revision'*/
#define DEVICE_NAME                     "Herbio"                                 /**< Name of device. Will be included in the advertising data. */
#endif



#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */
#define MANUFACTURER_NAME               "Herbio Co., Ltd."                       /**< Manufacturer. Will be passed to Device Information Service. */
//#define MODEL_NUM                       "EXAMPLE"                            /**< Model number. Will be passed to Device Information Service. */
#define MANUFACTURER_ID                 0x1122334455                                /**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
#define ORG_UNIQUE_ID                   0x667788                                    /**< Organizational Unique ID, part of System ID. Will be passed to Device Information Service. */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */

#define BATTERY_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(2000)                       /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL               81                                          /**< Minimum battery level as returned by the simulated measurement function. */
#define MAX_BATTERY_LEVEL               100                                         /**< Maximum battery level as returned by the simulated measurement function. */
#define BATTERY_LEVEL_INCREMENT         1                                           /**< Value by which the battery level is incremented/decremented for each call to the simulated measurement function. */

#define HEART_RATE_MEAS_INTERVAL            APP_TIMER_TICKS(1000)                   /**< Heart rate measurement interval (ticks). */
#define MIN_HEART_RATE                      140                                     /**< Minimum heart rate as returned by the simulated measurement function. */
#define MAX_HEART_RATE                      300                                     /**< Maximum heart rate as returned by the simulated measurement function. */
#define HEART_RATE_INCREMENT                10                                      /**< Value by which the heart rate is incremented/decremented for each call to the simulated measurement function. */

#define RR_INTERVAL_INTERVAL                APP_TIMER_TICKS(300)                    /**< RR interval interval (ticks). */
#define MIN_RR_INTERVAL                     100                                     /**< Minimum RR interval as returned by the simulated measurement function. */
#define MAX_RR_INTERVAL                     500                                     /**< Maximum RR interval as returned by the simulated measurement function. */
#define RR_INTERVAL_INCREMENT               1                                       /**< Value by which the RR interval is incremented/decremented for each call to the simulated measurement function. */

#define TEMPERATURE_MEAS_INTERVAL           APP_TIMER_TICKS(1000)                   /**< temperature measurement interval (ticks). */
#define MIN_TEMPERATURE                     33                                      /**< Minimum temperature as returned by the simulated measurement function. */
#define MAX_TEMPERATURE                     42                                      /**< Maximum temperature as returned by the simulated measurement function. */
#define TEMPERATURE_INCREMENT                1                                       /**< Value by which the temperature is incremented/decremented for each call to the simulated measurement function. */

#define SENSOR_CONTACT_DETECTED_INTERVAL    APP_TIMER_TICKS(5000)                   /**< Sensor Contact Detected toggle interval (ticks). */

#define CODE_NOT_WEARING                1                                           /**< heartbeat not measuring, not wearing or not start >**/

#ifndef DEBUG_MEAS_INTERVAL
#define DATA_RECORD_MEAS_INTERVAL           APP_TIMER_TICKS(10000)                   /**< Body Temp. and Heart rate data record interval (ticks). */
#else
//for debug setting
#define DATA_RECORD_MEAS_INTERVAL           APP_TIMER_TICKS(100)                   /**< Body Temp. and Heart rate data record interval (ticks). */
#endif

#define DATA_OUTPUT_INTERVAL                APP_TIMER_TICKS(25)                     /**< nus(nordic uart service) data output interval (ticks). */
//#define DATA_OUTPUT_INTERVAL                APP_TIMER_TICKS(40)                     /**< nus(nordic uart service) data output interval (ticks). */

#define TEMP_TYPE_AS_CHARACTERISTIC     0                                           /**< Determines if temperature type is given as characteristic (1) or as a field of measurement (0). */

#define MIN_CELCIUS_DEGREES             3688                                        /**< Minimum temperature in celcius for use in the simulated measurement function (multiplied by 100 to avoid floating point arithmetic). */
#define MAX_CELCIUS_DEGRESS             3972                                        /**< Maximum temperature in celcius for use in the simulated measurement function (multiplied by 100 to avoid floating point arithmetic). */
#define CELCIUS_DEGREES_INCREMENT       36                                          /**< Value by which temperature is incremented/decremented for each call to the simulated measurement function (multiplied by 100 to avoid floating point arithmetic). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of indication) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */

BLE_HRS_DEF(m_hrs);                                                 /**< Heart rate service instance. */

APP_TIMER_DEF(m_battery_timer_id);                                                  /**< Battery timer. */
BLE_BAS_DEF(m_bas);                                                                 /**< Structure used to identify the battery service. */
BLE_HTS_DEF(m_hts);

NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

APP_TIMER_DEF(m_heart_rate_timer_id);                               /**< Heart rate measurement timer. */
APP_TIMER_DEF(m_rr_interval_timer_id);                              /**< RR interval timer. */
APP_TIMER_DEF(m_sensor_contact_timer_id);                           /**< Sensor contact detected timer. */

APP_TIMER_DEF(m_temperature_timer_id);                               /**< Temperature measurement timer. */

APP_TIMER_DEF(m_data_record_timer_id);                               /**< Measurement data record timer. */
APP_TIMER_DEF(m_data_output_timer_id);                               /**< Measurement data output timer. */

static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static bool     m_rr_interval_enabled = true;                       /**< Flag for enabling and disabling the registration of new RR interval measurements (the purpose of disabling this is just to test sending HRM without RR interval data. */
static uint16_t          m_conn_handle = BLE_CONN_HANDLE_INVALID;                   /**< Handle of the current connection. */
static bool              m_hts_meas_ind_conf_pending = false;                       /**< Flag to keep track of when an indication confirmation is pending. */
static sensorsim_cfg_t   m_battery_sim_cfg;                                         /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t m_battery_sim_state;                                       /**< Battery Level sensor simulator state. */
static sensorsim_cfg_t   m_temp_celcius_sim_cfg;                                    /**< Temperature simulator configuration. */
static sensorsim_state_t m_temp_celcius_sim_state;                                  /**< Temperature simulator state. */

static sensorsim_cfg_t   m_heart_rate_sim_cfg;                      /**< Heart Rate sensor simulator configuration. */
static sensorsim_state_t m_heart_rate_sim_state;                    /**< Heart Rate sensor simulator state. */
static sensorsim_cfg_t   m_rr_interval_sim_cfg;                     /**< RR Interval sensor simulator configuration. */
static sensorsim_state_t m_rr_interval_sim_state;                   /**< RR Interval sensor simulator state. */

static ble_uuid_t m_sr_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_HEART_RATE_SERVICE,           BLE_UUID_TYPE_BLE},
    {BLE_UUID_HEALTH_THERMOMETER_SERVICE,   BLE_UUID_TYPE_BLE},
    {BLE_UUID_BATTERY_SERVICE,              BLE_UUID_TYPE_BLE},
    {BLE_UUID_DEVICE_INFORMATION_SERVICE,   BLE_UUID_TYPE_BLE}
};

#define SAMPLES_IN_BUFFER 1

static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(1);
static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t     m_ppi_channel;

//typedef enum
//{
static const int     STATE_ADVERTISING          = 0;   // 0: Blink GREEN: Advertising that Before Pairing
static const int     STATE_PAIRING              = 1;   // 1: GREEN: Pairing and Idle state
static const int     STATE_MEASURING            = 2;   // 2: RED: Measring state and Pairing
static const int     STATE_EMERGENCY            = 3;   // 3: Blink GREEN and RED: Emergency
static const int     STATE_MEASURINGADVERTISING = 4;   // 4: Blink RED: Measuring and Adverting
static const int     STATE_BOOTING              = 5;   // 5: BLUE: boot, reboot from sleep
static const int     STATE_SLEEPING             = 6;   // 6: Blink BLUE: go to sleep
//}

static volatile int State_keeper = STATE_ADVERTISING;
static volatile bool State_emergency_lock = false;

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

//#ifdef BSP_BUTTON_0
//    #define PIN_IN BSP_BUTTON_0
#ifdef BSP_BUTTON_3
    #define PIN_IN BSP_BUTTON_3
#endif
#ifndef PIN_IN
    #error "Please indicate input pin"
#endif

//#ifdef BSP_LED_0
//    #define PIN_OUT BSP_LED_0
#ifdef BSP_LED_3
    #define PIN_OUT BSP_LED_3
#endif
#ifndef PIN_OUT
    #error "Please indicate output pin"
#endif

APP_TIMER_DEF(m_bh1792glc_timer_id);
//#define BH1792GLC_MEAS_INTERVAL         APP_TIMER_TICKS(1000)   //1 Hz Timer
//#define BH1792GLC_MEAS_INTERVAL         APP_TIMER_TICKS(25)       //40 Hz Timer
//#define BH1792GLC_MEAS_INTERVAL         APP_TIMER_TICKS(10)       //100 Hz Timer
#define BH1792GLC_MEAS_INTERVAL         APP_TIMER_TICKS(31)       //32.258 Hz Timer

/* Indicates if operation on TWI has ended (when received). */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

volatile static bool twi_tx_done = false;
volatile static bool twi_rx_done = false;

bh1792_t      m_bh1792;
bh1792_data_t m_bh1792_dat;

int32_t i2c_write(uint8_t slv_adr, uint8_t reg_adr, uint8_t *reg, uint8_t reg_size);
int32_t i2c_read(uint8_t slv_adr, uint8_t reg_adr, uint8_t *reg, uint8_t reg_size);

#define BH1792_TWI_TIMEOUT 			10000 
#define BH1792_TWI_BUFFER_SIZE     	8 // 8byte = tx max(7) + addr(1)

uint8_t twi_tx_buffer[BH1792_TWI_BUFFER_SIZE];

static void advertising_start(bool erase_bonds);
static void temperature_measurement_send(void);

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,

    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
     * last page of flash available to write data. */
    .start_addr = 0x60000,
    .end_addr   = 0x6ffff,
};

/* Dummy data to write to flash. */
//static uint32_t m_data          = 0xBADC0FFE;
//static char     m_hello_world[] = "hello world";



/**@brief   Helper function to obtain the last address on the last page of the on-chip flash that
 *          can be used to write user data.
 */
static uint32_t nrf5_flash_end_addr_get()
{
    uint32_t const bootloader_addr = NRF_UICR->NRFFW[0];
    uint32_t const page_sz         = NRF_FICR->CODEPAGESIZE;
    uint32_t const code_sz         = NRF_FICR->CODESIZE;

    return (bootloader_addr != 0xFFFFFFFF ?
            bootloader_addr : (code_sz * page_sz));
}

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        default:
            break;
    }
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}



/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') ||
                (data_array[index - 1] == '\r') ||
                (index >= m_ble_nus_max_data_len))
            {
                if (index > 1)
                {
                    NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                    NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                    do
                    {
                        uint16_t length = (uint16_t)index;
                        err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
                            (err_code != NRF_ERROR_RESOURCES) &&
                            (err_code != NRF_ERROR_NOT_FOUND))
                        {
                            APP_ERROR_CHECK(err_code);
                        }
                    } while (err_code == NRF_ERROR_RESOURCES);
                }

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */

#define EBSHCNZWZ_RX_PIN_NUMBER   24
#define EBSHCNZWZ_TX_PIN_NUMBER   25
#define EBSHCNZWZ_RTS_PIN_NUMBER  26
#define EBSHCNZWZ_CTS_PIN_NUMBER  27

/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = EBSHCNZWZ_RX_PIN_NUMBER,
        .tx_pin_no    = EBSHCNZWZ_TX_PIN_NUMBER,
        .rts_pin_no   = EBSHCNZWZ_RTS_PIN_NUMBER,
        .cts_pin_no   = EBSHCNZWZ_CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */



/**@brief Function starting the internal LFCLK oscillator.
 *
 * @details This is needed by RTC1 which is used by the Application Timer
 *          (When SoftDevice is enabled the LFCLK is always running and this is not needed).
 */
static void lfclk_request(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}



/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output,
 * and configures GPIOTE to give an interrupt on pin change.
 */
#define LED_3_COLOR_BLUE_PIN    20
#define LED_3_COLOR_GREEN_PIN   18
#define LED_3_COLOR_RED_PIN     4
#define SWITCH1_PIN             8

#define BH1792GLC_SCL_PIN 7
#define BH1792GLC_SDA_PIN 6
#define BH1792GLC_INT_PIN 5

void bh1792_isr(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

static volatile bool LED_output_state = true;
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    LED_output_state = !LED_output_state;
}

static void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    // bh1792glc, arudino_10_pin, 5
    nrf_drv_gpiote_in_config_t in_config_bh1792 = GPIOTE_CONFIG_IN_SENSE_HITOLO(true); // interrupt when falling edge
    in_config_bh1792.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(BH1792GLC_INT_PIN, &in_config_bh1792, bh1792_isr);
    APP_ERROR_CHECK(err_code);

#ifndef TWI_DISABLE
    nrf_drv_gpiote_in_event_enable(BH1792GLC_INT_PIN, true);
    //nrf_drv_gpiote_in_event_disable(BH1792GLC_INT_PIN);
#else
    //nrf_drv_gpiote_in_event_enable(BH1792GLC_INT_PIN, false);
#endif

    // 3-color LED  LED_3_COLOR_BLUE_PIN, 20
    nrf_drv_gpiote_out_config_t out_config_blue = GPIOTE_CONFIG_OUT_SIMPLE(true);
    err_code = nrf_drv_gpiote_out_init(LED_3_COLOR_BLUE_PIN, &out_config_blue);
    APP_ERROR_CHECK(err_code);

    // 3-color LED  LED_3_COLOR_GREEN_PIN, 18
    nrf_drv_gpiote_out_config_t out_config_green = GPIOTE_CONFIG_OUT_SIMPLE(true);
    err_code = nrf_drv_gpiote_out_init(LED_3_COLOR_GREEN_PIN, &out_config_green);
    APP_ERROR_CHECK(err_code);
    
    // 3-color LED  LED_3_COLOR_RED_PIN, 4
    nrf_drv_gpiote_out_config_t out_config_red = GPIOTE_CONFIG_OUT_SIMPLE(true);
    err_code = nrf_drv_gpiote_out_init(LED_3_COLOR_RED_PIN, &out_config_red);
    APP_ERROR_CHECK(err_code);

    // SWITCH1, 8
    nrf_drv_gpiote_in_config_t in_config_switch1 = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    in_config_switch1.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(SWITCH1_PIN, &in_config_switch1, in_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(SWITCH1_PIN, true);
}






static void battery_level_meas_timeout_handler(void * p_context);
static void heart_rate_meas_timeout_handler(void * p_context);
static void rr_interval_timeout_handler(void * p_context);
static void temperature_meas_timeout_handler(void * p_context);
static void sensor_contact_detected_timeout_handler(void * p_context);
static void bh1792glc_meas_timeout_handler(void * p_context);
static void meas_data_record_timeout_handler(void * p_context);
static void meas_data_output_timeout_handler(void * p_context);

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_heart_rate_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                heart_rate_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_rr_interval_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                rr_interval_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_temperature_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                temperature_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_sensor_contact_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                sensor_contact_detected_timeout_handler);
    APP_ERROR_CHECK(err_code);
                             
    err_code = app_timer_create(&m_bh1792glc_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                bh1792glc_meas_timeout_handler);        
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_data_record_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                meas_data_record_timeout_handler);        
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_data_output_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                meas_data_output_timeout_handler);        
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for performing a battery measurement, and update the Battery Level characteristic in the Battery Service.
 */
volatile float Battery_percent = 0.0;

static void battery_level_update(void)
{
    ret_code_t err_code;
    uint8_t  battery_level;

    //battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);
    battery_level = (uint8_t)(Battery_percent);

    err_code = ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) &&
        (err_code != NRF_ERROR_FORBIDDEN)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}

/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context   Pointer used for passing some arbitrary information (context) from the
 *                        app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}

/**@brief Function for handling the Heart rate measurement timer timeout.
 *
 * @details This function will be called each time the heart rate measurement timer expires.
 *          It will exclude RR Interval data from every third measurement.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
bool Debug_output_heart_rate = false;
bool Debug_output_body_temperature = false;
bool Debug_output_battery_temperature = false;
bool Debug_output_battery_voltage = false;
bool Debug_output_current_time = true;

uint8_t   Bpm     = 0U;
uint8_t   Wearing = 0U;

static void heart_rate_meas_timeout_handler(void * p_context)
{
    static uint32_t cnt = 0;
    ret_code_t      err_code;
    uint16_t        heart_rate;

    UNUSED_PARAMETER(p_context);

    //heart_rate = (uint16_t)sensorsim_measure(&m_heart_rate_sim_state, &m_heart_rate_sim_cfg);
    heart_rate = Bpm;

    cnt++;
    err_code = ble_hrs_heart_rate_measurement_send(&m_hrs, heart_rate);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) &&
        (err_code != NRF_ERROR_FORBIDDEN)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }

    // Disable RR Interval recording every third heart rate measurement.
    // NOTE: An application will normally not do this. It is done here just for testing generation
    // of messages without RR Interval measurements.
    m_rr_interval_enabled = ((cnt % 3) != 0);
}

/**@brief Function for handling the RR interval timer timeout.
 *
 * @details This function will be called each time the RR interval timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void rr_interval_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    if (m_rr_interval_enabled)
    {
        uint16_t rr_interval;

        rr_interval = (uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
                                                  &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
    }
}

/**@brief Function for populating simulated health thermometer measurement.
 */
volatile float Body_temperature = 0.0;
volatile float Battery_temperature = 0.0;

static ble_date_time_t time_stamp = { 2019, 7, 31, 19, 8, 40 };
static const int month_days[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

static void hts_measurement(ble_hts_meas_t * p_meas)
{    
    uint32_t celciusX100;

    p_meas->temp_in_fahr_units = false;
    p_meas->time_stamp_present = true;
    p_meas->temp_type_present  = (TEMP_TYPE_AS_CHARACTERISTIC ? false : true);

    //celciusX100 = sensorsim_measure(&m_temp_celcius_sim_state, &m_temp_celcius_sim_cfg);
    celciusX100 = (uint32_t)(Body_temperature * 100);

    p_meas->temp_in_celcius.exponent = -2;
    p_meas->temp_in_celcius.mantissa = celciusX100;
    p_meas->temp_in_fahr.exponent    = -2;
    p_meas->temp_in_fahr.mantissa    = (32 * 100) + ((celciusX100 * 9) / 5);
    p_meas->time_stamp               = time_stamp;
    //p_meas->temp_type                = BLE_HTS_TEMP_TYPE_FINGER;
    p_meas->temp_type                = BLE_HTS_TEMP_TYPE_BODY;

    // update simulated time stamp
    time_stamp.seconds++;
    if (time_stamp.seconds > 59)
    {
        time_stamp.seconds = 0;
        time_stamp.minutes++;
        if (time_stamp.minutes > 59)
        {
            time_stamp.minutes = 0;
            time_stamp.hours++;
            if (time_stamp.hours > 23)
            {
                time_stamp.hours = 0;
                time_stamp.day++;
                if (time_stamp.day > month_days[time_stamp.month])
                {
                    if (time_stamp.month == 2)
                    {
                        if ((time_stamp.year % 4 == 0 && time_stamp.year % 100 != 0) || (time_stamp.year % 400 == 0))   // leap year
                        {
                            if (time_stamp.day > month_days[time_stamp.month] + 1)
                            {
                                time_stamp.day = 1;
                                time_stamp.month++;
                            }
                        }
                        else    // not leap year
                        {
                            time_stamp.day = 1;
                            time_stamp.month++;
                        }
                    }
                    else
                    {
                        time_stamp.day = 1;
                        time_stamp.month++;
                        if (time_stamp.month > 12)
                        {
                            time_stamp.month = 1;
                            time_stamp.year++;
                        }
                    }
                }
            }
        }
    }
    
    if (Debug_output_current_time == true)
    {
        NRF_LOG_INFO("%04d-%02d-%02dT%02d:%02d:%02d", time_stamp.year, time_stamp.month, time_stamp.day, time_stamp.hours, time_stamp.minutes, time_stamp.seconds);
    }
}

/**@brief Function for handling the Temperature measurement timer timeout.
 *
 * @details 
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void temperature_meas_timeout_handler(void * p_context)
{
    ble_hts_meas_t simulated_meas;
    ret_code_t     err_code;

    UNUSED_PARAMETER(p_context);

    hts_measurement(&simulated_meas);

    err_code = ble_hts_measurement_send(&m_hts, &simulated_meas);

    switch (err_code)
    {
      case NRF_SUCCESS:
        // Measurement was successfully sent, wait for confirmation.
        m_hts_meas_ind_conf_pending = true;
        break;
      case NRF_ERROR_INVALID_STATE:
        // Ignore error.
        break;
      default:
        //APP_ERROR_HANDLER(err_code);
        break;
    }
    
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) &&
        (err_code != NRF_ERROR_FORBIDDEN)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}

/**@brief Function for handling the Sensor Contact Detected timer timeout.
 *
 * @details This function will be called each time the Sensor Contact Detected timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void sensor_contact_detected_timeout_handler(void * p_context)
{
    static bool sensor_contact_detected = false;

    UNUSED_PARAMETER(p_context);

    sensor_contact_detected = !sensor_contact_detected;
    ble_hrs_sensor_contact_detected_update(&m_hrs, sensor_contact_detected);
}

static void timer_isr(void * p_context)
{
    //UNUSED_PARAMETER(p_context);
    //NRF_LOG_INFO("timer_isr.");
    
    int32_t ret = 0;

    nrf_drv_gpiote_in_event_disable(BH1792GLC_INT_PIN);
    
    //m_bh1792.prm.led_cur1 = BH1792_PRM_LED_CUR1_MA(0);
    //m_bh1792.prm.led_cur2 = BH1792_PRM_LED_CUR2_MA(0);
    pw_GetParam(BH1792_PRM_CTRL2_CUR_LED1, &(m_bh1792.prm.led_cur1));
    pw_GetParam(BH1792_PRM_CTRL3_CUR_LED2, &(m_bh1792.prm.led_cur2));
    ret = bh1792_SetParams();
    //ret = bh1792_StartMeasure();
    //error_check(ret, "bh1792_StartMeasure");
    ret = hr_bh1792_StartMeasure();
    //error_check(ret, "hr_bh1792_StartMeasure");

    nrf_drv_gpiote_in_event_enable(BH1792GLC_INT_PIN, true);
}

/**@brief Function for handling the BH1792GLC measurement timer timeout.
 *
 * @details This function will be called each time BH1792GLC measurement timer expires.
 *
 * @param[in] p_context   Pointer used for passing some arbitrary information (context) from the
 *                        app_start_timer() call to the timeout handler.
 */
static void bh1792glc_meas_timeout_handler(void * p_context)
{
    //NRF_LOG_INFO("bh1792glc measure timer interrupt.");
    timer_isr(p_context);    
}



/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch(p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            switch(p_event->xfer_desc.type)
            {
                case NRF_DRV_TWI_XFER_TX:
                    twi_tx_done = true;
                    break;
                case NRF_DRV_TWI_XFER_TXTX:
                    twi_tx_done = true;
                    break;
                case NRF_DRV_TWI_XFER_RX:
                    twi_rx_done = true;
                    //m_xfer_done = true;
                    break;
                case NRF_DRV_TWI_XFER_TXRX:
                    twi_rx_done = true;
                    //m_xfer_done = true;
                    break;
                default:
                    break;
            }
            break;
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            break;
        case NRF_DRV_TWI_EVT_DATA_NACK:
            break;
        default:
            break;
    }
}



/**
 * @brief Measurement data record events handler.
 */
#ifndef DEBUG_MEAS_INTERVAL
#define Num_of_data_hr_hr   256
static volatile unsigned int Count_10sec = 59;   // 10 minutes
#else
//for debug setting
#define Num_of_data_hr_hr   32
//for debug setting
static volatile unsigned int Count_10sec = 9;   // 100 seconds
#endif

static volatile unsigned int Meas10sec = 0;
static volatile unsigned int Write_index_data_hr_hr = 0;
static volatile unsigned int Read_index_data_hr_hr = 0;
static volatile unsigned int Count_index_data_hr_hr = 0;
/**@brief record every 10 mintutes, 
 * temprature: 6 points between 10 seconds
 * heart_rate: 60 second average
 */
typedef struct
{
    ble_date_time_t start_time;         // 7byte = 56 = 16 + 8 + 8 + 8 + 8 + 8
    //uint8_t reserved_0;                 // 1byte = 8
    float body_temperature_array[6];    // 4byte = 32
    uint16_t heart_rate;                // 2byte = 16
    uint16_t reserved_1;                // 2byte = 16
} ble_data_ht_hr_t;
static volatile ble_data_ht_hr_t data_hr_hr[Num_of_data_hr_hr] = {};
static volatile ble_data_ht_hr_t temp_data_hr_hr = {};
//static ble_date_time_t time_stamp = { 2019, 2, 28, 23, 59, 50 };

static void meas_data_record_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    //NRF_LOG_INFO("10 second interval, it will measurement dara record");

    //  measure 10 seconds, record 10 minutes
    if (Meas10sec < 6)
    {
        //NRF_LOG_INFO("10 second measure, 6 times");
        if (Meas10sec == 0)
        {
            temp_data_hr_hr.start_time.year     = time_stamp.year;
            temp_data_hr_hr.start_time.month    = time_stamp.month;
            temp_data_hr_hr.start_time.day      = time_stamp.day;
            temp_data_hr_hr.start_time.hours    = time_stamp.hours;
            temp_data_hr_hr.start_time.minutes  = time_stamp.minutes;
            temp_data_hr_hr.start_time.seconds  = time_stamp.seconds;
        }

        temp_data_hr_hr.body_temperature_array[Meas10sec] = Body_temperature;
        NRF_LOG_INFO("R10sec %d:" NRF_LOG_FLOAT_MARKER "", Meas10sec, NRF_LOG_FLOAT(Body_temperature));
        
        if (Meas10sec == 5)
        {
            if (Wearing == 1)
            {
              temp_data_hr_hr.heart_rate = Bpm;
            }
            else
            {
              temp_data_hr_hr.heart_rate = CODE_NOT_WEARING;
            }

            data_hr_hr[Write_index_data_hr_hr] = temp_data_hr_hr;

            Write_index_data_hr_hr++;
            if (Write_index_data_hr_hr > Num_of_data_hr_hr - 1)
            {
                Write_index_data_hr_hr = 0;
            }

            Count_index_data_hr_hr++;
            if (Count_index_data_hr_hr > Num_of_data_hr_hr - 1)
            {
                Count_index_data_hr_hr = Num_of_data_hr_hr - 1;
                Read_index_data_hr_hr = Write_index_data_hr_hr + 1;
                if (Read_index_data_hr_hr > Num_of_data_hr_hr - 1)
                {
                    Read_index_data_hr_hr = 0;
                }
                NRF_LOG_INFO("data full:%03d", Count_index_data_hr_hr);
            }
            else
            {
                NRF_LOG_INFO("data increment:%03d", Count_index_data_hr_hr);
            }
            NRF_LOG_INFO("Write: %d", Write_index_data_hr_hr);
            NRF_LOG_INFO("Read: %d", Read_index_data_hr_hr);
            NRF_LOG_INFO("Count: %d", Count_index_data_hr_hr);
            NRF_LOG_FLUSH();
        }
    }

    Meas10sec++;
    if(Meas10sec > Count_10sec)
    {
        Meas10sec = 0;
    }
}

static void meas_data_output_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    
    uint32_t err_code;

    static char restime[20] =   "";
    static char resdatanum[4] = "";
    static char respulse[4] =   "";
    static char restemp[36] =   "";
    static char resdata[128] =  "";
    uint16_t reslength;

    //NRF_LOG_INFO("it will measurement data output");

    sprintf(restime, "%04d-%02d-%02dT%02d:%02d:%02d", 
    data_hr_hr[Read_index_data_hr_hr].start_time.year, 
    data_hr_hr[Read_index_data_hr_hr].start_time.month, 
    data_hr_hr[Read_index_data_hr_hr].start_time.day, 
    data_hr_hr[Read_index_data_hr_hr].start_time.hours, 
    data_hr_hr[Read_index_data_hr_hr].start_time.minutes, 
    data_hr_hr[Read_index_data_hr_hr].start_time.seconds);
    sprintf(resdatanum, "%03d", Count_index_data_hr_hr);
    sprintf(respulse,"%03d", data_hr_hr[Read_index_data_hr_hr].heart_rate);
    NRF_LOG_INFO("%03d", data_hr_hr[Read_index_data_hr_hr].heart_rate);

    sprintf(restemp, 
        "" NRF_LOG_FLOAT_MARKER "," NRF_LOG_FLOAT_MARKER "," NRF_LOG_FLOAT_MARKER "," NRF_LOG_FLOAT_MARKER "," NRF_LOG_FLOAT_MARKER "," NRF_LOG_FLOAT_MARKER "", 
        NRF_LOG_FLOAT(data_hr_hr[Read_index_data_hr_hr].body_temperature_array[0]),
        NRF_LOG_FLOAT(data_hr_hr[Read_index_data_hr_hr].body_temperature_array[1]),
        NRF_LOG_FLOAT(data_hr_hr[Read_index_data_hr_hr].body_temperature_array[2]),
        NRF_LOG_FLOAT(data_hr_hr[Read_index_data_hr_hr].body_temperature_array[3]),
        NRF_LOG_FLOAT(data_hr_hr[Read_index_data_hr_hr].body_temperature_array[4]),
        NRF_LOG_FLOAT(data_hr_hr[Read_index_data_hr_hr].body_temperature_array[5]));
    strcpy(resdata, restime);
    strcat(resdata, ",");
    strcat(resdata, resdatanum);
    strcat(resdata, ",");
    strcat(resdata, respulse);
    strcat(resdata, ",");
    strcat(resdata, restemp);
    reslength = strlen(resdata);

    err_code = ble_nus_data_send(&m_nus, &resdata[0], &reslength, m_conn_handle);

    Read_index_data_hr_hr++;
    if (Read_index_data_hr_hr > Num_of_data_hr_hr - 1)
    {
        Read_index_data_hr_hr = 0;
    }

    //Count_index_data_hr_hr = 0;
    Count_index_data_hr_hr--;
    NRF_LOG_INFO("data decrement:%03d, %03d", Count_index_data_hr_hr, Read_index_data_hr_hr - 1);

    if (Count_index_data_hr_hr < 1)
    {
        Count_index_data_hr_hr = 0;
        err_code = app_timer_stop(m_data_output_timer_id);
        APP_ERROR_CHECK(err_code);
    }
    NRF_LOG_INFO("Write: %d", Write_index_data_hr_hr);
    NRF_LOG_INFO("Read: %d", Read_index_data_hr_hr);
    NRF_LOG_INFO("Count: %d", Count_index_data_hr_hr);
    NRF_LOG_FLUSH();
}






/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;
/*
    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
*/
    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}



/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}






/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            if (State_keeper == STATE_MEASURINGADVERTISING)
            {
                State_keeper = STATE_MEASURING;
            }
            else
            {
                // not state_measuring
                State_keeper = STATE_PAIRING;
            }
            NRF_LOG_INFO("State_keeper: %d", State_keeper);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected, reason %d.",
                          p_ble_evt->evt.gap_evt.params.disconnected.reason);
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            m_hts_meas_ind_conf_pending = false;
            if (State_keeper == STATE_MEASURING)
            {
                State_keeper = STATE_MEASURINGADVERTISING;
            }
            else
            {
                // not state_measuring
                State_keeper = STATE_ADVERTISING;
            }
            NRF_LOG_INFO("State_keeper: %d", State_keeper);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            //err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            //APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            //err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            //APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
        
        case BLE_GAP_EVT_AUTH_KEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_AUTH_KEY_REQUEST");
            break;

        case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
            break;

         case BLE_GAP_EVT_AUTH_STATUS:
             NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
                          p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                          p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                          p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}



/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_THERMOMETER);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
    
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("GATT ATT MTU on connection 0x%x changed to %d.",
                     p_evt->conn_handle,
                     p_evt->params.att_mtu_effective);
    }

    ble_hrs_on_gatt_evt(&m_hrs, p_evt);
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}






/**@brief Function for initializing services that will be used by the application.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error);
static void nus_data_handler(ble_nus_evt_t * p_evt);
static void on_hts_evt(ble_hts_t * p_hts, ble_hts_evt_t * p_evt);

static void services_init(void)
{
    uint32_t           err_code;
    ble_hrs_init_t     hrs_init;
    ble_hts_init_t     hts_init;
    ble_bas_init_t     bas_init;
    ble_dis_init_t     dis_init;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};
    uint8_t            body_sensor_location;
    ble_dis_sys_id_t   sys_id;

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Health Thermometer Service
    memset(&hts_init, 0, sizeof(hts_init));

    hts_init.evt_handler                 = on_hts_evt;
    //hts_init.evt_handler                 = NULL;
    hts_init.temp_type_as_characteristic = TEMP_TYPE_AS_CHARACTERISTIC;
    hts_init.temp_type                   = BLE_HTS_TEMP_TYPE_BODY;

    // Here the sec level for the Health Thermometer Service can be changed/increased.
    //hts_init.ht_meas_cccd_wr_sec = SEC_JUST_WORKS;
    hts_init.ht_meas_cccd_wr_sec = SEC_OPEN;
    hts_init.ht_type_rd_sec      = SEC_OPEN;

    err_code = ble_hts_init(&m_hts, &hts_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Heart Rate Service.
    body_sensor_location = BLE_HRS_BODY_SENSOR_LOCATION_FINGER;

    memset(&hrs_init, 0, sizeof(hrs_init));

    hrs_init.evt_handler                 = NULL;
    hrs_init.is_sensor_contact_supported = true;
    hrs_init.p_body_sensor_location      = &body_sensor_location;

    // Here the sec level for the Heart Rate Service can be changed/increased.
    hrs_init.hrm_cccd_wr_sec = SEC_OPEN;
    hrs_init.bsl_rd_sec      = SEC_OPEN;

    err_code = ble_hrs_init(&m_hrs, &hrs_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    bas_init.bl_rd_sec        = SEC_OPEN;
    bas_init.bl_cccd_wr_sec   = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);
    //ble_srv_ascii_to_utf8(&dis_init.model_num_str, MODEL_NUM);

    //sys_id.manufacturer_id            = MANUFACTURER_ID;
    //sys_id.organizationally_unique_id = ORG_UNIQUE_ID;
    //dis_init.p_sys_id                 = &sys_id;

    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static const char * NusCommand[] = 
{
    "sta",    /* 0: start measurement command */
    "sto",    /* 1: stop measurement command  */
    "rqs",    /* 2: request series command    */
    "rqd",    /* 3: request data command      */
    "scd",    /* 4: set current days command  Ex. "scd 2018-12-25" */
    "sct",    /* 5: set current time command  Ex. "sct 12:20:15"   */
    "slp",    /* 6: set sleep enter command */
    "nnn", "nnn", "nnn",     /* 7-9 */
    "nnn", "nnn", "nnn", "nnn", "nnn", "nnn", "nnn", "nnn", "nnn", "nnn",     /* 10-19 */
    "nnn", "nnn", "nnn", "nnn", "nnn", "nnn", "nnn",                          /* 20-26 */
    "dhs",    /* 27: debug heartrate measure start */
    "dhe",    /* 28: debug heartrate measure end  */ 
    "dhg",    /* 29: debug output heartrate */
    "dhr",    /* 30: debug output heart rate command (state change) */
    "dbt",    /* 31: debug output temperature command (state change) */
    "dsp",    /* 32: debug output stop command (state change) */
    "dct",    /* 33: debut output current time */
    "ver",    /* 34: debug output firmware version */
    "dcm",    /* 35: debug Change measurement interval, Count_10sec */
    "dmi",    /* 36: debug output measurement interval, Count_10sec */
    "dba",    /* 37: debug output battery percentage */
    "nnn", "nnn",     /* 38-39 */
};
//uint16_t Number_of_command = 40;
uint16_t Number_of_command = sizeof(NusCommand)/sizeof(char *);

static void sleep_mode_enter();

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;
        static char com_buf[128] = "";
        uint16_t i;
        uint16_t j;
        uint16_t buf_ind;
        int ind;

        char restime[20] =   ""; //"2018-12-25T12:20:15";
        char resdatanum[4] = "";
        //char respulse[4] =   "";
        char restemp[36] =   ""; //"36.00,36.01,36.02,36.03,36.04,36.05";
        char resdata[128] =  "";

        for (i = 0; i < p_evt->params.rx_data.length; i++)
        {
            do
            {
                err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
                    NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                    APP_ERROR_CHECK(err_code);
                }
                //NRF_LOG_INFO("string: %c, %d", p_evt->params.rx_data.p_data[i], i);
                com_buf[i] = p_evt->params.rx_data.p_data[i];
            } while (err_code == NRF_ERROR_BUSY);
        }
        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
        {
            while (app_uart_put('\n') == NRF_ERROR_BUSY);
        }

        com_buf[p_evt->params.rx_data.length] = '\0';
        //NRF_LOG_INFO("command: %s", (uint8_t)(&com_buf[0]));
        NRF_LOG_INFO("command: %s", p_evt->params.rx_data.p_data);

        uint16_t reslength;
        long temp_year;
        long temp_month;
        long temp_day;
        long temp_hours;
        long temp_minutes;
        long temp_seconds;
        long max_temp_days;
        unsigned int temp_time_interval;

        NRF_LOG_INFO("number of command: %d", Number_of_command);
        for (i = 0; i < Number_of_command; i++)
        {
            if((strncmp(com_buf, NusCommand[i], 3)) == 0)
            {
                switch (i)
                {
                    case 0:   // 0: sta
                        if (State_keeper == STATE_PAIRING)
                        {
#ifndef TWI_DISABLE
                            err_code = app_timer_start(m_bh1792glc_timer_id, BH1792GLC_MEAS_INTERVAL, NULL);
                            APP_ERROR_CHECK(err_code);
                            //nrf_drv_gpiote_in_event_enable(BH1792GLC_INT_PIN, true);
#endif

                            err_code = app_timer_start(m_data_record_timer_id, DATA_RECORD_MEAS_INTERVAL, NULL);
                            APP_ERROR_CHECK(err_code);
                            NRF_LOG_INFO("10 second measure and 10 minutes record start");
                            Meas10sec = 0;
                            //Count_index_data_hr_hr = 0;
                            State_keeper = STATE_MEASURING;
                            NRF_LOG_INFO("State_keeper: %d", State_keeper);
                            reslength = 3;
                            err_code = ble_nus_data_send(&m_nus, "ack", &reslength, m_conn_handle);
                        }
                        else
                        {
                            NRF_LOG_INFO("already record started");
                            reslength = 3;
                            err_code = ble_nus_data_send(&m_nus, "nak", &reslength, m_conn_handle);                            
                        }
                        break;
                    case 1:   // 1: sto
#ifndef TWI_DISABLE
                        err_code = app_timer_stop(m_bh1792glc_timer_id);
                        APP_ERROR_CHECK(err_code);
                        //nrf_drv_gpiote_in_event_disable(BH1792GLC_INT_PIN);
#endif
    
                        err_code = app_timer_stop(m_data_record_timer_id);
                        APP_ERROR_CHECK(err_code);
                        NRF_LOG_INFO("10 second measure and 10 minutes record stop");
                        if (State_keeper == STATE_MEASURING)
                        {
                            State_keeper = STATE_PAIRING;
                            NRF_LOG_INFO("State_keeper: %d", State_keeper);
                            reslength = 3;
                            err_code = ble_nus_data_send(&m_nus, "ack", &reslength, m_conn_handle);
                        }
                        else
                        {
                            reslength = 3;
                            err_code = ble_nus_data_send(&m_nus, "nak", &reslength, m_conn_handle);
                        }
                        break;
                    case 2:   // 2: rqs
                        sprintf(resdatanum, "%03d", Count_index_data_hr_hr);
                        reslength = 3;
                        err_code = ble_nus_data_send(&m_nus, resdatanum, &reslength, m_conn_handle);
                        break;
                    case 3:   // 3: rqd
                        if (Count_index_data_hr_hr > 0)
                        {
                            err_code = app_timer_start(m_data_output_timer_id, DATA_OUTPUT_INTERVAL, NULL);
                            APP_ERROR_CHECK(err_code);
                            NRF_LOG_INFO("data output start");
                            reslength = 3;
                            err_code = ble_nus_data_send(&m_nus, "ack", &reslength, m_conn_handle);
                        }
                        else
                        {
                            NRF_LOG_INFO("data output not start, none data");
                            reslength = 3;
                            err_code = ble_nus_data_send(&m_nus, "nak", &reslength, m_conn_handle);
                        }
                        break;
                    case 4:   // 4: scd
                        /* Ex. "scd 2018-01-03" */
                        temp_year =  strtol((const char *)(&com_buf[4]), NULL, 10);
                        temp_month = strtol((const char *)(&com_buf[9]), NULL, 10);
                        temp_day =  strtol((const char *)(&com_buf[12]), NULL, 10);
                        
                        max_temp_days = month_days[temp_month];
                        if (temp_month == 2)
                        {
                            if ((temp_year % 4 == 0 && temp_year % 100 != 0) || (temp_year % 400 == 0)) //leap year
                            {
                                max_temp_days = max_temp_days + 1;
                            }
                        }

                        if ((temp_year >= 1900 && temp_year < 2200) &&
                            (temp_month >= 1 && temp_month <= 12) &&
                            (temp_day >= 1 && temp_day <= max_temp_days))
                        {
                          time_stamp.year = temp_year;
                          time_stamp.month = temp_month;
                          time_stamp.day = temp_day;
                          reslength = 3;
                          err_code = ble_nus_data_send(&m_nus, "ack", &reslength, m_conn_handle);
                        }
                        else
                        {
                          reslength = 3;
                          err_code = ble_nus_data_send(&m_nus, "nak", &reslength, m_conn_handle);
                        }
                        break;
                    case 5:   // 5: sct
                        /* Ex. "sct 23:59:55" */
                        temp_hours =   strtol((const char *)(&com_buf[4]), NULL, 10);
                        temp_minutes = strtol((const char *)(&com_buf[7]), NULL, 10);
                        temp_seconds = strtol((const char *)(&com_buf[10]), NULL, 10);

                        if ((temp_hours >= 0 && temp_hours < 24) &&
                            (temp_minutes >= 0 && temp_minutes < 60) &&
                            (temp_seconds >= 0 && temp_seconds < 60))
                        {
                          time_stamp.hours = temp_hours;
                          time_stamp.minutes = temp_minutes;
                          time_stamp.seconds = temp_seconds;
                          reslength = 3;
                          err_code = ble_nus_data_send(&m_nus, "ack", &reslength, m_conn_handle);
                        }
                        else
                        {
                          reslength = 3;
                          err_code = ble_nus_data_send(&m_nus, "nak", &reslength, m_conn_handle);
                        }
                        break;
                    case 6:   // 6: slp, set sleep enter command
                        reslength = 3;
                        err_code = ble_nus_data_send(&m_nus, "ack", &reslength, m_conn_handle);
                        NRF_LOG_INFO("set sleep, good night");
                        NRF_LOG_FLUSH();
                        sleep_mode_enter();
                        break;
                    case 27:    //dhs, 27: debug heartrate measure start
#ifndef TWI_DISABLE
                        err_code = app_timer_start(m_bh1792glc_timer_id, BH1792GLC_MEAS_INTERVAL, NULL);
                        APP_ERROR_CHECK(err_code);
                        //nrf_drv_gpiote_in_event_enable(BH1792GLC_INT_PIN, true);

                        reslength = 3;
                        err_code = ble_nus_data_send(&m_nus, "ack", &reslength, m_conn_handle);
#else
                        reslength = 3;
                        err_code = ble_nus_data_send(&m_nus, "nak", &reslength, m_conn_handle);
#endif
                        break;
                    case 28:    //dhe, 28: debug heartrate measure end
#ifndef TWI_DISABLE
                        err_code = app_timer_stop(m_bh1792glc_timer_id);
                        APP_ERROR_CHECK(err_code);
                        //nrf_drv_gpiote_in_event_disable(BH1792GLC_INT_PIN);

                        reslength = 3;
                        err_code = ble_nus_data_send(&m_nus, "ack", &reslength, m_conn_handle);
#else
                        reslength = 3;
                        err_code = ble_nus_data_send(&m_nus, "nak", &reslength, m_conn_handle);
#endif
                        break;
                    case 29:    //dhg, 29: debug output heartrate
                        if (Wearing == 1)
                        {
                            sprintf(resdatanum, "%03d", Bpm);
                        }
                        else
                        {
                            sprintf(resdatanum, "%03d", CODE_NOT_WEARING);
                        }
                        reslength = 3;
                        err_code = ble_nus_data_send(&m_nus, resdatanum, &reslength, m_conn_handle);
                        break;
                    case 30:    // 30: dhr
                        Debug_output_heart_rate = true;
                        Debug_output_body_temperature = false;
                        reslength = 3;
                        err_code = ble_nus_data_send(&m_nus, "ack", &reslength, m_conn_handle);
                        break;
                    case 31:    // 31: dbt
                        Debug_output_heart_rate = false;
                        Debug_output_body_temperature = true;
                        reslength = 3;
                        err_code = ble_nus_data_send(&m_nus, "ack", &reslength, m_conn_handle);
                        break;
                    case 32:    // 32: dsp
                        Debug_output_heart_rate = false;
                        Debug_output_body_temperature = false;
                        reslength = 3;
                        err_code = ble_nus_data_send(&m_nus, "ack", &reslength, m_conn_handle);
                        break;
                    case 33:   // 33: dct
                        //char restime[] =    "2018-12-25T12:20:15";
                        NRF_LOG_INFO("%04d-%02d-%02dT%02d:%02d:%02d", time_stamp.year, time_stamp.month, time_stamp.day, time_stamp.hours, time_stamp.minutes, time_stamp.seconds);
                        sprintf(restime, "%04d-%02d-%02dT%02d:%02d:%02d", time_stamp.year, time_stamp.month, time_stamp.day, time_stamp.hours, time_stamp.minutes, time_stamp.seconds);
                        reslength = strlen(restime);
                        err_code = ble_nus_data_send(&m_nus, &restime[0], &reslength, m_conn_handle);
                        break;
                    case 34:   // 34: ver
                        NRF_LOG_INFO("FIRMWARE_VERSION: %s", FIRMWARE_VERSION);
                        sprintf(resdata, "FIRMWARE_VERSION: %s", FIRMWARE_VERSION);
                        reslength = strlen(resdata);
                        err_code = ble_nus_data_send(&m_nus, &resdata[0], &reslength, m_conn_handle);
                        break;
                    case 35:   //35: "dcm", debug Change measurement interval, Meas10sec
                        /* Ex. "dcm 0060", range: 060 - 990 */
                        temp_time_interval = strtol((const char *)(&com_buf[4]), NULL, 10);
                        NRF_LOG_INFO("SET MEAS INTERVAL: %03d", temp_time_interval);
                        
                        if (temp_time_interval >= 60 && temp_time_interval <= 990)
                        {
                          Count_10sec = temp_time_interval / 10 - 1;
                          reslength = 3;
                          err_code = ble_nus_data_send(&m_nus, "ack", &reslength, m_conn_handle);
                        }
                        else
                        {
                          reslength = 3;
                          err_code = ble_nus_data_send(&m_nus, "nak", &reslength, m_conn_handle);
                        }
                        break;
                    case 36:  //36: "dmi", debug output measurement interval, Meas10sec
                        temp_time_interval = (Count_10sec + 1) * 10;
                        NRF_LOG_INFO("MEAS INTERVAL: %03d", temp_time_interval);
                        sprintf(resdata, "MEAS INTERVAL: %03d", temp_time_interval);
                        reslength = strlen(resdata);
                        err_code = ble_nus_data_send(&m_nus, &resdata[0], &reslength, m_conn_handle);
                        break;
                    case 37: // 37: "dba", debug output battery percentage
                        NRF_LOG_INFO("BAT: " NRF_LOG_FLOAT_MARKER "", NRF_LOG_FLOAT(Battery_percent));
                        sprintf(resdata, "BAT: %03d", (int)(Battery_percent));
                        //sprintf(resdata, "BAT: ", NRF_LOG_FLOAT_MARKER " %", NRF_LOG_FLOAT(Battery_percent));
                        reslength = strlen(resdata);
                        err_code = ble_nus_data_send(&m_nus, &resdata[0], &reslength, m_conn_handle);
                        break;
                    default:
                        break;
                }
                break;
            }
            NRF_LOG_INFO("command check done: %d", i);
            NRF_LOG_FLUSH();
        }
        if (i == Number_of_command)
        {
            //NRF_LOG_INFO("nak");
            //NRF_LOG_INFO("Number_of_command: %d", Number_of_command);
            reslength = 3;
            err_code = ble_nus_data_send(&m_nus, "nak", &reslength, m_conn_handle);
        }

        if ((err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != NRF_ERROR_RESOURCES) &&
            (err_code != NRF_ERROR_NOT_FOUND))
        {
            APP_ERROR_CHECK(err_code);
        }
    }

}

/**@snippet [Handling the data received over BLE] */

/**@brief Function for simulating and sending one Temperature Measurement.
 */
static void temperature_measurement_send(void)
{
    ble_hts_meas_t simulated_meas;
    ret_code_t     err_code;

    if (!m_hts_meas_ind_conf_pending)
    {
        hts_measurement(&simulated_meas);

        err_code = ble_hts_measurement_send(&m_hts, &simulated_meas);

        switch (err_code)
        {
            case NRF_SUCCESS:
                // Measurement was successfully sent, wait for confirmation.
                m_hts_meas_ind_conf_pending = true;
                break;

            case NRF_ERROR_INVALID_STATE:
                // Ignore error.
                break;

            default:
                APP_ERROR_HANDLER(err_code);
                break;
        }
    }
}


/**@brief Function for handling the Health Thermometer Service events.
 *
 * @details This function will be called for all Health Thermometer Service events which are passed
 *          to the application.
 *
 * @param[in] p_hts  Health Thermometer Service structure.
 * @param[in] p_evt  Event received from the Health Thermometer Service.
 */
static void on_hts_evt(ble_hts_t * p_hts, ble_hts_evt_t * p_evt)
{
    temperature_measurement_send();
    /*
    switch (p_evt->evt_type)
    {
        case BLE_HTS_EVT_INDICATION_ENABLED:
            // Indication has been enabled, send a single temperature measurement
            temperature_measurement_send();
            break;

        case BLE_HTS_EVT_INDICATION_CONFIRMED:
            m_hts_meas_ind_conf_pending = false;
            break;

        default:
            // No implementation needed.
            break;
    }
    */
}






/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code;

    //err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    //APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);
    
    State_keeper = STATE_SLEEPING;
    /*
    int i;
    for(i = 0;i < 100000000;i++)
    {
      ;
    }

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
    */
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            //sleep_mode_enter();
            NRF_LOG_INFO("Reset Idle, Re-advertising.");
            err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
            APP_ERROR_CHECK(err_code); 
            break;
        default:
            break;
    }
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));
    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;
    
    init.srdata.uuids_complete.uuid_cnt = sizeof(m_sr_uuids) / sizeof(m_sr_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_sr_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}






/**@brief Function for initializing the sensor simulators.
 */
static void sensor_simulator_init(void)
{
    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    m_battery_sim_cfg.start_at_max = true;

    sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);

    // Temperature is in celcius (it is multiplied by 100 to avoid floating point arithmetic).
    m_temp_celcius_sim_cfg.min          = MIN_CELCIUS_DEGREES;
    m_temp_celcius_sim_cfg.max          = MAX_CELCIUS_DEGRESS;
    m_temp_celcius_sim_cfg.incr         = CELCIUS_DEGREES_INCREMENT;
    m_temp_celcius_sim_cfg.start_at_max = false;

    sensorsim_init(&m_temp_celcius_sim_state, &m_temp_celcius_sim_cfg);

    m_heart_rate_sim_cfg.min          = MIN_HEART_RATE;
    m_heart_rate_sim_cfg.max          = MAX_HEART_RATE;
    m_heart_rate_sim_cfg.incr         = HEART_RATE_INCREMENT;
    m_heart_rate_sim_cfg.start_at_max = false;

    sensorsim_init(&m_heart_rate_sim_state, &m_heart_rate_sim_cfg);

    m_rr_interval_sim_cfg.min          = MIN_RR_INTERVAL;
    m_rr_interval_sim_cfg.max          = MAX_RR_INTERVAL;
    m_rr_interval_sim_cfg.incr         = RR_INTERVAL_INCREMENT;
    m_rr_interval_sim_cfg.start_at_max = false;

    sensorsim_init(&m_rr_interval_sim_state, &m_rr_interval_sim_cfg);
}



/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    //cp_init.start_on_notify_cccd_handle    = m_hrs.hrm_handles.cccd_handle;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
/*
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    // ret_code_t err_code;
    // bool       is_indication_enabled;

    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
*/    /*
        case PM_EVT_CONN_SEC_SUCCEEDED:
            // Send a single temperature measurement if indication is enabled.
            // NOTE: For this to work, make sure ble_hts_on_ble_evt() is called before
            // pm_evt_handler() in ble_evt_dispatch().
            err_code = ble_hts_is_indication_enabled(&m_hts, &is_indication_enabled);
            APP_ERROR_CHECK(err_code);
            if (is_indication_enabled)
            {
                temperature_measurement_send();
            }
            break;
    */
/*        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            break;

        default:
            break;
    }
}
*/

/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    //err_code = pm_sec_params_set(&sec_param);
    //APP_ERROR_CHECK(err_code);

    //err_code = pm_register(pm_evt_handler);
    //APP_ERROR_CHECK(err_code);
}






void saadc_timer_handler(nrf_timer_event_t event_type, void * p_context)
{

}

void saadc_sampling_event_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, saadc_timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event every 1000ms */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, 1000);
    nrf_drv_timer_extended_compare(&m_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   //NRF_TIMER_CC_CHANNEL1,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer,
                                                                                NRF_TIMER_CC_CHANNEL0);
                                                                                //NRF_TIMER_CC_CHANNEL1);
    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
                                          timer_compare_event_addr,
                                          saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);
}



static uint8_t  m_adc_channel_enabled; 
static nrf_saadc_channel_config_t  channel_0_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);    //NRF_SAADC_INPUT_AIN0: P0.02, body temprature
static nrf_saadc_channel_config_t  channel_1_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);    //NRF_SAADC_INPUT_AIN1: P0.03, battery temprature
static nrf_saadc_channel_config_t  channel_5_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN5);    //NRF_SAADC_INPUT_AIN5: P0.29, battery voltage
const float Over_battery_temperature  = 45.00;
const float Over_warning_battery_temperature  = 42.00;
const float Under_battery_temperature =  0.00;

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    int ad_val;
    //float resolution = 1024.0; //1024 = 2^10, please check ad bit setting
    float resolution = 16384.0; //16384 = 2^14, please check ad bit setting
    float ad_voltage;
    float ad_resistance;
    float ad_resistance1;
    float vcc = 2.70;   // adc max 2.70v? vcc:3.00v
    //float resistance0 = 10000;   // R0, termista, 10k ohm (normal, 25deg) 
    float resistance0 = 6706.7;   // R0, termista, 10k ohm (normal, 36deg)
    float resistance1 = 6800.0;   // R1, split voltage resitance, 6.8k ohm
    float e = 2.7182818284; // Napier's constant
    float b = 3380.0; // B parameter termista value when 25 deg. = 3380
    //float standard_temp = 298.15;  // 25.0 deg + 273.15 absolute temp. [kelbin]
    float standard_temp = 309.15;  // 36.0 deg + 273.15 absolute temp. [kelbin]
    float temperature;
    float correction_term = 1292;
    float bat_voltage;
    float bat_percent;

    //maybe need nrf_log_flush()
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        static ret_code_t err_code;

        if (m_adc_channel_enabled == 0)   //NRF_SAADC_INPUT_AIN0: P0.02, body temprature
        {
            err_code = nrf_drv_saadc_channel_uninit(0);
            APP_ERROR_CHECK(err_code);
            err_code = nrf_drv_saadc_channel_init(1, &channel_1_config);
            APP_ERROR_CHECK(err_code);

            err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
            APP_ERROR_CHECK(err_code);

            if(Debug_output_body_temperature == true)
            {
                NRF_LOG_INFO("Channel %d value: %d", m_adc_channel_enabled, p_event->data.done.p_buffer[0]);
            }

            ad_val = (int)p_event->data.done.p_buffer[0];
            ad_voltage = (float)(ad_val + correction_term) / resolution * vcc;
            ad_resistance = (resistance1 * ad_voltage) / (vcc - ad_voltage);
            temperature = b/(logf(ad_resistance/resistance0) + (b/standard_temp)) - 273.15;
            
            if(Debug_output_body_temperature == true)
            {
                NRF_LOG_RAW_INFO("%d," NRF_LOG_FLOAT_MARKER ",", ad_val, NRF_LOG_FLOAT(temperature));
                NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(ad_voltage));
            }
            Body_temperature = temperature;

            m_adc_channel_enabled = 1;
        }
        else if (m_adc_channel_enabled == 1)    //NRF_SAADC_INPUT_AIN1: P0.03, battery temprature
        {
            err_code = nrf_drv_saadc_channel_uninit(1);
            APP_ERROR_CHECK(err_code);
            err_code = nrf_drv_saadc_channel_init(5, &channel_5_config);
            APP_ERROR_CHECK(err_code);

            err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
            APP_ERROR_CHECK(err_code);
            
            if(Debug_output_battery_temperature == true)
            {
                NRF_LOG_INFO("Channel %d value: %d", m_adc_channel_enabled, p_event->data.done.p_buffer[0]);
            }

            ad_val = (int)p_event->data.done.p_buffer[0];
            ad_voltage = (float)(ad_val + correction_term) / resolution * vcc;
            ad_resistance = (resistance1 * ad_voltage) / (vcc - ad_voltage);
            temperature = b/(logf(ad_resistance/resistance0) + (b/standard_temp)) - 273.15;
            
            if(Debug_output_battery_temperature == true)
            {
                NRF_LOG_RAW_INFO("%d," NRF_LOG_FLOAT_MARKER ",", ad_val, NRF_LOG_FLOAT(temperature));
                NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(ad_voltage));
            }
            Battery_temperature = temperature;

            if (Battery_temperature > Over_battery_temperature && State_emergency_lock == true)
            {
                //over temprature and sleep mode enter
                NRF_LOG_INFO("sleep mode enter, over temprature");

                err_code = app_timer_stop(m_data_record_timer_id);
                APP_ERROR_CHECK(err_code);
                NRF_LOG_INFO("10 second measure and 10 minutes record stop");
                if (State_keeper == STATE_MEASURING)
                {
                    State_keeper = STATE_PAIRING;
                    NRF_LOG_INFO("State_keeper: %d", State_keeper);
                }

                NRF_LOG_FLUSH();
                sleep_mode_enter();
            }

            if (Battery_temperature > Over_warning_battery_temperature || Battery_temperature < Under_battery_temperature)
            {
                State_keeper = STATE_EMERGENCY;
                State_emergency_lock = true;
            }
            else
            {
                State_emergency_lock = false; 
            }

            m_adc_channel_enabled = 5;
        }
        else if (m_adc_channel_enabled == 5)    //NRF_SAADC_INPUT_AIN5: P0.29, battery voltage
        {
            err_code = nrf_drv_saadc_channel_uninit(5);
            APP_ERROR_CHECK(err_code);
            err_code = nrf_drv_saadc_channel_init(0, &channel_0_config);
            APP_ERROR_CHECK(err_code);

            err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
            APP_ERROR_CHECK(err_code);

            if(Debug_output_battery_voltage == true)
            {
                NRF_LOG_INFO("Channel %d value: %d", m_adc_channel_enabled, p_event->data.done.p_buffer[0]);
            }

            ad_val = (int)p_event->data.done.p_buffer[0];
            ad_voltage = (float)(ad_val) / resolution * vcc;
            bat_voltage = ad_voltage * 2.03;    // 2.03 = 3.98V / 1.96V, measurement, ratio of resistance
            
            if (bat_voltage > 4.10)
            {
                bat_percent = 100;
            }
            else if (bat_voltage < 3.40)
            {
                bat_percent = 1;
            }
            else
            {
                bat_percent = (bat_voltage - 3.40) / 0.70 * 100;    // 0.70 = 4.10 - 3.40, max 4.2 V - min 3.3 V, I found that battery stop 4.05V
            }
            
            if (bat_voltage < 3.25)
            {
                //low battery and sleep mode enter
                NRF_LOG_INFO("sleep mode enter, low battery");

                err_code = app_timer_stop(m_data_record_timer_id);
                APP_ERROR_CHECK(err_code);
                NRF_LOG_INFO("10 second measure and 10 minutes record stop");
                if (State_keeper == STATE_MEASURING)
                {
                    State_keeper = STATE_PAIRING;
                    NRF_LOG_INFO("State_keeper: %d", State_keeper);
                }

                NRF_LOG_FLUSH();
                sleep_mode_enter();
            }

            if(Debug_output_battery_voltage == true)
            {
                NRF_LOG_RAW_INFO("%d," NRF_LOG_FLOAT_MARKER ",", ad_val, NRF_LOG_FLOAT(bat_percent));
                NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(ad_voltage));
            }
            Battery_percent = bat_percent;

            m_adc_channel_enabled = 0;
        }
    }
}

void saadc_init(void)
{
    ret_code_t err_code;
    /*
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);    //NRF_SAADC_INPUT_AIN0: P0.02 on nRF52DK board
    */
    //nrf_saadc_channel_config_t channel_config;
    //channel_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0); 

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_0_config);
    APP_ERROR_CHECK(err_code);
    m_adc_channel_enabled = 0;

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    //err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    //APP_ERROR_CHECK(err_code);
}



void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);

    APP_ERROR_CHECK(err_code);
}



/**
 * @brief TWI initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;
    int32_t ret = 0;

    const nrf_drv_twi_config_t twi_bh1792glc_config = {
       .scl                = BH1792GLC_SCL_PIN,
       .sda                = BH1792GLC_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    NRF_LOG_INFO("before nrf_drv_twi_init.");
    err_code = nrf_drv_twi_init(&m_twi, &twi_bh1792glc_config, twi_handler, NULL);
    NRF_LOG_INFO("finished nrf_drv_twi_init.");
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);

    // BH1792
    m_bh1792.fnWrite      = i2c_write;
    m_bh1792.fnRead       = i2c_read;
    ret = bh1792_Reg_Init(&m_bh1792);
    NRF_LOG_INFO("finished bh1792_Reg_Init.");
    //error_check(ret, "bh1792_Reg_Init");

    ret = hr_bh1792_Init();
    NRF_LOG_INFO("finished hr_bh1792_Init.");
    //error_check(ret, "hr_bh1792_Init");

    m_bh1792.prm.sel_adc  = BH1792_PRM_SEL_ADC_GREEN;
    m_bh1792.prm.msr      = BH1792_PRM_MSR_SINGLE;//BH1792_PRM_MSR_1024HZ;
    m_bh1792.prm.led_en   = (BH1792_PRM_LED_EN1_0 << 1) | BH1792_PRM_LED_EN2_0;
    m_bh1792.prm.led_cur1 = BH1792_PRM_LED_CUR1_MA(0);
    m_bh1792.prm.led_cur2 = BH1792_PRM_LED_CUR2_MA(0);
    m_bh1792.prm.ir_th    = 0xFFFC;
    m_bh1792.prm.int_sel  = BH1792_PRM_INT_SEL_SGL;//BH1792_PRM_INT_SEL_WTM;
    NRF_LOG_INFO("before bh1792_SetParams.");
    ret = bh1792_SetParams();
    //error_check(ret, "bh1792_SetParams");
    NRF_LOG_INFO("finished bh1792_SetParams.");

    //NRF_LOG_INFO("GDATA(@LED_ON),GDATA(@LED_OFF)\n");

    //ret = bh1792_StartMeasure();
    //error_check(ret, "bh1792_StartMeasure");
    //NRF_LOG_INFO("finished bh1792_StartMeasure.");

    ret = hr_bh1792_StartMeasure();
    //error_check(ret, "hr_bh1792_StartMeasure");
    NRF_LOG_INFO("finished Hr_bh1792_StartMeasure.");

    //ret = bh1792_StopMeasure();
    //error_check(ret, "bh1792_StopMeasure");
    //NRF_LOG_INFO("finished bh1792_StopMeasure.");
}



void bh1792_isr(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    int32_t ret = 0;
    //uint8_t i   = 0;
    u16_pair_t s_pwData_test;
    float32_t pw_test;
    //static uint8_t  bpm     = 0U;
    //static uint8_t  wearing = 0U;
    static uint8_t    s_cnt_freq = 0;

    nrf_drv_gpiote_in_event_disable(BH1792GLC_INT_PIN);

    //ret = bh1792_GetMeasData(&m_bh1792_dat);
    //error_check(ret, "bh1792_GetMeasData");
    //ret = hr_bh1792_Calc(s_cnt_freq);
    //s_pwData_test = m_bh1792_dat.green;
    ret = hr_bh1792_Calc(s_cnt_freq, &m_bh1792_dat, &s_pwData_test, &pw_test);
    s_cnt_freq++;
    if (s_cnt_freq >= 31)
    {
        s_cnt_freq = 0;
        hr_bh1792_GetData(&Bpm, &Wearing);
        //NRF_LOG_RAW_INFO("%d, %d\n", bpm, wearing);
        NRF_LOG_RAW_INFO("%d, %d, %d, %d, ", Bpm, Wearing, s_pwData_test.on, s_pwData_test.off);
        NRF_LOG_RAW_INFO("" NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(pw_test));
    }
    //NRF_LOG_RAW_INFO("%d, %d, %d, %d, ", bpm, wearing, s_pwData_test.on, s_pwData_test.off);
    //NRF_LOG_RAW_INFO("" NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(pw_test));

    //ret = hr_bh1792_Calc(s_cnt_freq, &s_pwData_test, &pw_test);
    //error_check(ret, "hr_bh1792_Calc");

    nrf_drv_gpiote_in_event_enable(BH1792GLC_INT_PIN, true);
}

// Note:  I2C access should be completed within 0.5ms
int32_t i2c_write(uint8_t slv_adr, uint8_t reg_adr, uint8_t *reg, uint8_t reg_size)
{
    ret_code_t err_code;
    uint32_t timeout = BH1792_TWI_TIMEOUT;

    twi_tx_buffer[0] = reg_adr;
    memcpy(&twi_tx_buffer[1], &reg[0], reg_size);
    
    err_code = nrf_drv_twi_tx(&m_twi, slv_adr, &twi_tx_buffer[0], reg_size + 1, false);
    if(err_code != NRF_SUCCESS) return err_code;
    while((!twi_tx_done) && --timeout) ;
    if(!timeout) return NRF_ERROR_TIMEOUT;
    twi_tx_done = false;

    //return rc;   //rc is return value that arduino, Wire endTransmission, rc:0 is normal
    return 0;
}

// Note:  I2C access should be completed within 0.5ms
int32_t i2c_read(uint8_t slv_adr, uint8_t reg_adr, uint8_t *reg, uint8_t reg_size)
{
    ret_code_t err_code;
    uint32_t timeout = BH1792_TWI_TIMEOUT;

    err_code = nrf_drv_twi_tx(&m_twi, slv_adr, &reg_adr, 1, false);
    if(err_code != NRF_SUCCESS) return err_code;

    while((!twi_tx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    twi_tx_done = false;

    err_code = nrf_drv_twi_rx(&m_twi, slv_adr, reg, reg_size);
    if(err_code != NRF_SUCCESS) return err_code;

    timeout = BH1792_TWI_TIMEOUT;
    while((!twi_rx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    twi_rx_done = false;

    //return rc;  //rc:0 is normal, rc:4 is error. but in nrf5 when case of error, already return
    return 0;
}

/*
void error_check(int32_t ret, String msg)
{
  if(ret < 0) {
    msg = "Error: " + msg;
    msg += " function";
    NRF_LOG_INFO("%s\n", msg);
    NRF_LOG_INFO("ret = ");
    NRF_LOG_INFO("%d", ret);
    if(ret == BH1792_I2C_ERR) {
      NRF_LOG_INFO("i2c_ret = ");
      NRF_LOG_INFO("%d\n", m_bh1792.i2c_err);
    }
    while(1);
  }
}
*/






/** @brief: Function for handling the RTC0 interrupts.
 * Triggered on TICK and COMPARE0 match.
 */
static volatile int Tick_count = 0;
static volatile int Enter_sleep_count = 0;
static volatile int Boot_count = 0;
static volatile int Wait_sleep_count = 0;

void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage);

const uint8_t write_count = 4;
const uint32_t Flash_start_address = 0x60000;
uint32_t write_index = Flash_start_address;
//static uint8_t flash_ff_padding[4096] = {0xFF};
__ALIGN(4) uint8_t flash_ff_padding[0x1000];

const int size_write_data = sizeof(data_hr_hr[0]);
const int size_write_buffer = sizeof(data_hr_hr[0]) * 8;
__ALIGN(4) uint8_t write_buffer[sizeof(data_hr_hr[0]) * 8];
const int Flash_write_count_for_sleep = Num_of_data_hr_hr / 8;

static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    ret_code_t rc;
    if (int_type == NRF_DRV_RTC_INT_TICK)
    {
        if (Tick_count == 4 || Tick_count == 8)
        {
            m_xfer_done = true;
        }


        /*Blink/light up LED display state, State_keeper.
        // Blink GREEN: Advertising that Before Pairing, 0
        // GREEN: Pairing and Idle state, 1
        // RED: Measring state and Pairing, 2
        // Blink GREEN and RED: Emergency, 3
        // Blink RED: Measuring and Adverting, 4
        // BLUE: boot, reboot from sleep, 5
        // Blink BLUE: go to sleep, 6
        */

        if (State_emergency_lock == true)
        {
            // State_keeper == STATE_EMERGENCY
            // Blink GREEN and RED: Emergency, 3
            if (Tick_count % 2 == 0)
            {
                nrf_drv_gpiote_out_set(LED_3_COLOR_RED_PIN);
                nrf_drv_gpiote_out_clear(LED_3_COLOR_GREEN_PIN);
                nrf_drv_gpiote_out_set(LED_3_COLOR_BLUE_PIN);
            }
            else
            {
                nrf_drv_gpiote_out_clear(LED_3_COLOR_RED_PIN);
                nrf_drv_gpiote_out_set(LED_3_COLOR_GREEN_PIN);
                nrf_drv_gpiote_out_set(LED_3_COLOR_BLUE_PIN);
            }
        }
        else if (Boot_count < 24)
        {
            // BLUE: boot, reboot from sleep, 5
            nrf_drv_gpiote_out_set(LED_3_COLOR_RED_PIN);
            nrf_drv_gpiote_out_set(LED_3_COLOR_GREEN_PIN);
            nrf_drv_gpiote_out_clear(LED_3_COLOR_BLUE_PIN);

            if(Boot_count == 4)
            {
                memset(&flash_ff_padding[0], 0xFF, sizeof(flash_ff_padding));
                NRF_LOG_INFO("ff[0x0000]: 0x%x", flash_ff_padding[0x0000]);
                NRF_LOG_INFO("ff[0x0500]: 0x%x ", flash_ff_padding[0x0500]);
                NRF_LOG_INFO("ff[0x0999]: 0x%x ", flash_ff_padding[0x0999]);
                //NRF_LOG_INFO("ff[0000]: 0x%x", flash_ff_padding[0]);
                //NRF_LOG_INFO("ff[2016]: 0x%x ", flash_ff_padding[2016]);
                //NRF_LOG_INFO("ff[4095]: 0x%x ", flash_ff_padding[4095]);
            }
            else if (Boot_count == 5)
            {
                rc = nrf_fstorage_write(&fstorage, write_index, flash_ff_padding, sizeof(flash_ff_padding), NULL);
                APP_ERROR_CHECK(rc);
            }
            else if (Boot_count == 9)
            {
                rc = nrf_fstorage_write(&fstorage, write_index + 0x1000, flash_ff_padding, sizeof(flash_ff_padding), NULL);
                APP_ERROR_CHECK(rc);
            }
            else if (Boot_count == 13)
            {
                rc = nrf_fstorage_write(&fstorage, write_index + 0x2000, flash_ff_padding, sizeof(flash_ff_padding), NULL);
                APP_ERROR_CHECK(rc);
            }
            //Boot_count++;
        }
        else if (State_keeper == STATE_SLEEPING)
        {
            // Blink BLUE: go to sleep, 6
            if (Tick_count % 2 == 0)
            {
                nrf_drv_gpiote_out_set(LED_3_COLOR_RED_PIN);
                nrf_drv_gpiote_out_set(LED_3_COLOR_GREEN_PIN);
                nrf_drv_gpiote_out_toggle(LED_3_COLOR_BLUE_PIN);
            }
            else
            {
                nrf_drv_gpiote_out_set(LED_3_COLOR_RED_PIN);
                nrf_drv_gpiote_out_set(LED_3_COLOR_GREEN_PIN);
                nrf_drv_gpiote_out_set(LED_3_COLOR_BLUE_PIN);
            }
        }
        else if (LED_output_state)
        {
            switch (State_keeper)
            {
                case 0: //STATE_ADVERTISING:
                    // Blink GREEN: Advertising that Before Pairing, 0
                    if (Tick_count == 0)
                    {
                        nrf_drv_gpiote_out_set(LED_3_COLOR_RED_PIN);
                        nrf_drv_gpiote_out_clear(LED_3_COLOR_GREEN_PIN);
                        nrf_drv_gpiote_out_set(LED_3_COLOR_BLUE_PIN);
                        }
                    else
                    {
                        nrf_drv_gpiote_out_set(LED_3_COLOR_RED_PIN);
                        nrf_drv_gpiote_out_set(LED_3_COLOR_GREEN_PIN);
                        nrf_drv_gpiote_out_set(LED_3_COLOR_BLUE_PIN);               
                    }
                    break;
                case 1: //STATE_PAIRING:
                    // GREEN: Pairing and Idle state, 1
                    nrf_drv_gpiote_out_set(LED_3_COLOR_RED_PIN);
                    nrf_drv_gpiote_out_clear(LED_3_COLOR_GREEN_PIN);
                    nrf_drv_gpiote_out_set(LED_3_COLOR_BLUE_PIN);
                    break;
                case 2: //STATE_MEASURING (and STATE_PAIRING):
                    // RED: Measring state and Pairing, 2
                    nrf_drv_gpiote_out_clear(LED_3_COLOR_RED_PIN);
                    nrf_drv_gpiote_out_set(LED_3_COLOR_GREEN_PIN);
                    nrf_drv_gpiote_out_set(LED_3_COLOR_BLUE_PIN);
                    break;
                case 3: //STATE_EMERGENCY:
                    // Blink GREEN and RED: Emergency, 3
                    if (Tick_count % 2 == 0)
                    {
                        nrf_drv_gpiote_out_set(LED_3_COLOR_RED_PIN);
                        nrf_drv_gpiote_out_clear(LED_3_COLOR_GREEN_PIN);
                        nrf_drv_gpiote_out_set(LED_3_COLOR_BLUE_PIN);
                    }
                    else
                    {
                        nrf_drv_gpiote_out_clear(LED_3_COLOR_RED_PIN);
                        nrf_drv_gpiote_out_set(LED_3_COLOR_GREEN_PIN);
                        nrf_drv_gpiote_out_set(LED_3_COLOR_BLUE_PIN);
                    }
                    break;
                case 4: //STATE_
                    // Blink RED: Measuring and Adverting, 4
                    if (Tick_count == 0)
                    {
                        nrf_drv_gpiote_out_clear(LED_3_COLOR_RED_PIN);
                        nrf_drv_gpiote_out_set(LED_3_COLOR_GREEN_PIN);
                        nrf_drv_gpiote_out_set(LED_3_COLOR_BLUE_PIN);
                        }
                    else
                    {
                        nrf_drv_gpiote_out_set(LED_3_COLOR_RED_PIN);
                        nrf_drv_gpiote_out_set(LED_3_COLOR_GREEN_PIN);
                        nrf_drv_gpiote_out_set(LED_3_COLOR_BLUE_PIN);               
                    }
                    break;
                case 5:
                    // BLUE: boot, reboot from sleep, 5
                    nrf_drv_gpiote_out_set(LED_3_COLOR_RED_PIN);
                    nrf_drv_gpiote_out_set(LED_3_COLOR_GREEN_PIN);
                    nrf_drv_gpiote_out_clear(LED_3_COLOR_BLUE_PIN);
                    break;
                case 6:
                    // Blink BLUE: go to sleep, 6
                    nrf_drv_gpiote_out_set(LED_3_COLOR_RED_PIN);
                    nrf_drv_gpiote_out_set(LED_3_COLOR_GREEN_PIN);
                    nrf_drv_gpiote_out_toggle(LED_3_COLOR_BLUE_PIN);
                    break;
                default:
                    nrf_drv_gpiote_out_set(LED_3_COLOR_RED_PIN);
                    nrf_drv_gpiote_out_set(LED_3_COLOR_GREEN_PIN);
                    nrf_drv_gpiote_out_set(LED_3_COLOR_BLUE_PIN);
            }
        }
        else
        {
            // when finish boot count, LED_output_state = false; not state_emergency and not state_sleeping
            nrf_drv_gpiote_out_set(LED_3_COLOR_RED_PIN);
            nrf_drv_gpiote_out_set(LED_3_COLOR_GREEN_PIN);
            nrf_drv_gpiote_out_set(LED_3_COLOR_BLUE_PIN);
        }

        if (nrf_gpio_pin_read(SWITCH1_PIN))
        {
            //NRF_LOG_INFO("swith1 true");
            Enter_sleep_count = 0;
        }
        else
        {
            //NRF_LOG_INFO("switch1 false");
            Enter_sleep_count++;
            if (Enter_sleep_count == 24)
            {
                //3s long push and sleep mode enter
                NRF_LOG_INFO("sleep mode enter, user operation");

                rc = app_timer_stop(m_data_record_timer_id);
                APP_ERROR_CHECK(rc);
                NRF_LOG_INFO("10 second measure and 10 minutes record stop");
                if (State_keeper == STATE_MEASURING)
                {
                    State_keeper = STATE_PAIRING;
                    NRF_LOG_INFO("State_keeper: %d", State_keeper);
                }

                NRF_LOG_FLUSH();
                sleep_mode_enter();
            }
        }

        if (State_keeper == STATE_SLEEPING)
        {
            if (Wait_sleep_count < Flash_write_count_for_sleep)
            {
                /*
                // for debug begin
                if (Wait_sleep_count > 0)
                {
                    int check_index = Wait_sleep_count - 1;
                    uint8_t check_read_data[sizeof(data_hr_hr[check_index])];
                    int check_read_address = write_index - sizeof(data_hr_hr[check_index]);
                    rc = nrf_fstorage_read(&fstorage, check_read_address, check_read_data, sizeof(check_read_data));
                    if (rc != NRF_SUCCESS)
                    {
                        NRF_LOG_INFO("nrf_fstorage_read() returned: %s\n", nrf_strerror_get(rc));
                    }
                    ble_data_ht_hr_t check_data_hr_hr = *(ble_data_ht_hr_t *)(check_read_data);
                    if (check_data_hr_hr.start_time.year != data_hr_hr[check_index].start_time.year) { NRF_LOG_INFO("Error write failed"); }
                    if (check_data_hr_hr.start_time.month != data_hr_hr[check_index].start_time.month) { NRF_LOG_INFO("Error write failed"); }
                    if (check_data_hr_hr.start_time.day != data_hr_hr[check_index].start_time.day) { NRF_LOG_INFO("Error write failed"); }
                    if (check_data_hr_hr.start_time.hours != data_hr_hr[check_index].start_time.hours) { NRF_LOG_INFO("Error write failed"); }
                    if (check_data_hr_hr.start_time.minutes != data_hr_hr[check_index].start_time.minutes) { NRF_LOG_INFO("Error write failed"); }
                    if (check_data_hr_hr.start_time.seconds != data_hr_hr[check_index].start_time.seconds) { NRF_LOG_INFO("Error write failed"); }
                    if (check_data_hr_hr.heart_rate != data_hr_hr[check_index].heart_rate) { NRF_LOG_INFO("Error write failed"); }
                    if (check_data_hr_hr.body_temperature_array[0] != data_hr_hr[check_index].body_temperature_array[0]) { NRF_LOG_INFO("Error write failed"); }
                    if (check_data_hr_hr.body_temperature_array[1] != data_hr_hr[check_index].body_temperature_array[1]) { NRF_LOG_INFO("Error write failed"); }
                    if (check_data_hr_hr.body_temperature_array[2] != data_hr_hr[check_index].body_temperature_array[2]) { NRF_LOG_INFO("Error write failed"); }
                    if (check_data_hr_hr.body_temperature_array[3] != data_hr_hr[check_index].body_temperature_array[3]) { NRF_LOG_INFO("Error write failed"); }
                    if (check_data_hr_hr.body_temperature_array[4] != data_hr_hr[check_index].body_temperature_array[4]) { NRF_LOG_INFO("Error write failed"); }
                    if (check_data_hr_hr.body_temperature_array[5] != data_hr_hr[check_index].body_temperature_array[5]) { NRF_LOG_INFO("Error write failed"); }
                    NRF_LOG_INFO("Check read done, 0x%x", check_read_address);
                    NRF_LOG_FLUSH();
                }
                // for debug end
                */
                
                //while (nrf_fstorage_is_busy(p_fstorage))
                if (nrf_fstorage_is_busy(&fstorage) == false)
                {
                    NRF_LOG_INFO("write data to sleep: %d, %d", Wait_sleep_count, size_write_data);
                    memcpy(&write_buffer[size_write_data * 0], (const void *)(&data_hr_hr[Wait_sleep_count * 8 + 0]), size_write_data);
                    memcpy(&write_buffer[size_write_data * 1], (const void *)(&data_hr_hr[Wait_sleep_count * 8 + 1]), size_write_data);
                    memcpy(&write_buffer[size_write_data * 2], (const void *)(&data_hr_hr[Wait_sleep_count * 8 + 2]), size_write_data);
                    memcpy(&write_buffer[size_write_data * 3], (const void *)(&data_hr_hr[Wait_sleep_count * 8 + 3]), size_write_data);
                    memcpy(&write_buffer[size_write_data * 4], (const void *)(&data_hr_hr[Wait_sleep_count * 8 + 4]), size_write_data);
                    memcpy(&write_buffer[size_write_data * 5], (const void *)(&data_hr_hr[Wait_sleep_count * 8 + 5]), size_write_data);
                    memcpy(&write_buffer[size_write_data * 6], (const void *)(&data_hr_hr[Wait_sleep_count * 8 + 6]), size_write_data);
                    memcpy(&write_buffer[size_write_data * 7], (const void *)(&data_hr_hr[Wait_sleep_count * 8 + 7]), size_write_data);
                    rc = nrf_fstorage_write(&fstorage, write_index, write_buffer, sizeof(write_buffer), NULL);
                    APP_ERROR_CHECK(rc);
                    write_index = write_index + sizeof(write_buffer);

                    NRF_LOG_INFO("Done, %d", Wait_sleep_count);
                    NRF_LOG_FLUSH();
                    Wait_sleep_count++;
                }
            }
            else if (Wait_sleep_count == Flash_write_count_for_sleep)
            {
                if (nrf_fstorage_is_busy(&fstorage) == false)
                {
                    //Current time, time_stamp
                    __ALIGN(4) static uint8_t write_time_stamp[sizeof(time_stamp)];
                    *(ble_date_time_t *) write_time_stamp = time_stamp;
                    rc = nrf_fstorage_write(&fstorage, write_index, write_time_stamp, sizeof(write_time_stamp), NULL);
                    APP_ERROR_CHECK(rc);
                    write_index = write_index + sizeof(write_time_stamp);

                    NRF_LOG_INFO("Done, %d", Wait_sleep_count);
                    NRF_LOG_FLUSH();
                    Wait_sleep_count++;
                }
            }
            else if (Wait_sleep_count == Flash_write_count_for_sleep + 1)
            {
                if (nrf_fstorage_is_busy(&fstorage) == false)
                {
                    //Write index, Write_index_data_hr_hr
                    __ALIGN(4) static uint8_t write_write_index[sizeof(Write_index_data_hr_hr)];
                    *(unsigned int *) write_write_index = Write_index_data_hr_hr;
                    rc = nrf_fstorage_write(&fstorage, write_index, write_write_index, sizeof(write_write_index), NULL);
                    APP_ERROR_CHECK(rc);
                    write_index = write_index + sizeof(write_write_index);

                    NRF_LOG_INFO("Done, %d", Wait_sleep_count);
                    NRF_LOG_FLUSH();
                    Wait_sleep_count++;
                }
            }
            else if (Wait_sleep_count == Flash_write_count_for_sleep + 2)
            {
                if (nrf_fstorage_is_busy(&fstorage) == false)
                {
                    //Read index, Read_index_data_hr_hr
                    __ALIGN(4) static uint8_t write_read_index[sizeof(Read_index_data_hr_hr)];
                    *(unsigned int *) write_read_index = Read_index_data_hr_hr;
                    rc = nrf_fstorage_write(&fstorage, write_index, write_read_index, sizeof(write_read_index), NULL);
                    APP_ERROR_CHECK(rc);
                    write_index = write_index + sizeof(write_read_index);

                    NRF_LOG_INFO("Done, %d", Wait_sleep_count);
                    NRF_LOG_FLUSH();
                    Wait_sleep_count++;
                }
            }
            else if (Wait_sleep_count == Flash_write_count_for_sleep + 3)
            {
                if (nrf_fstorage_is_busy(&fstorage) == false)
                {
                    //Count index, Count_index_data_hr_hr
                    __ALIGN(4) static uint8_t write_count_index[sizeof(Count_index_data_hr_hr)];
                    *(unsigned int *) write_count_index = Count_index_data_hr_hr;
                    rc = nrf_fstorage_write(&fstorage, write_index, write_count_index, sizeof(write_count_index), NULL);
                    APP_ERROR_CHECK(rc);
                    write_index = write_index + sizeof(write_count_index);

                    NRF_LOG_INFO("Done, %d", Wait_sleep_count);
                    NRF_LOG_FLUSH();
                    Wait_sleep_count++;
                }
            }
            else if (Wait_sleep_count == Flash_write_count_for_sleep + 4)
            {
                if (nrf_fstorage_is_busy(&fstorage) == false)
                {
                    //nrf_drv_gpiote_out_set(LED_3_COLOR_RED_PIN);
                    //nrf_drv_gpiote_out_set(LED_3_COLOR_GREEN_PIN);
                    //nrf_drv_gpiote_out_set(LED_3_COLOR_BLUE_PIN);
                    if (nrf_gpio_pin_read(SWITCH1_PIN))
                    {
                        // released button check 
                        Wait_sleep_count++;
                    }
                }
            }
            else if (Wait_sleep_count > Flash_write_count_for_sleep + 4)
            {
                //Wait_sleep_count = 0;
                // Go to system-off mode (this function will not return; wakeup will cause a reset).
                nrf_drv_gpiote_out_set(LED_3_COLOR_RED_PIN);
                nrf_drv_gpiote_out_set(LED_3_COLOR_GREEN_PIN);
                nrf_drv_gpiote_out_set(LED_3_COLOR_BLUE_PIN);
                NRF_LOG_INFO("system-off and sleep");
                NRF_LOG_FLUSH();
                sd_power_system_off();
                //rc = sd_power_system_off();
                //APP_ERROR_CHECK(rc);
            }

            //Wait_sleep_count++;
        }

        if (Boot_count < 24)
        {
            Boot_count++;
        }

        Tick_count++;
        if (Tick_count > 9)
        {
            Tick_count = 0;
        }
    }
}

const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2); /**< Declaring an instance of nrf_drv_rtc for RTC2. */
/** @brief Function initialization and configuration of RTC driver instance.
 */
static void rtc_config(void)
{
    uint32_t err_code;

    //Initialize RTC instance
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = 4095;
    err_code = nrf_drv_rtc_init(&rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    //Enable tick event & interrupt
    nrf_drv_rtc_tick_enable(&rtc, true);

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc);
}



/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    ret_code_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_heart_rate_timer_id, HEART_RATE_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_rr_interval_timer_id, RR_INTERVAL_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_temperature_timer_id, TEMPERATURE_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_sensor_contact_timer_id, SENSOR_CONTACT_DETECTED_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

#ifndef TWI_DISABLE
    //err_code = app_timer_start(m_bh1792glc_timer_id, BH1792GLC_MEAS_INTERVAL, NULL);
    //APP_ERROR_CHECK(err_code);
#endif

    //err_code = app_timer_start(m_data_record_timer_id, DATA_RECORD_MEAS_INTERVAL, NULL);
    //APP_ERROR_CHECK(err_code);
}



/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        uint32_t err_code;
        err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }
}



/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    //UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    //nrf_pwr_mgmt_run();
    /*
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
    */
    do
    {
        //__WFE();
        nrf_pwr_mgmt_run();
    }while (m_xfer_done == false);
    NRF_LOG_FLUSH();
    m_xfer_done = false;
}



static void print_flash_info(nrf_fstorage_t * p_fstorage)
{
    NRF_LOG_INFO("========| flash info |========");
    NRF_LOG_INFO("erase unit: \t%d bytes",      p_fstorage->p_flash_info->erase_unit);
    NRF_LOG_INFO("program unit: \t%d bytes",    p_fstorage->p_flash_info->program_unit);
    NRF_LOG_INFO("==============================");
}

void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage)
{
    /* While fstorage is busy, sleep and wait for an event. */
    while (nrf_fstorage_is_busy(p_fstorage))
    {
        //power_manage();
        //sd_app_evt_wait();
        nrf_pwr_mgmt_run();
    }
}






/**@brief Application main function.
 */
int main(void)
  {
    bool erase_bonds;

    // Initialize.
    State_keeper = STATE_BOOTING;
    Boot_count = 0;
    log_init();
    uart_init();
    NRF_LOG_INFO("Finish uart init, log init");
    lfclk_request();
    gpio_init();
    NRF_LOG_INFO("Finish gpio init");;
    timers_init();
    NRF_LOG_INFO("Finish timers init");    
    buttons_leds_init(&erase_bonds);
    NRF_LOG_INFO("Finish buttons leds init");
    power_management_init();
    NRF_LOG_INFO("Finish power management init");
    ble_stack_init();
    NRF_LOG_INFO("Finish ble stack init");
    gap_params_init();
    NRF_LOG_INFO("Finish gap params init");
    gatt_init();
    NRF_LOG_INFO("Finish gatt init");
    services_init();
    NRF_LOG_INFO("Finish services init");
    advertising_init();
    NRF_LOG_INFO("Finish advertising init");
    sensor_simulator_init();
    NRF_LOG_INFO("Finish sensor simulator init");
    conn_params_init();
    NRF_LOG_INFO("Finish conn params init");
    peer_manager_init();
    NRF_LOG_INFO("Finish saadc_init init");
    saadc_sampling_event_init();
    NRF_LOG_INFO("Finish peer manager init");
    saadc_init();
    NRF_LOG_INFO("Finish saadc_sampling_event_init init");
    saadc_sampling_event_enable();
    NRF_LOG_INFO("SAADC HAL simple example started.");

#ifndef TWI_DISABLE
    NRF_LOG_INFO("TWI sensor example started.");
    NRF_LOG_FLUSH();
    twi_init();
    NRF_LOG_INFO("finished twi init.");
#endif

    rtc_config();
    NRF_LOG_INFO("finished rtc config.")

    ret_code_t rc;
    NRF_LOG_INFO("fstorage example started.");
    nrf_fstorage_api_t * p_fs_api;
    NRF_LOG_INFO("SoftDevice is present.");
    NRF_LOG_INFO("Initializing nrf_fstorage_sd implementation...");
    /* Initialize an fstorage instance using the nrf_fstorage_sd backend.
     * nrf_fstorage_sd uses the SoftDevice to write to flash. This implementation can safely be
     * used whenever there is a SoftDevice, regardless of its status (enabled/disabled). */
    p_fs_api = &nrf_fstorage_sd;
    rc = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
    APP_ERROR_CHECK(rc);
    print_flash_info(&fstorage);
    /* It is possible to set the start and end addresses of an fstorage instance at runtime.
     * They can be set multiple times, should it be needed. The helper function below can
     * be used to determine the last address on the last page of flash memory available to
     * store data. */
    (void) nrf5_flash_end_addr_get();

    /* Let's write to flash. */
    /*
    uint32_t test_write_address = 0x60000;
    NRF_LOG_INFO("Writing \"%x\" to flash.", m_data);
    rc = nrf_fstorage_write(&fstorage, test_write_address, m_data, sizeof(m_data), NULL);
    APP_ERROR_CHECK(rc);

    wait_for_flash_ready(&fstorage);
    NRF_LOG_INFO("Done.");

    uint8_t    data[256] = {0};

    rc = nrf_fstorage_read(&fstorage, test_write_address, data, 8);
    if (rc != NRF_SUCCESS)
    {
        NRF_LOG_INFO("nrf_fstorage_read() returned: %s\n", nrf_strerror_get(rc));
    }

    for (uint32_t i = 0; i < 8; i++)
    {
        NRF_LOG_INFO("0x%x ", data[i]);
    }
    */

    uint32_t i = 0;
    uint32_t read_address = Flash_start_address;
    static uint8_t read_data[sizeof(data_hr_hr[0])];

    for (i = 0; i <  Num_of_data_hr_hr; i++)
    {
        rc = nrf_fstorage_read(&fstorage, read_address, read_data, sizeof(read_data));
        if (rc != NRF_SUCCESS)
        {
            NRF_LOG_INFO("nrf_fstorage_read() returned: %s\n", nrf_strerror_get(rc));
        }
        data_hr_hr[i] = *(ble_data_ht_hr_t *)(read_data);
        NRF_LOG_INFO("Read done, %d, 0x%x", i, read_address);
        NRF_LOG_FLUSH();
        read_address = read_address + sizeof(read_data);
    }

    //Current time, time_stamp
    static uint8_t read_time_stamp[sizeof(time_stamp)];
    rc = nrf_fstorage_read(&fstorage, read_address, read_time_stamp, sizeof(read_time_stamp));
    if (rc != NRF_SUCCESS)
    {
        NRF_LOG_INFO("nrf_fstorage_read() returned: %s\n", nrf_strerror_get(rc));
    }
    //time_stamp = *(ble_date_time_t *)(read_time_stamp);
    volatile ble_date_time_t temp_stamp = *(ble_date_time_t *)(read_time_stamp);
    if (temp_stamp.year > 1900 && temp_stamp.year < 2200)
    {
        time_stamp.year = temp_stamp.year;
    }
    else
    {
        NRF_LOG_INFO("Read error: year, %d", temp_stamp.year);
    }
    if (temp_stamp.month > 0 && temp_stamp.month < 13)
    {
        time_stamp.month = temp_stamp.month;
    }
    else
    {
        NRF_LOG_INFO("Read error: month, %d", temp_stamp.month);
    }
    if (temp_stamp.day > 0 && temp_stamp.day < 32)
    {
        time_stamp.day = temp_stamp.day;
    }
    else
    {
        NRF_LOG_INFO("Read error: day, %d", temp_stamp.day);
    }
    if (temp_stamp.hours < 24)
    {
        time_stamp.hours = temp_stamp.hours;
    }
    else
    {
        NRF_LOG_INFO("Read error: hours, %d", temp_stamp.hours);
    }
    if (temp_stamp.minutes < 60)
    {
        time_stamp.minutes = temp_stamp.minutes;
    }
    else
    {
        NRF_LOG_INFO("Read error: minutes, %d", temp_stamp.minutes);
    }
    if (temp_stamp.seconds < 60)
    {
        time_stamp.seconds = temp_stamp.seconds;
    }
    else
    {
        NRF_LOG_INFO("Read error: seconds, %d", temp_stamp.seconds);
    }
    NRF_LOG_INFO("%04d-%02d-%02dT%02d:%02d:%02d", time_stamp.year, time_stamp.month, time_stamp.day, time_stamp.hours, time_stamp.minutes, time_stamp.seconds);
    NRF_LOG_INFO("Read done, 0x%x", read_address);
    NRF_LOG_FLUSH();
    read_address = read_address + sizeof(read_time_stamp);

    //Write index, Write_index_data_hr_hr
    static uint8_t read_write_index[sizeof(Write_index_data_hr_hr)];
    rc = nrf_fstorage_read(&fstorage, read_address, read_write_index, sizeof(read_write_index));
    if (rc != NRF_SUCCESS)
    {
        NRF_LOG_INFO("nrf_fstorage_read() returned: %s\n", nrf_strerror_get(rc));
    }
    volatile unsigned int temp_Write_index_data_hr_hr = *(unsigned int *)(read_write_index);
    if (temp_Write_index_data_hr_hr < Num_of_data_hr_hr)
    {
        Write_index_data_hr_hr = temp_Write_index_data_hr_hr;
    }
    else
    {
        Write_index_data_hr_hr = 0;
    }
    NRF_LOG_INFO("Read done, 0x%x", read_address);
    NRF_LOG_FLUSH();
    read_address = read_address + sizeof(read_write_index);

    //Read index, Read_index_data_hr_hr
    static uint8_t read_read_index[sizeof(Read_index_data_hr_hr)];
    rc = nrf_fstorage_read(&fstorage, read_address, read_read_index, sizeof(read_read_index));
    if (rc != NRF_SUCCESS)
    {
        NRF_LOG_INFO("nrf_fstorage_read() returned: %s\n", nrf_strerror_get(rc));
    }
    volatile unsigned int temp_Read_index_data_hr_hr = *(unsigned int *)(read_read_index);
    if (temp_Read_index_data_hr_hr < Num_of_data_hr_hr)
    {
        Read_index_data_hr_hr = temp_Read_index_data_hr_hr;
    }
    else
    {
        Read_index_data_hr_hr = 0;
    }
    NRF_LOG_INFO("Read done, 0x%x", read_address);
    NRF_LOG_FLUSH();
    read_address = read_address + sizeof(read_read_index);

    //Count index, Count_index_data_hr_hr
    static uint8_t read_count_index[sizeof(Count_index_data_hr_hr)];
    rc = nrf_fstorage_read(&fstorage, read_address, read_count_index, sizeof(read_count_index));
    if (rc != NRF_SUCCESS)
    {
        NRF_LOG_INFO("nrf_fstorage_read() returned: %s\n", nrf_strerror_get(rc));
    }
    volatile unsigned int temp_Count_index_data_hr_hr = *(unsigned int *)(read_count_index);
    if (temp_Count_index_data_hr_hr <= Num_of_data_hr_hr)
    {
        Count_index_data_hr_hr = temp_Count_index_data_hr_hr;
    }
    else
    {
        Count_index_data_hr_hr = 0;
    }
    NRF_LOG_INFO("temp Count index : 0x%x", temp_Count_index_data_hr_hr);
    NRF_LOG_INFO("Read done, 0x%x", read_address);
    NRF_LOG_FLUSH();
    read_address = read_address + sizeof(read_count_index);

    NRF_LOG_INFO("==============================");
    NRF_LOG_INFO("Write index: %d", Write_index_data_hr_hr);
    NRF_LOG_INFO("Read index : %d", Read_index_data_hr_hr);
    NRF_LOG_INFO("Count index: %d", Count_index_data_hr_hr);
    NRF_LOG_INFO("==============================");

    //flash erase
    uint32_t erase_address = Flash_start_address;
    uint32_t pages_cnt = 3;
    rc = nrf_fstorage_erase(&fstorage, erase_address, pages_cnt, NULL);
    if (rc != NRF_SUCCESS)
    {
        NRF_LOG_INFO("nrf_fstorage_erase() returned: %s\n", nrf_strerror_get(rc));
    }

    // Start execution.
    printf("\r\nUART started.\r\n");
    NRF_LOG_INFO("Debug logging for UART over RTT started.");
    NRF_LOG_INFO("Heart Rate Sensor example started.");
    NRF_LOG_INFO("Health Thermometer example started.");
    
    application_timers_start();
    //NRF_LOG_FLUSH();

    /*
    nrf_drv_gpiote_in_event_disable(BH1792GLC_INT_PIN);
    ret_code_t err_code;
    err_code = app_timer_stop(m_bh1792glc_timer_id);
    APP_ERROR_CHECK(err_code);
    int32_t ret = 0;
    ret = bh1792_StopMeasure();
    error_check(ret, "bh1792_StopMeasure");
    NRF_LOG_INFO("finished bh1792_StopMeasure.");
    */

    advertising_start(erase_bonds);
    State_keeper = STATE_ADVERTISING;
    //NRF_LOG_FLUSH();

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}


/**
 * @}
 */
