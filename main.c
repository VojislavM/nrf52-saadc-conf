/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
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
 * @defgroup ble_sdk_app_beacon_main main.c
 * @{
 * @ingroup ble_sdk_app_beacon
 * @brief Beacon Transmitter Sample Application main file.
 *
 * This file contains the source code for an Beacon transmitter sample application.
 */

#include <stdbool.h>
#include <stdint.h>
#include "nordic_common.h"
#include "bsp.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_advdata.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_saadc.h"


#include "nrf_pwr_mgmt.h"

#include "nrf_drv_clock.h"

#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define UART_PRINTING_ENABLED
#define SAADC_CALIBRATION_INTERVAL 5              //Determines how often the SAADC should be calibrated relative to NRF_DRV_SAADC_EVT_DONE event. E.g. value 5 will make the SAADC calibrate every fifth time the NRF_DRV_SAADC_EVT_DONE is received.
#define SAADC_SAMPLES_IN_BUFFER 1                 //Number of SAADC samples in RAM before returning a SAADC event. For low power SAADC set this constant to 1. Otherwise the EasyDMA will be enabled for an extended time which consumes high current.
#define SAADC_OVERSAMPLE NRF_SAADC_OVERSAMPLE_4X  //Oversampling setting for the SAADC. Setting oversample to 4x This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times. Enable BURST mode to make the SAADC sample 4 times when triggering SAMPLE task once.
#define SAADC_BURST_MODE 1                        //Set to 1 to enable BURST mode, otherwise set to 0.

#define USER_BUTTON                     20                                 /**< User button for interfacing with the device connected to pin P0.20. */
#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */
#define USER_LED                        17                                 /**< User LED is on pin P0.11. */
#define POWER_PIN                       4                                 /**< Power pin that holds power enabled for the MCU, on pin P0.04. */

#define ALARM_TIMEOUT_SEC               900                                /**< After that time alarm state will stop, 900 sec = 15 min. */

#define SIMPLE_TIMER_CONFIG_FREQUENCY   4

#define APP_BLE_CONN_CFG_TAG            1                                  /**< A tag identifying the SoftDevice BLE configuration. */

#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(5000, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define APP_BEACON_INFO_LENGTH          0x04                               /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x15                               /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE                 0x02                               /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI               0xC3                               /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER          0xFFFE                             /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_MAJOR_VALUE                 0x01, 0x02                         /**< Major value used to identify Beacons. */
#define APP_MINOR_VALUE                 0x03, 0x04                         /**< Minor value used to identify Beacons. */
#define APP_BEACON_UUID                 0x01, 0x12, 0x23, 0x34, \
                                        0x45, 0x56, 0x67, 0x78, \
                                        0x89, 0x9a, 0xab, 0xbc, \
                                        0xcd, 0xde, 0xef, 0xf0             /**< Proprietary UUID for Beacon. */

#define VERSION_BUILD                   13
#define DEVICE_NAME                     "PacsanaV"                          /**< Name of device. Will be included in the advertising data. */

#define DEAD_BEEF                       0xDEADBEEF                         /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS 600
#define ADC_RES_12BIT                 4095
#define ADC_PRE_SCALING_COMPENSATION  6
#define ADC_OFFSET                    100

#define EC_SIG_PIN_NO_3               3
#define EC_SIG_PIN_NO_1               4
#define EC_SIG_PIN_NO_2               5
#define EC_PWR_1_PIN                  17
#define EC_PWR_2_PIN                  11
#define EC_PWR_3_PIN                  12


#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_12BIT) * ADC_PRE_SCALING_COMPENSATION) + ADC_OFFSET
//#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
//          (((ADC_VALUE) * 3600) / 4096)

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
#define MAJ_VAL_OFFSET_IN_BEACON_INFO   18                                 /**< Position of the MSB of the Major Value in m_beacon_info array. */
#define UICR_ADDRESS                    0x10001080                         /**< Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location. */
#endif
static const int ADV_DEFAULT_INTERVAL_MS = MSEC_TO_UNITS(1000, UNIT_0_625_MS);
static const int ADV_ALARM_INTERVAL_MS = MSEC_TO_UNITS(100, UNIT_0_625_MS);
static const int BUTTON_TIMEOUT_MS = 5000;

static const int ANALOG_VALUES_READ_MS = 1000; //one second more then advertising data time

static ble_gap_adv_params_t m_adv_params;                                  /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t              m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t              m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set. */

static nrf_saadc_value_t       m_buffer_pool[2][SAADC_SAMPLES_IN_BUFFER];
static uint32_t                m_adc_evt_counter = 0;
static bool                    m_saadc_calibrate = false;    

ble_advdata_t advdata;
ble_advdata_manuf_data_t manuf_specific_data;

APP_TIMER_DEF(m_repeated_timer_id);     /**< Handler for repeated timer used to blink LED 1. */
APP_TIMER_DEF(m_alarm_timer_id);     /**< Handler for oneshot alarm timer. */
APP_TIMER_DEF(m_timer_analog_values_id);     /**< Handler for repeated timer used to blink LED 1. */

/*functions prototypes */
static void button_event_handler(uint8_t, uint8_t);
void saadc_init(uint8_t analog_pin_no, uint8_t gnd_pin_no);
bool adc_pin_configuration_set(uint8_t analog_pin_no, uint8_t gnd_pin_no);

//const nrf_drv_timer_t SIMPLE_TIMER = NRF_DRV_TIMER_INSTANCE(SIMPLE_TIMER_CONFIG_INSTANCE);

uint8_t alarm_state_set = false;



/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = NULL,
        .len    = 0

    }
};


static uint8_t m_beacon_info[4] =                    /**< Information advertised by the Beacon. */
{
    187, //battery
    0,   //status
    0,   //txPower
    VERSION_BUILD
    //APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this
                         // implementation.
    //APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the
                         // manufacturer specific data in this implementation.
    //APP_BEACON_UUID,     // 128 bit UUID value.
    //APP_MAJOR_VALUE,     // Major arbitrary value that can be used to distinguish between Beacons.
    //APP_MINOR_VALUE,     // Minor arbitrary value that can be used to distinguish between Beacons.
    //APP_MEASURED_RSSI    // Manufacturer specific information. The Beacon's measured TX power in
                         // this implementation.
};


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    //ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;
    ble_gap_conn_sec_mode_t sec_mode;

    //ble_advdata_manuf_data_t manuf_specific_data;

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
    // If USE_UICR_FOR_MAJ_MIN_VALUES is defined, the major and minor values will be read from the
    // UICR instead of using the default values. The major and minor values obtained from the UICR
    // are encoded into advertising data in big endian order (MSB First).
    // To set the UICR used by this example to a desired value, write to the address 0x10001080
    // using the nrfjprog tool. The command to be used is as follows.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val <your major/minor value>
    // For example, for a major value and minor value of 0xabcd and 0x0102 respectively, the
    // the following command should be used.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val 0xabcd0102
    uint16_t major_value = ((*(uint32_t *)UICR_ADDRESS) & 0xFFFF0000) >> 16;
    uint16_t minor_value = ((*(uint32_t *)UICR_ADDRESS) & 0x0000FFFF);

    uint8_t index = MAJ_VAL_OFFSET_IN_BEACON_INFO;

    m_beacon_info[index++] = MSB_16(major_value);
    m_beacon_info[index++] = LSB_16(major_value);

    m_beacon_info[index++] = MSB_16(minor_value);
    m_beacon_info[index++] = LSB_16(minor_value);
#endif

    manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;


    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type             = BLE_ADVDATA_FULL_NAME;
    advdata.flags                 = flags;
    advdata.p_manuf_specific_data = &manuf_specific_data;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);


    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    m_adv_params.p_peer_addr     = NULL;    // Undirected advertisement.
    m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval        = ADV_DEFAULT_INTERVAL_MS;
    m_adv_params.duration        = 0;       // Never time out.

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    // no indication for adv
    //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    //APP_ERROR_CHECK(err_code);
}

/**@brief Function for stoping advertising.
 */
static void advertising_stop(void)
{
    ret_code_t err_code;

    err_code = sd_ble_gap_adv_stop(m_adv_handle);
    APP_ERROR_CHECK(err_code);

    return;
}

static void advertising_parameters_update(int status){
    ret_code_t err_code;

    m_beacon_info[1] = status;
    manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;

    advdata.p_manuf_specific_data = &manuf_specific_data;
    
    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);

    return;
}

static void advertising_parameters_battery_update(int battery){
    ret_code_t err_code;

    m_beacon_info[0] = battery;
    manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;

    advdata.p_manuf_specific_data = &manuf_specific_data;
    
    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);

    return;
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
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
}


/**@brief Function for initializing logging. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing LEDs. */
static void leds_init(void)
{
    //ret_code_t err_code = bsp_init(BSP_INIT_LEDS, NULL);
    nrf_gpio_cfg_output(USER_LED);
    //APP_ERROR_CHECK(err_code);
}

/**@brief Function for turning on led. */
static void user_led_on(void){
    //turn on user led
    nrf_gpio_pin_clear(USER_LED);
}

/**@brief Function for turning off led. */
static void user_led_off(void){
    //turn off user led
    nrf_gpio_pin_set(USER_LED);
}

/**@brief Function for initializing LEDs. */
static void power_pin_init(void)
{
    nrf_gpio_cfg_output(POWER_PIN);
}

/**@brief Function for turning on led. */
static void power_pin_on(void){
    //turn on user led
    nrf_gpio_pin_set(POWER_PIN);
}

/**@brief Function for turning off led. */
static void power_pin_off(void){
    //turn off user led
    nrf_gpio_pin_clear(POWER_PIN);
}


/**@brief Function for initializing the button handler module.*/
static void buttons_init(void)
{
    ret_code_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {USER_BUTTON, APP_BUTTON_ACTIVE_HIGH, NRF_GPIO_PIN_PULLDOWN, button_event_handler}
    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons), BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);

}

/**@brief Function for initializing timers. */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

/**@brief Timeout handler for the repeated timer.
 */
static void repeated_timer_handler(void * p_context){
    //nrf_drv_gpiote_out_toggle(USER_LED);
    ret_code_t err_code;
    static bool buttonStatus = false;

    if(!buttonStatus){
        if(nrf_gpio_pin_read(USER_BUTTON) == 1){
            buttonStatus = true;
            user_led_on();
            // Stop the repeated timer.
            err_code = app_timer_stop(m_repeated_timer_id);
            APP_ERROR_CHECK(err_code);
            // start the timer
            err_code = app_timer_start(m_repeated_timer_id, APP_TIMER_TICKS(2000), NULL);
            APP_ERROR_CHECK(err_code);
        }
    }
    else{
        buttonStatus = false;
        user_led_off();
        // Stop the repeated timer.
        err_code = app_timer_stop(m_repeated_timer_id);
        APP_ERROR_CHECK(err_code);
        // Power OFF
        power_pin_off();
        while(1);
    }
    //nrf_drv_gpiote_out_toggle(POWER_PIN);
}

static void pin_write(int ec_pin, bool level){
    if(level == true){
        nrf_gpio_pin_set(ec_pin);
    }
    else if(level == false){
        nrf_gpio_pin_clear(ec_pin);
    }
}

static void adc_timer_handler(void * p_context){
    NRF_LOG_INFO("timer_handler.");
    //TODO: 
    //set right pin configuration
    //init saadc with same right pins
    //adc_pin_configuration_set(EC_SIG_PIN_NO_1, EC_SIG_PIN_NO_2);
    //saadc_init(EC_SIG_PIN_NO_1, EC_SIG_PIN_NO_2);                                    //Initialize and start SAADC
    nrf_drv_saadc_sample();                                        //Trigger the SAADC SAMPLE task
}

void adc_pin_configuration_init(void)
{
    //init all pins to idle default state
    //set pins output - default state output low
    nrf_gpio_cfg_output(EC_SIG_PIN_NO_1);
    nrf_gpio_cfg_output(EC_SIG_PIN_NO_2);
    nrf_gpio_cfg_output(EC_SIG_PIN_NO_3);
    //set pins LOW
    pin_write(EC_SIG_PIN_NO_1, false);
    pin_write(EC_SIG_PIN_NO_2, false);
    pin_write(EC_SIG_PIN_NO_3, false);

    //set pins as output
    nrf_gpio_cfg_output(EC_PWR_1_PIN);
    nrf_gpio_cfg_output(EC_PWR_2_PIN); 
    nrf_gpio_cfg_output(EC_PWR_3_PIN);
    //set pins LOW
    pin_write(EC_PWR_1_PIN, false);
    pin_write(EC_PWR_2_PIN, false);
    pin_write(EC_PWR_3_PIN, false);
}

bool adc_pin_configuration_set(uint8_t analog_pin_no, uint8_t gnd_pin_no)
{
    //chack if the pins are same
    if(analog_pin_no == gnd_pin_no)
    {
        return false;
    }
    //set analog_pin_no to  nrf_gpio_cfg_default
    if(analog_pin_no == EC_SIG_PIN_NO_1)
    {
        pin_write(EC_PWR_1_PIN, true);
        nrf_gpio_cfg_default(EC_PWR_2_PIN);
        nrf_gpio_cfg_input(EC_PWR_2_PIN, NRF_GPIO_PIN_NOPULL);
        nrf_gpio_cfg_default(EC_PWR_3_PIN);
        nrf_gpio_cfg_input(EC_PWR_3_PIN, NRF_GPIO_PIN_NOPULL);

        nrf_gpio_cfg_default(EC_SIG_PIN_NO_1);
    }    
    else if(analog_pin_no == EC_SIG_PIN_NO_2)
    {
        nrf_gpio_cfg_default(EC_PWR_1_PIN);
        nrf_gpio_cfg_input(EC_PWR_1_PIN, NRF_GPIO_PIN_NOPULL);
        pin_write(EC_PWR_2_PIN, true);
        nrf_gpio_cfg_default(EC_PWR_3_PIN);
        nrf_gpio_cfg_input(EC_PWR_3_PIN, NRF_GPIO_PIN_NOPULL);
        
        nrf_gpio_cfg_default(EC_SIG_PIN_NO_2);
    }
    else if(analog_pin_no == EC_SIG_PIN_NO_3)
    {
        nrf_gpio_cfg_default(EC_PWR_1_PIN);
        nrf_gpio_cfg_input(EC_PWR_1_PIN, NRF_GPIO_PIN_NOPULL);
        pin_write(EC_PWR_2_PIN, true);
        nrf_gpio_cfg_default(EC_PWR_3_PIN);
        nrf_gpio_cfg_input(EC_PWR_3_PIN, NRF_GPIO_PIN_NOPULL);
        
        nrf_gpio_cfg_default(EC_SIG_PIN_NO_3);
    }
    else
    {
        return false;
    }

    //selelc ref gnd
    if(gnd_pin_no == EC_SIG_PIN_NO_1)
    {
        pin_write(EC_SIG_PIN_NO_1, false);
        nrf_gpio_cfg_default(EC_SIG_PIN_NO_2);
        nrf_gpio_cfg_input(EC_SIG_PIN_NO_2, NRF_GPIO_PIN_NOPULL);
        nrf_gpio_cfg_default(EC_SIG_PIN_NO_3);
        nrf_gpio_cfg_input(EC_SIG_PIN_NO_3, NRF_GPIO_PIN_NOPULL);
    }
    else if(gnd_pin_no == EC_SIG_PIN_NO_2)
    {
        nrf_gpio_cfg_default(EC_SIG_PIN_NO_1);
        nrf_gpio_cfg_input(EC_SIG_PIN_NO_1, NRF_GPIO_PIN_NOPULL);
        pin_write(EC_SIG_PIN_NO_2, false);
        nrf_gpio_cfg_default(EC_SIG_PIN_NO_3);
        nrf_gpio_cfg_input(EC_SIG_PIN_NO_3, NRF_GPIO_PIN_NOPULL);
    }
    else if(gnd_pin_no == EC_SIG_PIN_NO_3)
    {
        nrf_gpio_cfg_default(EC_SIG_PIN_NO_1);
        nrf_gpio_cfg_input(EC_SIG_PIN_NO_1, NRF_GPIO_PIN_NOPULL);
        nrf_gpio_cfg_default(EC_SIG_PIN_NO_2);
        nrf_gpio_cfg_input(EC_SIG_PIN_NO_2, NRF_GPIO_PIN_NOPULL);
        pin_write(EC_SIG_PIN_NO_3, false);
    }
    else
    {
        return false;
    }
    
    return true;
}

//pin configuration when there is no measurement and divice is in idle mode
void adc_pin_configuration_idle(void)
{
    //set pins to default state
    nrf_gpio_cfg_default(EC_PWR_1_PIN);
    nrf_gpio_cfg_default(EC_PWR_2_PIN);
    nrf_gpio_cfg_default(EC_PWR_3_PIN);
    nrf_gpio_cfg_default(EC_PWR_1_PIN);
    nrf_gpio_cfg_default(EC_PWR_2_PIN);
    nrf_gpio_cfg_default(EC_PWR_3_PIN);
    //init all pins to idle state
    adc_pin_configuration_init();
}

void timer2_create(void){
    ret_code_t err_code;

    // Create timers
    err_code = app_timer_create(&m_repeated_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                repeated_timer_handler);
    APP_ERROR_CHECK(err_code);
}

void timer_adc_create(void){
    ret_code_t err_code;

    // Create timers
    err_code = app_timer_create(&m_timer_analog_values_id,
                                APP_TIMER_MODE_REPEATED,
                                adc_timer_handler);
    APP_ERROR_CHECK(err_code);
}

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    ret_code_t err_code;
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)                                                        //Capture offset calibration complete event
    {
			
        //LEDS_INVERT(BSP_LED_1_MASK);                                                                    //Toggle LED2 to indicate SAADC buffer full		

        if((m_adc_evt_counter % SAADC_CALIBRATION_INTERVAL) == 0)                                  //Evaluate if offset calibration should be performed. Configure the SAADC_CALIBRATION_INTERVAL constant to change the calibration frequency
        {
            nrf_drv_saadc_abort();                                                                      // Abort all ongoing conversions. Calibration cannot be run if SAADC is busy
            m_saadc_calibrate = true;                                                                   // Set flag to trigger calibration in main context when SAADC is stopped
        }
        

#ifdef UART_PRINTING_ENABLED
        NRF_LOG_INFO("ADC event number: %d\r\n",(int)m_adc_evt_counter);                                //Print the event number on UART

        for (int i = 0; i < p_event->data.done.size; i++)
        {
            NRF_LOG_INFO("%d\r\n", ADC_RESULT_IN_MILLI_VOLTS(p_event->data.done.p_buffer[i]));                                     //Print the SAADC result on UART
        }
#endif //UART_PRINTING_ENABLED    
        advertising_stop();
        //NRF_LOG_INFO("%d, %d, \r\n", ADC_RESULT_IN_MILLI_VOLTS(p_event->data.done.p_buffer[0]), (ADC_RESULT_IN_MILLI_VOLTS(p_event->data.done.p_buffer[0]))/16);
        advertising_parameters_battery_update((ADC_RESULT_IN_MILLI_VOLTS(p_event->data.done.p_buffer[0]))/16);
        advertising_start();

        if(m_saadc_calibrate == false)
        {
            err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);             //Set buffer so the SAADC can write to it again. 
            APP_ERROR_CHECK(err_code);
        }
        
        m_adc_evt_counter++;

        //TODO: 
        //set adc pins to default state
        //uninit saadc
//        NRF_LOG_INFO("SAADC uninit\r\n"); 
//        nrf_drv_saadc_uninit();                                                                   //Unintialize SAADC to disable EasyDMA and save power
//        NRF_SAADC->INTENCLR = (SAADC_INTENCLR_END_Clear << SAADC_INTENCLR_END_Pos);               //Disable the SAADC interrupt
//        NVIC_ClearPendingIRQ(SAADC_IRQn);
//        adc_pin_configuration_idle();
  
    }
    else if (p_event->type == NRF_DRV_SAADC_EVT_CALIBRATEDONE)
    {
        //LEDS_INVERT(BSP_LED_2_MASK);                                                                    //Toggle LED3 to indicate SAADC calibration complete
        
        err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAADC_SAMPLES_IN_BUFFER);             //Set buffer so the SAADC can write to it again. 
        APP_ERROR_CHECK(err_code);
        err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAADC_SAMPLES_IN_BUFFER);             //Need to setup both buffers, as they were both removed with the call to nrf_drv_saadc_abort before calibration.
        APP_ERROR_CHECK(err_code);
        
#ifdef UART_PRINTING_ENABLED
        NRF_LOG_INFO("SAADC calibration complete ! \r\n");                                              //Print on UART
#endif //UART_PRINTING_ENABLED	
        
    }

}


void saadc_init(uint8_t analog_pin_no, uint8_t gnd_pin_no)
{
    ret_code_t err_code;
    nrf_drv_saadc_config_t saadc_config;
    nrf_saadc_channel_config_t channel_config;
	
    //Configure SAADC
    saadc_config.low_power_mode = true;                                                   //Enable low power mode.
    saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;                                 //Set SAADC resolution to 12-bit. This will make the SAADC output values from 0 (when input voltage is 0V) to 2^12=2048 (when input voltage is 3.6V for channel gain setting of 1/6).
    saadc_config.oversample = SAADC_OVERSAMPLE;                                           //Set oversample to 4x. This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times.
    saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;                               //Set SAADC interrupt to low priority.
	
    //Initialize SAADC
    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);                         //Initialize the SAADC with configuration and callback function. The application must then implement the saadc_callback function, which will be called when SAADC interrupt is triggered
    APP_ERROR_CHECK(err_code);
		
    //Configure SAADC channel
    channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL;                              //Set internal reference of fixed 0.6 volts
    channel_config.gain = NRF_SAADC_GAIN1_6;                                              //Set input gain to 1/6. The maximum SAADC input voltage is then 0.6V/(1/6)=3.6V. The single ended input range is then 0V-3.6V
    channel_config.acq_time = NRF_SAADC_ACQTIME_10US;                                     //Set acquisition time. Set low acquisition time to enable maximum sampling frequency of 200kHz. Set high acquisition time to allow maximum source resistance up to 800 kohm, see the SAADC electrical specification in the PS. 
    channel_config.mode = NRF_SAADC_MODE_SINGLE_ENDED;                                    //Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
    
    //selelct the right analog input pin and gnd reference pin
    if(analog_pin_no == EC_SIG_PIN_NO_1)                                                  //Select the input pin for the channel. AIN0 pin maps to physical pin P0.02.
    {
        channel_config.pin_p = NRF_SAADC_INPUT_AIN2;
    }
    else if(analog_pin_no == EC_SIG_PIN_NO_2)                                              //Select the input pin for the channel. AIN0 pin maps to physical pin P0.02.
    {
        channel_config.pin_p = NRF_SAADC_INPUT_AIN3;
    }
    else if(analog_pin_no == EC_SIG_PIN_NO_3)
    {
        channel_config.pin_p = NRF_SAADC_INPUT_AIN1;
    }                                         
    channel_config.pin_n = NRF_SAADC_INPUT_DISABLED;                                      //Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
    channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pullup resistor on the input pin
    channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pulldown resistor on the input pin

	
    //Initialize SAADC channel
    err_code = nrf_drv_saadc_channel_init(0, &channel_config);                            //Initialize SAADC channel 0 with the channel configuration
    APP_ERROR_CHECK(err_code);
		
    if(SAADC_BURST_MODE)
    {
        NRF_SAADC->CH[0].CONFIG |= 0x01000000;                                            //Configure burst mode for channel 0. Burst is useful together with oversampling. When triggering the SAMPLE task in burst mode, the SAADC will sample "Oversample" number of times as fast as it can and then output a single averaged value to the RAM buffer. If burst mode is not enabled, the SAMPLE task needs to be triggered "Oversample" number of times to output a single averaged value to the RAM buffer.		
    }

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0],SAADC_SAMPLES_IN_BUFFER);    //Set SAADC buffer 1. The SAADC will start to write to this buffer
    APP_ERROR_CHECK(err_code);

    
    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1],SAADC_SAMPLES_IN_BUFFER);    //Set SAADC buffer 2. The SAADC will write to this buffer when buffer 1 is full. This will give the applicaiton time to process data in buffer 1.
    APP_ERROR_CHECK(err_code);

}

void application_init(void){
    ret_code_t err_code;
    // Initialize.
    log_init();
    //bsp_board_init(BSP_INIT_LEDS);
    timers_init();//used by softdevice
    timer_adc_create();
    power_management_init();
    ble_stack_init();
    advertising_init();

    adc_pin_configuration_init();
    adc_pin_configuration_set(EC_SIG_PIN_NO_1, EC_SIG_PIN_NO_2);
    saadc_init(EC_SIG_PIN_NO_1, EC_SIG_PIN_NO_2);                                    //Initialize and start SAADC


    //bsp_board_leds_on();

    return;
}

/**
 * @brief Function for application main entry.
 */
int main(void){
    ret_code_t err_code;
    //init appliction 
    application_init();

    /*start the button timer*/
    err_code = app_timer_start(m_timer_analog_values_id, APP_TIMER_TICKS(ANALOG_VALUES_READ_MS), NULL);
    APP_ERROR_CHECK(err_code);

    // Start execution.
    NRF_LOG_INFO("saadc conf example started.");
    advertising_start();

    // Enter main loop.
    for (;; ){
        if(m_saadc_calibrate == true)
        {
#ifdef UART_PRINTING_ENABLED
            NRF_LOG_INFO("SAADC calibration starting...  \r\n");    //Print on UART
#endif //UART_PRINTING_ENABLED	
            while(nrf_drv_saadc_calibrate_offset() != NRF_SUCCESS); //Trigger calibration task
            m_saadc_calibrate = false;
        }
        idle_state_handle();
#ifdef UART_PRINTING_ENABLED
        NRF_LOG_FLUSH();
#endif //UART_PRINTING_ENABLED
    }
}


/**
 * @}
 */
