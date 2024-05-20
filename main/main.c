/*
 Zigbee interface 
 Sends sensors data read from bme680 multi data sensor using zigbee 
 */

#include <stdio.h>
#include <esp_system.h>
#include <bme680.h>
#include <string.h>
#include "bh1750_i2c.h"
#include "bh1750_i2c_hal.h"
/*configuration of address and pins for the BME680 sensor*/
#define BME680_I2C_ADDR 0x77
#define PORT 0
#define CONFIG_EXAMPLE_I2C_MASTER_SDA 21
#define CONFIG_EXAMPLE_I2C_MASTER_SCL 22

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "esp_zb_temperature_sensor.h"
/* I2C pins for the BHT1750 sensor*/
#define CONFIG_I2C_MASTER_SCL 19
#define CONFIG_I2C_MASTER_SDA 18

// Global variables
volatile float temperature=0;
volatile float humidity=0;
volatile float pressure=0;
volatile float lux=0;
#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile light (End Device) source code.
#endif

static const char *TAG = "ESP_ZB_SENSOR_NODE";
/********************* Define functions **************************/

static int16_t zb_temperature_to_s16(float temp)
{
    return (int16_t)(temp * 100);
}

static int16_t zb_humidity_to_s16(float humidity)
{
    return (int16_t)(humidity * 100);
}
static int16_t zb_illuminance_to_s16(float lux)
{
    return (int16_t)(lux * 10);
}
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}


void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialized");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Start network steering");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            /* commissioning failed */
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
            printf("Entering deep sleep\n");
            
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel());
        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

void bme680_test(void *pvParameters)
{
    bme680_t sensor;
    memset(&sensor, 0, sizeof(bme680_t));

    ESP_ERROR_CHECK(bme680_init_desc(&sensor, BME680_I2C_ADDR, PORT, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    // init the sensor
    ESP_ERROR_CHECK(bme680_init_sensor(&sensor));

    // Changes the oversampling rates to 4x oversampling for temperature
    // and 2x oversampling for humidity. Pressure measurement is skipped.
    bme680_set_oversampling_rates(&sensor, BME680_OSR_4X, BME680_OSR_2X, BME680_OSR_2X);

    // Change the IIR filter size for temperature and pressure to 7.
    bme680_set_filter_size(&sensor, BME680_IIR_SIZE_7);

    // Change the heater profile 0 to 200 degree Celsius for 100 ms.
    bme680_set_heater_profile(&sensor, 0, 200, 100);
    bme680_use_heater_profile(&sensor, 0);

    // Set ambient temperature to 10 degree Celsius
    bme680_set_ambient_temperature(&sensor, 10);

    // as long as sensor configuration isn't changed, duration is constant
    uint32_t duration;
    bme680_get_measurement_duration(&sensor, &duration);

    TickType_t last_wakeup = xTaskGetTickCount();

    bme680_values_float_t values;
    
    
        // trigger the sensor to start one TPHG measurement cycle
        if (bme680_force_measurement(&sensor) == ESP_OK)
        {
            // passive waiting until measurement results are available
            vTaskDelay(duration);

            // get the results and do something with them
            if (bme680_get_results_float(&sensor, &values) == ESP_OK)
            temperature = values.temperature;
            humidity = values.humidity;
            pressure = values.pressure;
                printf("BME680 Sensor: %.2f °C, %.2f %%, %.2f hPa, %.2f Ohm\n",
                        values.temperature, values.humidity, values.pressure, values.gas_resistance);
        }
    
        // passive waiting until 1 second is over
        vTaskDelayUntil(&last_wakeup, pdMS_TO_TICKS(1000));
        //printf("char_handle %c is", charHandle);


    vTaskDelete(NULL);
}
void bht1750()
{
    bh1750_dev_t dev_1;
    esp_err_t err;
    int x=0;
    bh1750_i2c_hal_init();

    /* Device init */
    dev_1.i2c_addr = I2C_ADDRESS_BH1750;
    dev_1.mtreg_val = DEFAULT_MEAS_TIME_REG_VAL;

    /* Perform device reset */
    err = bh1750_i2c_dev_reset(dev_1); 
    ESP_LOGI(TAG, "Device reset: %s", err == BH1750_OK ? "Successful" : "Failed");

    err += bh1750_i2c_set_power_mode(dev_1, BH1750_POWER_ON);
    ESP_LOGI(TAG, "Changing power mode to ON: %s", err == BH1750_OK ? "Successful" : "Failed");

    /* Change measurement time with  50% optical window transmission rate */
    err += bh1750_i2c_set_mtreg_val(&dev_1, 50);
    ESP_LOGI(TAG, "Changing measurement time: %s", err == BH1750_OK ? "Successful" : "Failed");

    /* Configure device */
    err += bh1750_i2c_set_resolution_mode(&dev_1, BH1750_CONT_H_RES_MODE);
    if (err == BH1750_OK)
    {
        ESP_LOGI(TAG, "BH1750 config successful");
    }
    else{
        ESP_LOGE(TAG, "BH1750 config failed!");
    }
    /* End of device config */

    if (err == BH1750_OK)
    {
        ESP_LOGI(TAG, "BH1750 initialization successful");
        //Start reading data
        uint16_t data_light;
        while(x<3){
        
            bh1750_i2c_read_data(dev_1, &data_light);
            ESP_LOGI(TAG, "Light Intensity: %d Lux", data_light);
            lux = data_light;
            x+=1;
            vTaskDelay(500);
            
        }
    }
    else{
        ESP_LOGE(TAG, "BH1750 initialization failed!");
    }
    i2c_driver_delete(0);
    vTaskDelete(NULL);

}




static void esp_zb_task()
{
    // Define cluster attributes
    char manufname[] = {9, 'E', 's', 'p', 'r', 'e', 's', 's', 'i', 'f'};
    char modelid[] = {14, 'E', 'S', 'P', '3', '2', 'C', '6', '.', 'B', 'M', 'E', '2', '8', '0'};

    uint16_t tempValue = zb_temperature_to_s16(temperature);
    uint16_t tempMin = 0;
    uint16_t tempMax = 50;

    uint16_t humidityValue = zb_temperature_to_s16(humidity);
    uint16_t humidityMin = 0;
    uint16_t humidityMax = 9999;
    uint16_t humidityTolerance = 1;

    uint16_t pressureValue = zb_temperature_to_s16(pressure);
    uint16_t pressureMin = 0;
    uint16_t pressureMax = 100;

    uint16_t luxValue = lux;
    uint16_t luxMin = 0;
    uint16_t luxMax =7019;

    // Initialize Zigbee stack
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    //genBasic cluster/attribute list
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, &manufname[0]);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, &modelid[0]);

    //temperature cluster/attribute list
    esp_zb_attribute_list_t *esp_zb_temperature_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);
    esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &tempValue);
    esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_ID, &tempMin);
    esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_ID, &tempMax);

    //humidity cluster/attribute list
    esp_zb_attribute_list_t *esp_zb_humidity_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT);
    esp_zb_humidity_meas_cluster_add_attr(esp_zb_humidity_cluster, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &humidityValue);
    esp_zb_humidity_meas_cluster_add_attr(esp_zb_humidity_cluster, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MIN_VALUE_ID, &humidityMin);
    esp_zb_humidity_meas_cluster_add_attr(esp_zb_humidity_cluster, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MAX_VALUE_ID, &humidityMax);
    esp_zb_humidity_meas_cluster_add_attr(esp_zb_humidity_cluster, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_TOLERANCE_ID, &humidityTolerance);

    //pressure cluster/attribute list
    esp_zb_attribute_list_t *esp_zb_pressure_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT);
    esp_zb_pressure_meas_cluster_add_attr(esp_zb_pressure_cluster, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID, &luxValue);
    esp_zb_pressure_meas_cluster_add_attr(esp_zb_pressure_cluster, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_MIN_VALUE_ID, &luxMin);
    esp_zb_pressure_meas_cluster_add_attr(esp_zb_pressure_cluster, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_MAX_VALUE_ID, &luxMax);
    // light cluster/attribute list
    

    // Create cluster list
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_temperature_meas_cluster(esp_zb_cluster_list, esp_zb_temperature_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_humidity_meas_cluster(esp_zb_cluster_list, esp_zb_humidity_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_pressure_meas_cluster(esp_zb_cluster_list, esp_zb_pressure_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    //endpoint list
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_ESP_SENSOR_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_TEST_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, endpoint_config);

    // Register endpoint list
    esp_zb_device_register(esp_zb_ep_list);
   
   
    
    
    // Error check and start zigbee main loop
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_main_loop_iteration();
    vTaskDelete(NULL);
}


esp_err_t zb_update_temperature(int32_t temperature)
{
    /* Update temperature attribute */
    esp_zb_zcl_status_t state = esp_zb_zcl_set_attribute_val(
        HA_ESP_SENSOR_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        &temperature,
        false
    );

    /* Error check */
    if(state != ESP_ZB_ZCL_STATUS_SUCCESS)
    {
        ESP_LOGE(TAG, "Updating temperature attribute failed!");
        return ESP_FAIL;
    }

    return ESP_OK;
}
esp_err_t zb_update_humidity(int32_t humidity)
{
    /* Update temperature attribute */
    esp_zb_zcl_status_t state = esp_zb_zcl_set_attribute_val(
        HA_ESP_SENSOR_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
         ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
        &humidity,
        false
    );

    /* Error check */
    if(state != ESP_ZB_ZCL_STATUS_SUCCESS)
    {
        ESP_LOGE(TAG, "Updating Humidity attribute failed!");
        
        return ESP_FAIL;
    }

    return ESP_OK;
}
esp_err_t zb_update_illuminance(int32_t lux)
{
    /* Update temperature attribute */
    esp_zb_zcl_status_t state = esp_zb_zcl_set_attribute_val(
        HA_ESP_SENSOR_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID,
        &lux,
        false
    );

    /* Error check */
    if(state != ESP_ZB_ZCL_STATUS_SUCCESS)
    {
        ESP_LOGE(TAG, "Updating illuminance attribute failed!");
        
        return ESP_FAIL;
    }

    return ESP_OK;
}
static void update_attributes(void *pvParameters)
{
    /* update temperature and humidity values(attributes)*/
    

    ESP_LOGI("update_attributes","it started");
    float temp = zb_temperature_to_s16(temperature);
    float humd = zb_temperature_to_s16(humidity);
    float light = zb_illuminance_to_s16(lux);
    ESP_ERROR_CHECK(zb_update_temperature(temp));
    ESP_ERROR_CHECK(zb_update_humidity(humd));
    ESP_ERROR_CHECK(zb_update_illuminance(light));


    
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    
    ESP_ERROR_CHECK(i2cdev_init());
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    xTaskCreate(esp_zb_task,"bme_main", 4096, NULL, 3, NULL);
    vTaskDelay(1000);

    while(1){
    i2c_driver_delete(0);
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(bme680_test, "bme_main", 4096, NULL, 2, NULL);
    vTaskDelay(1000);

    xTaskCreate(update_attributes, "Zigbee_main", 4096, NULL, 4, NULL);

    vTaskDelay(1000);
    xTaskCreate(bht1750, "bht1750", 4096,NULL, 1, NULL);
    vTaskDelay(1000);
    xTaskCreate(update_attributes, "Zigbee_main", 4096, NULL, 4, NULL);
    vTaskDelay(1000);
    
    }
    
}
