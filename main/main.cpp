
#include "Arduino.h"
#include "Wire.h"
#include "bsec2.h"
#include "bme68xLibrary.h"

extern "C"
{
    #include <stdio.h>
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "driver/gpio.h"
    #include "esp_log.h"
    #include "sdkconfig.h"
}

extern "C"
{
    void app_main(void);    
}

#define LED 2

#define PANIC_LED LED
#define ERROR_DUR 1000

Bsec2 envSensor;

String output;
int IAQ = 0;
int IAQ_accuracy = 0;
long eCO2 = 0;
long VOCe = 0;
float raw_temp = 0;
float pressure = 0;
float raw_humidity = 0;
float raw_gas = 0;
float gas_pc = 0;


void errLeds(void);
void checkBsecStatus(Bsec2 bsec);
void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec);
void BME688_Initialization(void);
void BME688_Readings(void* arg);


void app_main(void)
{

    BME688_Initialization();
    xTaskCreate(BME688_Readings,"BME688 Readings",1024*2,NULL,2,NULL);
}

void BME688_Initialization(void)
{
    bsecSensor sensorList[] = {
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_STATIC_IAQ,
        BSEC_OUTPUT_CO2_EQUIVALENT,
        BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
        BSEC_OUTPUT_RAW_TEMPERATURE,
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_RAW_HUMIDITY,
        BSEC_OUTPUT_RAW_GAS,
        BSEC_OUTPUT_STABILIZATION_STATUS,
        BSEC_OUTPUT_RUN_IN_STATUS,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
        BSEC_OUTPUT_GAS_PERCENTAGE};

    Serial.begin(115200);
    Wire.begin();
    pinMode(PANIC_LED, OUTPUT);

    while (!Serial)
        vTaskDelay(1000/portTICK_PERIOD_MS);
    if (!envSensor.begin(BME68X_I2C_ADDR_LOW, Wire))
    {
        checkBsecStatus(envSensor);
    }

    if (!envSensor.updateSubscription(sensorList, ARRAY_LEN(sensorList), BSEC_SAMPLE_RATE_LP))
    {
        checkBsecStatus(envSensor);
    }
    {
        envSensor.attachCallback(newDataCallback);
    }

    Serial.println("BSEC library version " +
                   String(envSensor.version.major) + "." + String(envSensor.version.minor) + "." + String(envSensor.version.major_bugfix) + "." + String(envSensor.version.minor_bugfix));

    vTaskDelay(2000/portTICK_PERIOD_MS);
}

/* Function that is looped forever */
void BME688_Readings(void* arg)
{
    while(1)
    {
        if (!envSensor.run())
        {
            checkBsecStatus(envSensor);
        }
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

void errLeds(void)
{
    while (1)
    {
        digitalWrite(PANIC_LED, HIGH);
        vTaskDelay(ERROR_DUR/portTICK_PERIOD_MS);
        digitalWrite(PANIC_LED, LOW);
        vTaskDelay(ERROR_DUR/portTICK_PERIOD_MS);
    }
}

void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec)
{
    if (!outputs.nOutputs)
    {
        return;
    }

    Serial.println("BSEC outputs:\n\ttimestamp = " + String((int)(outputs.output[0].time_stamp / INT64_C(1000000))));

    for (uint8_t i = 0; i < outputs.nOutputs; i++)
    {
        const bsecData output = outputs.output[i];
        switch (output.sensor_id)
        {
        case BSEC_OUTPUT_IAQ:
            IAQ = output.signal;
            Serial.println("\tiaq = " + String(output.signal));
            break;
        case BSEC_OUTPUT_STATIC_IAQ:
            IAQ_accuracy = output.accuracy;
            Serial.println("\tiaq accuracy = " + String((int)output.accuracy));
            break;
        case BSEC_OUTPUT_CO2_EQUIVALENT:
            eCO2 = output.signal;
            Serial.println("\tCO2 equivalent = " + String(output.signal));
            break;
        case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
            VOCe = output.signal;
            Serial.println("\tVOC equivalent = " + String(output.signal));
            break;
        case BSEC_OUTPUT_RAW_TEMPERATURE:
            raw_temp = output.signal;
            Serial.println("\ttemperature = " + String(output.signal));
            break;
        case BSEC_OUTPUT_RAW_PRESSURE:
            pressure = (output.signal) / 100;
            Serial.println("\tpressure = " + String(pressure));
            break;
        case BSEC_OUTPUT_RAW_HUMIDITY:
            raw_humidity = output.signal;
            Serial.println("\thumidity = " + String(output.signal));
            break;
        case BSEC_OUTPUT_RAW_GAS:
            raw_gas = output.signal;
            Serial.println("\tgas resistance = " + String(output.signal));
            break;
        case BSEC_OUTPUT_STABILIZATION_STATUS:
            Serial.println("\tstabilization status = " + String(output.signal));
            break;
        case BSEC_OUTPUT_RUN_IN_STATUS:
            Serial.println("\trun in status = " + String(output.signal));
            break;
        case BSEC_OUTPUT_GAS_PERCENTAGE:
            gas_pc = output.signal;
            Serial.println("\tGas percentage = " + String(output.signal));
            break;
        default:
            break;
        }
    }
}

void checkBsecStatus(Bsec2 bsec)
{
    if (bsec.status < BSEC_OK)
    {
        Serial.println("BSEC error code : " + String(bsec.status));
        errLeds(); /* Halt in case of failure */
    }
    else if (bsec.status > BSEC_OK)
    {
        Serial.println("BSEC warning code : " + String(bsec.status));
    }

    if (bsec.sensor.status < BME68X_OK)
    {
        Serial.println("BME68X error code : " + String(bsec.sensor.status));
        errLeds(); /* Halt in case of failure */
    }
    else if (bsec.sensor.status > BME68X_OK)
    {
        Serial.println("BME68X warning code : " + String(bsec.sensor.status));
    }
}




