#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <sdkconfig.h>
#include <stdbool.h>
#include "esp_adc/adc_oneshot.h"

// difference between the last project, enable the pulldown 

#define ignitionLED 13  
#define engineLED 3     
#define ignitionEn 37
#define driverSeatBelt 38
#define passSeatBelt 39
#define driveSeat 40
#define passSeat 41
#define Alarm 18
    
//New components defined below 
#define headLights 14
#define highBeamsOut 16
#define highBeamsIn 15
#define photoResistor ADC_CHANNEL_4
#define poteniometer ADC_CHANNEL_3
#define adcAtten ADC_ATTEN_DB_12
#define bitWidth ADC_BITWIDTH_12

bool dSense = false;
bool dsbelt = false;
bool pSense = false;
bool psbelt = false;

// Define the handlers Globally to functionize the configuration, there is also a variable called

// NTS
// "volatile" that people recommend using instead of ust a plain global variable.
// (thread talks about ISR handlers, but same concept)
//source https://stackoverflow.com/questions/27204242/how-to-avoid-global-variables-when-using-interrupt-handlers
static adc_oneshot_unit_handle_t oneShotHandler; 
static adc_cali_handle_t adcPotentiometerHandler; 
static adc_cali_handle_t adcPhotoResistorHandler;     

/**
 * Defines a debouncing function that will return the value of the button input after a delay of 25MS
 */

bool debounce(int buttonInput){
    int button = gpio_get_level(buttonInput);
    vTaskDelay(25/ portTICK_PERIOD_MS);
    int new = gpio_get_level(buttonInput);
    if (new == button){
        return new;
    }
    return false;
}

/**
 * returns a boolean determining whether all of the car alarms systems have been satisifed ie:
 * driver seat belt, driver is seated etc.
 */
bool IgnitionReady(void){
    bool dSeatBelt = debounce(driverSeatBelt);
    bool pSeatBelt = debounce(passSeatBelt);
    bool dSeat = debounce(driveSeat);
    bool passengerSeat = debounce(passSeat);
    if (dSeatBelt){
        dsbelt = true;
    }
    else{dsbelt = false;}
    if (pSeatBelt){
        psbelt = true;
    }
    else{psbelt = false;}
    if (dSeat){
        dSense = true;
    }
    else{
        dSense = false;
    }
    if (passengerSeat){
        pSense = true;
    }
    else {pSense = false;}
    return (dsbelt && psbelt && dSense && pSense);
    }

    /**
     * Will configure all of the pins within the design resetting all of the pins within the design to a known state
     * setting the direction to either input or output 
     * enabling pullup resistors within the ESP
     * And finally setting all of the output pins to zero
     */

void pinConfig(void){
    gpio_reset_pin(ignitionLED);
    gpio_reset_pin(engineLED);
    gpio_reset_pin(ignitionEn);
    gpio_reset_pin(driverSeatBelt);
    gpio_reset_pin(passSeatBelt);
    gpio_reset_pin(driveSeat);
    gpio_reset_pin(passSeat);
    gpio_reset_pin(Alarm);
    gpio_reset_pin(highBeamsIn);
    gpio_reset_pin(highBeamsOut);
    gpio_reset_pin(headLights);

    gpio_set_direction(ignitionLED, GPIO_MODE_OUTPUT);
    gpio_set_direction(engineLED, GPIO_MODE_OUTPUT);
    gpio_set_direction(Alarm, GPIO_MODE_OUTPUT);
    gpio_set_direction(highBeamsOut, GPIO_MODE_OUTPUT);
    gpio_set_direction(headLights, GPIO_MODE_INPUT_OUTPUT);

    gpio_set_direction(highBeamsIn, GPIO_MODE_INPUT);
    gpio_set_direction(ignitionEn, GPIO_MODE_INPUT);
    gpio_set_direction(driverSeatBelt, GPIO_MODE_INPUT);
    gpio_set_direction(driveSeat, GPIO_MODE_INPUT);
    gpio_set_direction(passSeat, GPIO_MODE_INPUT);
    gpio_set_direction(passSeatBelt, GPIO_MODE_INPUT);

    gpio_pulldown_en(ignitionEn);
    gpio_pulldown_en(driverSeatBelt);
    gpio_pulldown_en(driveSeat);
    gpio_pulldown_en(passSeat);
    gpio_pulldown_en(passSeatBelt);
    gpio_pulldown_en(highBeamsIn);

    gpio_set_level(ignitionLED, 0);
    gpio_set_level(engineLED, 0);
    gpio_set_level(Alarm, 0);
    gpio_set_level(highBeamsOut, 0);
    gpio_set_level(headLights, 0);

}

/**
 * defines a function that will configure the adc conversion within the system
 */
void adcConfig(void){
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config1, &oneShotHandler);
     adc_oneshot_chan_cfg_t config = {
        .atten = adcAtten,
        .bitwidth = bitWidth
    };
    adc_oneshot_config_channel(oneShotHandler, photoResistor, &config);
    adc_oneshot_config_channel(oneShotHandler, poteniometer, &config);

    adc_cali_curve_fitting_config_t caliPhotoConfig = {
        .unit_id = ADC_UNIT_1,
        .chan = photoResistor,
        .atten = adcAtten,
        .bitwidth = bitWidth
    };
        adc_cali_curve_fitting_config_t caliPotenConfig = {
        .unit_id = ADC_UNIT_1,
        .chan = poteniometer,
        .atten = adcAtten,
        .bitwidth = bitWidth
    };
    adc_cali_create_scheme_curve_fitting(&caliPhotoConfig, &adcPhotoResistorHandler);
    adc_cali_create_scheme_curve_fitting(&caliPotenConfig, &adcPotentiometerHandler);
}
/**
 * defines a function that will change the settings of the headlights based on the amount of light, if the user has the auto headlights
 * enabled
 * (nts: photoresistor increases conductivity with darkness, for above 55 is daylight below nighttime )
 */
 void lightSense(int adcMV){
    bool onChecker = gpio_get_level(headLights);
    if (adcMV > 250 && onChecker){
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        gpio_set_level(headLights, 0);
    }
    if (adcMV < 150 && !onChecker){
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(headLights, 1);
    }
    else{
        return;
    }
    }

 /**
  * defines a function that configures the photoresitor
  */
int photoResistorRead(void){
    int adcBitsPhoto;
    int adcMVPhoto;
    adc_oneshot_read(oneShotHandler, photoResistor, &adcBitsPhoto);
    adc_cali_raw_to_voltage(adcPhotoResistorHandler, adcBitsPhoto, &adcMVPhoto); 
    return adcMVPhoto;

}


/**
 * oneshot read for the potentiometer in our system
 */

int potentiometerRead(void){
    int adcBitsPoten;
    int adcMVPoten;
    adc_oneshot_read(oneShotHandler, poteniometer, &adcBitsPoten);
    adc_cali_raw_to_voltage(adcPotentiometerHandler, adcBitsPoten, &adcMVPoten); 
    return adcMVPoten;
}

/**
 * defines a function that will return an integer corresponding to one of the headlight mode the user selects
 * 0 = OFF,
 * 1 = ON,
 * 2 = AUTO
 */
int headlightSelection(int adcMV){
    if (adcMV < 1100){
        return 0;
    }
    else if (adcMV < 1800){
        return 1;
    }
    else{
        return 2;
    }

}
void app_main(void) {
    pinConfig();
    adcConfig();
    bool initial_message = true;
    bool engineRunning = false;
    while(1){
       // printf("iamworking!\n");
        bool ignitEn = debounce(ignitionEn);
        bool ready = IgnitionReady();
        int headlightLevel = headlightSelection(potentiometerRead());
        int photoRead = photoResistorRead();
        // printf("%d\n", photoRead);
        // printf("%d\n", headlightLevel);
        // printf("%d\n", ready);
        // printf("%d\n", ignitEn);
        //printf("%d\n", gpio_get_level(headLights));

    
        if (dSense && initial_message){
            printf("Welcome to enhanced Alarm system model 218 -W25\n");
            initial_message = false;
        }

        if (headlightLevel == 0){
            gpio_set_level(headLights, 0);
        }

        if (headlightLevel == 1){
            gpio_set_level(headLights, 1);

        }
        if (headlightLevel == 2){
            lightSense(photoRead);
        }

        if (gpio_get_level(headLights) && gpio_get_level(highBeamsIn)){
            gpio_set_level(highBeamsOut, 1);
        }
        else{gpio_set_level(highBeamsOut, 0);}
            
        if(ready){
            gpio_set_level(ignitionLED, 1);
        }

        if(ignitEn){
            if (engineRunning){
            printf("engine stopping...\n");
            gpio_set_level(engineLED, 0);
            gpio_set_level(headLights, 0);
            gpio_set_level(highBeamsOut, 0);
            engineRunning = false;
            }
            if (ready) {
                engineRunning = true;
                printf("engine starting...\n");
                gpio_set_level(ignitionLED, 0);
                gpio_set_level(engineLED, 1);
            }
            else{
                gpio_set_level (Alarm, 1);
                if (!dSense){
                    printf("Driver seat not occupied\n");
                }
                
                if (!dsbelt){
                    printf("Driver seatbelt not fastened\n");
                }

                if (!pSense){
                    printf("Passenger seat not occupied\n");
                }
                if (!psbelt){
                    printf("Passenger seatbelt not fastened\n");
                }
                vTaskDelay (500/ portTICK_PERIOD_MS);
            }
        }
        else {
            gpio_set_level (Alarm, 0);
        }
        vTaskDelay(25 / portTICK_PERIOD_MS);

    }
}