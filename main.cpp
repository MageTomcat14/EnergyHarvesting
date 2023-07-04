#include <cstdio>
#include <exception>
#include <mbed.h>
#include <events/mbed_events.h>
#include "ble/BLE.h"
#include "ble/gap/Gap.h"
#include "ble/services/HeartRateService.h"
#include "ble/services/EnergyHarvestingService.h"
#include "ble/services/UARTService.h"
#include "main.hpp"
#include "print.h"
#include "hal/i2c_api.h"
#include "hal/can_api.h"
#include <stdio.h>
#include "LittleFileSystem.h"


#define uartDebug(f_, ...) printf((f_), ##__VA_ARGS__)
BlockDevice *bd = BlockDevice::get_default_instance();

i2c_t i2c_1, i2c_2;
LittleFileSystem fs("fs");

I2C TmpAdxl(p7,p31);
I2C coulomnetre(p8,p11);

Timer timerLTC;

char buffer_i2c[50];

static events::EventQueue event_queue(/* event count */ 16 * EVENTS_EVENT_SIZE);

void EnergyHarvesting::schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
}

void EnergyHarvesting::start(){

    _ble.gap().setEventHandler(this);
    _ble.init(this, &EnergyHarvesting::on_init_complete);

    // send the data to the BLE
    _event_queue.call_every(1000, this, &EnergyHarvesting::blink);
    _event_queue.call_every(1000, this, &EnergyHarvesting::testi2c1);
    _event_queue.call_every(1000, this, &EnergyHarvesting::testi2c2);
    _event_queue.call_every(1000, this, &EnergyHarvesting::readADC);
    _event_queue.call_every(1000, this, &EnergyHarvesting::testTimer);
    _event_queue.dispatch_forever();

}



bool EnergyHarvesting::initI2C_interface(int i2cToInit, int frequency){
    
    switch (i2cToInit) {
    case 1 :
        coulomnetre.frequency(frequency);
        coulomnetre.start();
        break;
    case 2 : 
        TmpAdxl.frequency(frequency);
        TmpAdxl.start();
    }
    return true;
}



/*
* function will send data other BLE using UART service from
* mbed BLE_API 
*/
void EnergyHarvesting::sendDataOtherBLE(uint8_t id, const void * data, int lengthData){

    switch (id) {
    case 1 : // value of LED
        _hr_service.write(data, lengthData);
        _hr_service.flush();
        break;
    case 2 : // value of potard
        _testUARTBLE.write(data, lengthData);
        _testUARTBLE.write("V\r\n", sizeof("V\r\n"));
        _testUARTBLE.flush();
        break;
    case 3 : // value of accelerometer
        _adxl.write(data, lengthData);
        _adxl.write("m/s2", sizeof("m/s2"));
        _adxl.flush();
        break;
    case 4 : // value of ltc sensor
        _ltc.write(data, lengthData);
        _ltc.write("Coulomb", sizeof("Coulomb"));
        _ltc.flush();
        break;
    case 5 : // value of tmp105 sensor
        _tmp.write(data, lengthData);
        _tmp.write("°C", sizeof("°C"));
        _tmp.flush();
        break;
    }
}


void EnergyHarvesting::readADC(){
    float value;
    char valueToString[10];
    value = _adc_tension1.read();

    uartDebug("value of portard : %f \r\n", value); 
    sprintf(valueToString, "%g", value);
    _test1.updateHeartRate(150);
    sendDataOtherBLE(2, valueToString, sizeof(valueToString));
    sendDataOtherBLE(3, "150", sizeof("150"));
    uartDebug("value of accelerometer : %s \r\n", "150"); 
    sendDataOtherBLE(4, "10", sizeof("10"));
    uartDebug("value of LTC :  %s \r\n", "10"); 
    sendDataOtherBLE(5, "15.0", sizeof("15.0"));
    uartDebug("value of TMP : %s \r\n", "15.0");

}

void EnergyHarvesting::testTimer(){
    timerLTC.start();
    //printf("timer has started \r\n");
    if(timerLTC.read_ms() == 2){
      //  printf("the timer has reach 2 ms. Real time %d \r\n", timerLTC.read_ms());
    }
}

void EnergyHarvesting::stopPeripherals(){
    _led1 = 0;
    coulomnetre.stop();
    TmpAdxl.stop();
            
}
void EnergyHarvesting::testi2c1(){
    //uartDebug("testing coulomnetre i2c\r\n");
    const int addr = 0x64 << 1;

    char begin[1];
    char registerC[1];
    char slaveByte[1];
    char readADXL[2];
    char read[2];
    begin[0] = 0X00;
    registerC[0] = 0x00;

    coulomnetre.frequency(200000);
    
    // read from TMP 
    coulomnetre.write(addr,begin,sizeof(begin));
    coulomnetre.write(addr,registerC,sizeof(registerC));
    coulomnetre.write(addr,begin,sizeof(begin));
    coulomnetre.read(addr,read,2);
}

void EnergyHarvesting::testi2c2(){

    const int addrTmp = 0x73 << 1;
    const int addrADXL = 0x53 << 1; // 8 bits I2C 
  
    char beginTMP[1];
    char registerTMP[1];
    char slaveByte[1];
    char read[2];
    beginTMP[0] = 0x49;
    registerTMP[0] = 0x00;


    char beginADXL[1];
    char registerADXL[1];
    char slaveByteADXL[1];
    char readADXL[2];
    beginADXL[0] = 0X00;
    registerADXL[0] = 0x00;

    TmpAdxl.frequency(200000);
    // read from TMP 
    TmpAdxl.write(addrTmp,beginTMP,sizeof(beginTMP));
    TmpAdxl.write(addrTmp,registerTMP,sizeof(registerTMP));
    TmpAdxl.write(addrTmp,beginTMP,sizeof(beginTMP));
    TmpAdxl.read(addrTmp,read,2);


    // read from ADXL 
    TmpAdxl.write(addrADXL,beginADXL,sizeof(beginADXL));
    TmpAdxl.write(addrADXL,registerADXL,sizeof(registerADXL));
    TmpAdxl.read(addrADXL,readADXL,2);

    if(_sleepMode == false){
        //uartDebug("button pressed \r\n");
        stopPeripherals();
        _led1 = 0;
    }
}

void EnergyHarvesting::blink(){
    char valueOfLEDToString[10];
    _led1 = !_led1;
    _test0.updateHeartRate(_led1);
    double valueLed = _led1;
    sprintf(valueOfLEDToString, "%g", valueLed);
    uartDebug("value of LED : %s \r\n", valueOfLEDToString); 
    sendDataOtherBLE(1, valueOfLEDToString, sizeof(valueOfLEDToString));

   /* _hr_service.write(valueOfLEDToString, sizeof(valueOfLEDToString));
    _hr_service.flush(); */
    if(_sleepMode == false){
       // uartDebug("button pressed \r\n");
        stopPeripherals();
    }
}

void EnergyHarvesting::on_init_complete(BLE::InitializationCompleteCallbackContext *params){
    if (params->error != BLE_ERROR_NONE) {
        uartDebug("Initialisation failed\r\n");
        return;
    }
    uartDebug("initialisation done\r\n");
    print_mac_address();
    start_advertising();

}


void EnergyHarvesting::start_advertising() {
    
    const static char DEVICE_NAME[] = "energy harvesting";

    ble::AdvertisingParameters adv_parameters(
        ble::advertising_type_t::CONNECTABLE_UNDIRECTED,
        ble::adv_interval_t(ble::millisecond_t(1000))
    );

    _adv_data_builder.setFlags();
    _adv_data_builder.setAppearance(ble::adv_data_appearance_t::UNKNOWN);
    _adv_data_builder.setLocalServiceList(mbed::make_Span(&_hr_uuid, 1));
    _adv_data_builder.setName(DEVICE_NAME);

    ble_error_t error = _ble.gap().setAdvertisingParameters(
            ble::LEGACY_ADVERTISING_HANDLE,
            adv_parameters
    );

    if (error) {
        uartDebug("_ble.gap().setAdvertisingParameters() failed\r\n");
        return;
    }

    error = _ble.gap().setAdvertisingPayload(
        ble::LEGACY_ADVERTISING_HANDLE,
        _adv_data_builder.getAdvertisingData()
    );

    if (error) {
        uartDebug("_ble.gap().setAdvertisingPayload() failed\r\n");
        return;
    }

    error = _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);

    if (error) {
        uartDebug("_ble.gap().startAdvertising() failed\r\n");
        return;
    }
}

void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
}


int main(){

    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(schedule_ble_events);
    
    EnergyHarvesting demo(ble, event_queue);


 /*   FILE *file = fopen("/test.txt", "w");
    if(file == NULL){
        printf("problem in creating file\r\n");
        return 1;
    }

    fprintf(file, "test file creation\r\n");
    fclose(file);*/



    bool initok_i2c_1 = demo.initI2C_interface(1, 200000); // coulomnetre
    bool initok_i2c_2 = demo.initI2C_interface(2, 200000); // acceleromètre & thermo
   // uartDebug("waiting for debugging print...\r\n");
    if( initok_i2c_1 == true && initok_i2c_2 == true){
        demo.start(); // start to communicate with BLE
    }
}