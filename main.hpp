#ifndef MAIN
#define MAIN


#include <cstdint>
#include <ctime>
#include <mbed.h>
#include <string>
#include "ble/BLE.h"
#include "ble/gap/Gap.h"
#include "ble/services/HeartRateService.h"
#include "ble/services/EnergyHarvestingService.h"
#include "ble/services/HealthThermometerService.h"
#include "ble/services/UARTService.h"




class EnergyHarvesting: ble::Gap::EventHandler{

    private:
        BLE &_ble;
        events::EventQueue &_event_queue;
        DigitalOut _led1;
        DigitalIn _sdaCoulomnetre;
        DigitalOut _sclCoulomnetre;
        DigitalOut _ALCoulomnetre;
        DigitalIn _sleepMode;
        DigitalIn _REF_TEG;
        DigitalIn _DATA_TEG;
        DigitalOut _enable;
        AnalogIn _BATT_CHG;
      //  AnalogIn _BATT_CON;
        AnalogIn _adc_tension1;
        DigitalIn _reset;
        DigitalOut _scl_i2c_2;
        DigitalOut _sda_i2c_2;
        DigitalIn _int_i2c_2;
        AnalogIn _adc_current1;
        AnalogIn _adc_tension2;
        AnalogIn _adc_current2;
        bool _connected;
        UUID _hr_uuid;
        uint8_t _hr_counter;
        float _testTemp;
        UARTService _hr_service;
        UARTService _ltc;
        UARTService _adxl;
        UARTService _tmp;
        UARTService _testUARTBLE;
        HeartRateService _test0;
        HeartRateService _test1;
        uint8_t _adv_buffer[ble::LEGACY_ADVERTISING_MAX_SIZE];
        ble::AdvertisingDataBuilder _adv_data_builder;

    public:
    EnergyHarvesting(BLE &ble, events::EventQueue &event_queue) :
        _ble(ble),
        _event_queue(event_queue),
        _led1(LED1, 1),
        _sdaCoulomnetre(p8),
        _sclCoulomnetre(p11,1),
        _ALCoulomnetre(p9),
        _REF_TEG(p13),
        _DATA_TEG(p14),
        _sleepMode(p29),
        _enable(p16),
        _BATT_CHG(p28),
       // _BATT_CON(p29),
        _adc_tension1(p30),
        _reset(p21),
        _scl_i2c_2(p31,1),
        _sda_i2c_2(p7,1),
        _int_i2c_2(p6),
        _adc_current1(p4),
        _adc_tension2(p2),
        _adc_current2(p3),
        _connected(false),
        _hr_uuid(GattService::UUID_HEART_RATE_SERVICE),
        _hr_counter(100),
        _testTemp(10),
        _hr_service(ble),
        _ltc(ble),
        _adxl(ble),
        _tmp(ble),
        _testUARTBLE(ble),
        _test0(ble, 150, HeartRateService::LOCATION_CHEST),
        _test1(ble, 150, HeartRateService::LOCATION_CHEST),
        _adv_data_builder(_adv_buffer) { }

    public:
        void start();
        bool initI2C_interface(int i2cToInit, int frequency);
        bool initI2c_1(int frequency);
        void sendDataOtherBLE(uint8_t id, const void * data, int lengthData);
        bool initI2c_2(int frequency);
        void readADC();
        void testTimer();
        void start_advertising();
        void update_sensor_value();
        void testI2C_oled();
        void testi2c1();
        void testi2c2();
        void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context);
        void on_init_complete(BLE::InitializationCompleteCallbackContext *params);
        void blink();
        void send();
        void stopPeripherals();

};

#endif