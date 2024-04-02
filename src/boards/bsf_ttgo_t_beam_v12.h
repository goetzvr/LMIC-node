/*******************************************************************************
 * 
 *  File:         bsf_ttgo_t_beam_v12.h
 * 
 *  Description:  Board Support File for TTGO V1.2 .
 * 
 *  Copyright:    Copyright (c) 2021 Leonel Lopes Parente
 * 
 *  License:      MIT License. See accompanying LICENSE file.
 * 
 *  Author:       Leonel Lopes Parente, Goetz van Rissenbeck
 * 
 *  Description:  This board has onboard USB (provided by onboard USB to serial).
 *                It supports automatic firmware upload and serial over USB. 
 *                External display is connected.
 *                No onboard user programmable LED..
 * 
 *                This board uses an AXP2101 power management chip to power
 *                onboard components and the +3.3V output pin.
 *                The AXP2101 must be correctly configured for things to work
 *                (see boardInit() below).
 * 
 * 
 *                CONNECTIONS AND PIN DEFINITIONS:
 *                
 *                Indentifiers between parentheses are defined in the board's 
 *                Board Support Package (BSP) which is part of the Arduino core. 
 * 
 *                Leds                  GPIO 
 *                ----                  ----
 *                LED                   -          No onboard user LED
 * 
 *                I2C [display]         GPIO
 *                ----                  ----
 *                SDA     <――――――――――>  21  (SDA)
 *                SCL     <――――――――――>  22  (SCL)
 *                RST                    -
 *
 *                SPI/LoRa              GPIO
 *                ---                   ----
 *                MOSI    <――――――――――>  27  (MOSI) (LORA_MOSI)
 *                MISO    <――――――――――>  19  (MISO) (LORA_MISO)
 *                SCK     <――――――――――>   5  (SCK)  (LORA_SCK)
 *                NSS     <――――――――――>  18  (SS)   (LORA_CS)
 *                RST     <――――――――――>  23         (LORA_RST)
 *                DIO0    <――――――――――>  26         (LORA_IO0)
 *                DIO1    <――――――――――>  33         (LORA_IO1)
 *                DIO2    <――――――――――>  32         (LORA_IO2)
 * 
 *                GPS                   GPIO
 *                ---                   ----
 *                RX      <――――――――――>  34   
 *                TX      <――――――――――>  12  
 *                PPS     <――――――――――>  37
 * 
 *                Power Management      GPIO
 *                -----                 ----
 *                PMU     <――――――――――>  35
 *  
 *                Button switches       GPIO 
 *                ------                ----
 *                USR_SW  <――――――――――>  39  (KEY_BUILTIN)
 * 
 *  Docs:         https://docs.platformio.org/en/latest/boards/espressif32/ttgo-t-beam.html
 *
 *  Identifiers:  LMIC-node
 *                    board:         ttgo_tbeam_v1
 *                PlatformIO
 *                    board:         ttgo-t-beam
 *                    platform:      espressif32
 *                Arduino
 *                    board:         ARDUINO_T_Beam
 *                    architecture:  ARDUINO_ARCH_ESP32
 * 
 ******************************************************************************/

#pragma once

#ifndef BSF_TTGO_T_BEAM_V1_2_H_
#define BSF_TTGO_T_BEAM_V1_2_H_

// Defined using AXP2102
#define XPOWERS_CHIP_AXP2101
#include "XPowersAXP2101.tpp"
#include "XPowersLibInterface.hpp"

#include "LMIC-node.h"

#define DEVICEID_DEFAULT "ttgo-tbeam-v1.2"  // Default deviceid value 

// Wait for Serial
// Can be useful for boards with MCU with integrated USB support.
// #define WAITFOR_SERIAL_SECONDS_DEFAULT 10   // -1 waits indefinitely  

// LMIC Clock Error
// This is only needed for slower 8-bit MCUs (e.g. 8MHz ATmega328 and ATmega32u4).
// Value is defined in parts per million (of MAX_CLOCK_ERROR).
// #ifndef LMIC_CLOCK_ERROR_PPM
//     #define LMIC_CLOCK_ERROR_PPM 0
// #endif   

// Pin mappings for LoRa tranceiver
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,
    .dio = { /*dio0*/ 26, /*dio1*/ 33, /*dio2*/ 32 }
#ifdef MCCI_LMIC
    ,
    .rxtx_rx_active = 0,
    .rssi_cal = 10,
    .spi_freq = 8000000     /* 8 MHz */
#endif    
};

XPowersLibInterface *PMU = NULL;
bool  pmu_flag = 0;


#ifdef USE_SERIAL
    HardwareSerial& serial = Serial;
#endif                                        

#ifdef USE_LED
    // #error Invalid option: USE_LED. This board has no onboard user LED.
    // EasyLed led(<external LED GPIO>, EasyLed::ActiveLevel::Low);
#endif

#ifdef USE_DISPLAY
    // Create U8x8 instance for SSD1306 OLED display (no reset) using hardware I2C.
    U8X8_SSD1306_128X64_NONAME_HW_I2C display(/*rst*/ U8X8_PIN_NONE, /*scl*/ SCL, /*sda*/ SDA);
#endif


bool initPMU() {
    if (!PMU) {
        PMU = new XPowersAXP2101(  Wire );
        if (!PMU->init()) {
            Serial.println("Warning: Failed to find AXP2101 power management");
            delete PMU;
            PMU = NULL;
        } else {
            Serial.println("AXP2101 PMU init succeeded, using AXP2101 PMU");
        }
    }

    if (!PMU) {
        return false;
    }

    PMU->setChargingLedMode(XPOWERS_CHG_LED_BLINK_1HZ);

    // pinMode(CONFIG_PMU_IRQ, INPUT_PULLUP);
    // attachInterrupt(CONFIG_PMU_IRQ, pmu_flag, FALLING);
 if (PMU->getChipModel() == XPOWERS_AXP2101) {

#if defined(CONFIG_IDF_TARGET_ESP32)
        //Unuse power channel   
        PMU->disablePowerOutput(XPOWERS_DCDC2);
        PMU->disablePowerOutput(XPOWERS_DCDC3);
        PMU->disablePowerOutput(XPOWERS_DCDC4);
        PMU->disablePowerOutput(XPOWERS_DCDC5);
        PMU->disablePowerOutput(XPOWERS_ALDO1);
        PMU->disablePowerOutput(XPOWERS_ALDO4);
        PMU->disablePowerOutput(XPOWERS_BLDO1);
        PMU->disablePowerOutput(XPOWERS_BLDO2);
        PMU->disablePowerOutput(XPOWERS_DLDO1);
        PMU->disablePowerOutput(XPOWERS_DLDO2);

        // GNSS RTC PowerVDD 3300mV
        PMU->setPowerChannelVoltage(XPOWERS_VBACKUP, 3300);
        PMU->enablePowerOutput(XPOWERS_VBACKUP);

        //ESP32 VDD 3300mV
        // ! No need to set, automatically open , Don't close it
        // PMU->setPowerChannelVoltage(XPOWERS_DCDC1, 3300);
        // PMU->setProtectedChannel(XPOWERS_DCDC1);
        PMU->setProtectedChannel(XPOWERS_DCDC1);

        // LoRa VDD 3300mV
        PMU->setPowerChannelVoltage(XPOWERS_ALDO2, 3300);
        PMU->enablePowerOutput(XPOWERS_ALDO2);

        //GNSS VDD 3300mV
        PMU->setPowerChannelVoltage(XPOWERS_ALDO3, 3300);
        PMU->enablePowerOutput(XPOWERS_ALDO3);

#endif /*CONFIG_IDF_TARGET_ESP32*/


#if defined(LILYGO_TBeamS3_SUPREME_V3_0)

        //t-beam m.2 inface
        //gps
        PMU->setPowerChannelVoltage(XPOWERS_ALDO4, 3300);
        PMU->enablePowerOutput(XPOWERS_ALDO4);

        // lora
        PMU->setPowerChannelVoltage(XPOWERS_ALDO3, 3300);
        PMU->enablePowerOutput(XPOWERS_ALDO3);

        // In order to avoid bus occupation, during initialization, the SD card and QMC sensor are powered off and restarted
        if (ESP_SLEEP_WAKEUP_UNDEFINED == esp_sleep_get_wakeup_cause()) {
            Serial.println("Power off and restart ALDO BLDO..");
            PMU->disablePowerOutput(XPOWERS_ALDO1);
            PMU->disablePowerOutput(XPOWERS_ALDO2);
            PMU->disablePowerOutput(XPOWERS_BLDO1);
            delay(250);
        }

        // Sensor
        PMU->setPowerChannelVoltage(XPOWERS_ALDO1, 3300);
        PMU->enablePowerOutput(XPOWERS_ALDO1);

        PMU->setPowerChannelVoltage(XPOWERS_ALDO2, 3300);
        PMU->enablePowerOutput(XPOWERS_ALDO2);

        //Sdcard

        PMU->setPowerChannelVoltage(XPOWERS_BLDO1, 3300);
        PMU->enablePowerOutput(XPOWERS_BLDO1);

        PMU->setPowerChannelVoltage(XPOWERS_BLDO2, 3300);
        PMU->enablePowerOutput(XPOWERS_BLDO2);

        //face m.2
        PMU->setPowerChannelVoltage(XPOWERS_DCDC3, 3300);
        PMU->enablePowerOutput(XPOWERS_DCDC3);

        PMU->setPowerChannelVoltage(XPOWERS_DCDC4, XPOWERS_AXP2101_DCDC4_VOL2_MAX);
        PMU->enablePowerOutput(XPOWERS_DCDC4);

        PMU->setPowerChannelVoltage(XPOWERS_DCDC5, 3300);
        PMU->enablePowerOutput(XPOWERS_DCDC5);


        //not use channel
        PMU->disablePowerOutput(XPOWERS_DCDC2);
        // PMU->disablePowerOutput(XPOWERS_DCDC4);
        // PMU->disablePowerOutput(XPOWERS_DCDC5);
        PMU->disablePowerOutput(XPOWERS_DLDO1);
        PMU->disablePowerOutput(XPOWERS_DLDO2);
        PMU->disablePowerOutput(XPOWERS_VBACKUP);

        // Set constant current charge current limit
        PMU->setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_500MA);

        // Set charge cut-off voltage
        PMU->setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);

        // Disable all interrupts
        PMU->disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
        // Clear all interrupt flags
        PMU->clearIrqStatus();
        // Enable the required interrupt function
        PMU->enableIRQ(
            XPOWERS_AXP2101_BAT_INSERT_IRQ    | XPOWERS_AXP2101_BAT_REMOVE_IRQ      |   //BATTERY
            XPOWERS_AXP2101_VBUS_INSERT_IRQ   | XPOWERS_AXP2101_VBUS_REMOVE_IRQ     |   //VBUS
            XPOWERS_AXP2101_PKEY_SHORT_IRQ    | XPOWERS_AXP2101_PKEY_LONG_IRQ       |   //POWER KEY
            XPOWERS_AXP2101_BAT_CHG_DONE_IRQ  | XPOWERS_AXP2101_BAT_CHG_START_IRQ       //CHARGE
            // XPOWERS_AXP2101_PKEY_NEGATIVE_IRQ | XPOWERS_AXP2101_PKEY_POSITIVE_IRQ   |   //POWER KEY
        );

#endif
    }

    PMU->enableSystemVoltageMeasure();
    PMU->enableVbusVoltageMeasure();
    PMU->enableBattVoltageMeasure();
    // It is necessary to disable the detection function of the TS pin on the board
    // without the battery temperature detection function, otherwise it will cause abnormal charging
    PMU->disableTSPinMeasure();

    Serial.printf("=========================================\n");
    if (PMU->isChannelAvailable(XPOWERS_DCDC1)) {
        Serial.printf("DC1  : %s   Voltage: %04u mV \n",  PMU->isPowerChannelEnable(XPOWERS_DCDC1)  ? "+" : "-",  PMU->getPowerChannelVoltage(XPOWERS_DCDC1));
    }
    if (PMU->isChannelAvailable(XPOWERS_DCDC2)) {
        Serial.printf("DC2  : %s   Voltage: %04u mV \n",  PMU->isPowerChannelEnable(XPOWERS_DCDC2)  ? "+" : "-",  PMU->getPowerChannelVoltage(XPOWERS_DCDC2));
    }
    if (PMU->isChannelAvailable(XPOWERS_DCDC3)) {
        Serial.printf("DC3  : %s   Voltage: %04u mV \n",  PMU->isPowerChannelEnable(XPOWERS_DCDC3)  ? "+" : "-",  PMU->getPowerChannelVoltage(XPOWERS_DCDC3));
    }
    if (PMU->isChannelAvailable(XPOWERS_DCDC4)) {
        Serial.printf("DC4  : %s   Voltage: %04u mV \n",  PMU->isPowerChannelEnable(XPOWERS_DCDC4)  ? "+" : "-",  PMU->getPowerChannelVoltage(XPOWERS_DCDC4));
    }
    if (PMU->isChannelAvailable(XPOWERS_DCDC5)) {
        Serial.printf("DC5  : %s   Voltage: %04u mV \n",  PMU->isPowerChannelEnable(XPOWERS_DCDC5)  ? "+" : "-",  PMU->getPowerChannelVoltage(XPOWERS_DCDC5));
    }
    if (PMU->isChannelAvailable(XPOWERS_LDO2)) {
        Serial.printf("LDO2 : %s   Voltage: %04u mV \n",  PMU->isPowerChannelEnable(XPOWERS_LDO2)   ? "+" : "-",  PMU->getPowerChannelVoltage(XPOWERS_LDO2));
    }
    if (PMU->isChannelAvailable(XPOWERS_LDO3)) {
        Serial.printf("LDO3 : %s   Voltage: %04u mV \n",  PMU->isPowerChannelEnable(XPOWERS_LDO3)   ? "+" : "-",  PMU->getPowerChannelVoltage(XPOWERS_LDO3));
    }
    if (PMU->isChannelAvailable(XPOWERS_ALDO1)) {
        Serial.printf("ALDO1: %s   Voltage: %04u mV \n",  PMU->isPowerChannelEnable(XPOWERS_ALDO1)  ? "+" : "-",  PMU->getPowerChannelVoltage(XPOWERS_ALDO1));
    }
    if (PMU->isChannelAvailable(XPOWERS_ALDO2)) {
        Serial.printf("ALDO2: %s   Voltage: %04u mV \n",  PMU->isPowerChannelEnable(XPOWERS_ALDO2)  ? "+" : "-",  PMU->getPowerChannelVoltage(XPOWERS_ALDO2));
    }
    if (PMU->isChannelAvailable(XPOWERS_ALDO3)) {
        Serial.printf("ALDO3: %s   Voltage: %04u mV \n",  PMU->isPowerChannelEnable(XPOWERS_ALDO3)  ? "+" : "-",  PMU->getPowerChannelVoltage(XPOWERS_ALDO3));
    }
    if (PMU->isChannelAvailable(XPOWERS_ALDO4)) {
        Serial.printf("ALDO4: %s   Voltage: %04u mV \n",  PMU->isPowerChannelEnable(XPOWERS_ALDO4)  ? "+" : "-",  PMU->getPowerChannelVoltage(XPOWERS_ALDO4));
    }
    if (PMU->isChannelAvailable(XPOWERS_BLDO1)) {
        Serial.printf("BLDO1: %s   Voltage: %04u mV \n",  PMU->isPowerChannelEnable(XPOWERS_BLDO1)  ? "+" : "-",  PMU->getPowerChannelVoltage(XPOWERS_BLDO1));
    }
    if (PMU->isChannelAvailable(XPOWERS_BLDO2)) {
        Serial.printf("BLDO2: %s   Voltage: %04u mV \n",  PMU->isPowerChannelEnable(XPOWERS_BLDO2)  ? "+" : "-",  PMU->getPowerChannelVoltage(XPOWERS_BLDO2));
    }
    Serial.printf("=========================================\n");


    // Set the time of pressing the button to turn off
    PMU->setPowerKeyPressOffTime(XPOWERS_POWEROFF_4S);
    uint8_t opt = PMU->getPowerKeyPressOffTime();
    Serial.print("PowerKeyPressOffTime:");
    switch (opt) {
    case XPOWERS_POWEROFF_4S: Serial.println("4 Second");
        break;
    case XPOWERS_POWEROFF_6S: Serial.println("6 Second");
        break;
    case XPOWERS_POWEROFF_8S: Serial.println("8 Second");
        break;
    case XPOWERS_POWEROFF_10S: Serial.println("10 Second");
        break;
    default:
        break;
    }

    return true;
}


bool boardInit(InitType initType)
{
    // This function is used to perform board specific initializations.
    // Required as part of standard template.

    // InitType::Hardware        Must be called at start of setup() before anything else.
    // InitType::PostInitSerial  Must be called after initSerial() before other initializations.    

    bool success = true;
    switch (initType)
    {
        case InitType::Hardware:
            // Note: Serial port and display are not yet initialized and cannot be used use here.

            // This board contains an AXP2101 power management chip.
            // Power output must be enabled for several components before they can be initialized.
            Wire.begin(SDA, SCL);

            success = initPMU();

            break;

        case InitType::PostInitSerial:
            // Note: If enabled Serial port and display are already initialized here.
            // No actions required for this board.
            break;      
    }
    return success;
}


#endif  // BSF_TTGO_T_BEAM_V1_2_H_
