/**********************************************************************
*
* Copyright (c) 2023 Tinkerbug Robotics
*
* This program is free software: you can redistribute it and/or modify it under the terms
* of the GNU General Public License as published by the Free Software Foundation, either
* version 3 of the License, or (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful, but WITHOUT ANY
* WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
* PARTICULAR PURPOSE. See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with this 
* program. If not, see <https://www.gnu.org/licenses/>.
* 
* Authors: 
* Christian Pedersen; tinkerbug@tinkerbugrobotics.com
* 
**********************************************************************/

#ifndef MAX17055_TR_h
#define MAX17055_TR_h

#include <Arduino.h>

class MAX17055
{
  public:
  
    // Register addresses 
    enum regAddr
    {
        status          = 0x00, // Flags for alert thresholds and battery insertion

        // Calculated values
        temp            = 0x08, // Internal temperature value (deg C)
        voltage         = 0x09, // Instantaneous voltage between BATT and CSP
        avg_voltage     = 0x19, // Average voltage  
        current         = 0x0A, // Current
        avg_current     = 0x0B, // Average Current
        SOC             = 0x06, // State of Charge of battery
        calc_capacity   = 0x05, // Calculated capacity
        full_capacity   = 0x10, // Calculated capacity at full, reduced by battery age
        time_to_empty   = 0x11, // Time (ms) till battery is empty
        battery_age     = 0x07, // Battery age calculated capacity / original capacity
        charge_cycle    = 0x17, // Number of charge cycle

        // Inputs/settings for MAX17055
        input_capacity                   = 0x18, // Input capacity of battery
        input_empty_voltage              = 0x3A, // Voltage when pack is empty
        input_recovery_voltage           = 0x7F, // Voltage when pack is fully charged
        input_charge_termination_current = 0x1E, // Current at which charging stops
    };
    
    // Constructor
    MAX17055();
    
    // Get calculated values
    float getTemp();
    float getInstantaneousVoltage();
    float getAvgVoltage();
    float getInstantaneousCurrent();
    float getAvgCurrent();
    float getCalculatedCapacity();
    float getSOC();
    float getTimeToEmpty();
    float getBatteryAge();
    float getChargeCycle();
    
    // Get values for inputs/settings
    float getResistSensor();
    float getInputCapacity();
    float getEmptyVoltage();
    
    // Set inputs
    void setCapacity(uint16_t batteryCapacity);
    void setResistSensor(float resistorValue); 
    void setEmptyVoltage(float empty_voltage);
    void setRecoveryVoltage(float recovery_voltage); 
    void setChargeTermination(float charge_termination); 


private:
    //variables
    float resist_sensor = 0.01;
    uint8_t I2CAddress = 0x36;
    
    //Based on "Standard Register Formats" AN6358, figure 1.3. 
    //Multipliers are constants used to multiply register value in order to get final result
    float capacity_multiplier_mAH = (5e-3)/resist_sensor;
    float current_multiplier_mA = (1.5625e-3)/resist_sensor;
    float voltage_multiplier_V = 7.8125e-5;
    float time_multiplier = 5.625;
    float percentage_multiplier = 1.0/256.0;
    
    //methods
    uint16_t readReg16Bit(uint8_t reg);
    void writeReg16Bit(uint8_t reg, uint16_t value);
   };

#endif



