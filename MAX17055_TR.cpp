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

#include <MAX17055_TR.h>
#include <Wire.h>




// Constructors 
MAX17055::MAX17055()
{
	Wire1.begin();
}

// Public Methods

// Methods to get calculated values
float MAX17055::getTemp()
{
   	// uint16_t capacity_raw = readReg16Bit(RepCap);
   	uint16_t temp_raw = readReg16Bit(temp);
	return (temp_raw * percentage_multiplier);
}
float MAX17055::getInstantaneousVoltage()
{
   	uint16_t voltage_raw = readReg16Bit(voltage);
	return voltage_raw * voltage_multiplier_V;
}
float MAX17055::getAvgVoltage()
{
   	uint16_t avg_voltage_raw = readReg16Bit(avg_voltage);
	return avg_voltage_raw * voltage_multiplier_V;
}
float MAX17055::getInstantaneousCurrent()
{
   	int16_t current_raw = readReg16Bit(current);
	return current_raw * current_multiplier_mA;
}
float MAX17055::getAvgCurrent()
{
   	int16_t avg_current_raw = readReg16Bit(avg_current);
	return avg_current_raw * current_multiplier_mA;
}
float MAX17055::getCalculatedCapacity()
{
   	uint16_t calc_capacity_raw = readReg16Bit(calc_capacity);
	return (calc_capacity_raw * capacity_multiplier_mAH);
}

// Return state of charge in percent
float MAX17055::getSOC()
{
   	uint16_t SOC_raw = readReg16Bit(SOC);
	return SOC_raw * percentage_multiplier;
}

// Return time till empty under current temperature and load in seconds
float MAX17055::getTimeToEmpty()
{
	uint16_t time_to_empty_raw = readReg16Bit(time_to_empty);
	return time_to_empty_raw * time_multiplier;
}
float MAX17055::getBatteryAge()
{
	uint16_t battery_age_raw = readReg16Bit(battery_age);
	return (battery_age_raw * percentage_multiplier);
}
float MAX17055::getChargeCycle()
{
	uint16_t charge_cycle_raw = readReg16Bit(charge_cycle);
	return (charge_cycle_raw/655.35);
}

// Methods to get input values
float MAX17055::getResistSensor()
{
	return resist_sensor;
}
float MAX17055::getInputCapacity()
{
   	uint16_t input_capacity_raw = readReg16Bit(input_capacity);
	return (input_capacity_raw * capacity_multiplier_mAH);
}
float MAX17055::getEmptyVoltage()
{
   	uint16_t input_empty_voltage_raw = readReg16Bit(input_empty_voltage);
	return (input_empty_voltage_raw * voltage_multiplier_V);
}

// Methods to set inputs
void MAX17055::setCapacity(uint16_t capacity)
{
	writeReg16Bit(input_capacity, (capacity/capacity_multiplier_mAH));	
}
void MAX17055::setResistSensor(float resistor_value)
{
	resist_sensor = resistor_value;
}
void MAX17055::setEmptyVoltage(float empty_voltage)
{
	writeReg16Bit(input_empty_voltage, (empty_voltage/voltage_multiplier_V));	
}
void MAX17055::setRecoveryVoltage(float recovery_voltage)
{
	writeReg16Bit(input_recovery_voltage, (recovery_voltage/voltage_multiplier_V));	
}
void MAX17055::setChargeTermination(float charge_termination_current)
{
	writeReg16Bit(input_charge_termination_current, (charge_termination_current/current_multiplier_mA)*2);	
}

// Private Methods

void MAX17055::writeReg16Bit(uint8_t reg, uint16_t value)
{
  //Write order is LSB first, and then MSB
  Wire1.beginTransmission(I2CAddress);
  Wire1.write(reg);
  Wire1.write( value       & 0xFF); // value low byte
  Wire1.write((value >> 8) & 0xFF); // value high byte
  uint8_t last_status = Wire1.endTransmission();
}

uint16_t MAX17055::readReg16Bit(uint8_t reg)
{
  uint16_t value = 0;  
  Wire1.beginTransmission(I2CAddress); 
  Wire1.write(reg);
  uint8_t last_status = Wire1.endTransmission(false);
  
  Wire1.requestFrom(I2CAddress, (uint8_t) 2); 
  value  = Wire1.read();
  value |= (uint16_t)Wire1.read() << 8;      // value low byte
  return value;
}