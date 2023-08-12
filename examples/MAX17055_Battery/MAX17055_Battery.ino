#include <MAX17055_TR.h>
#include <Wire.h>

// I2C pins
#define SDA 26
#define SCL 27

MAX17055 max17055;

void setup() 
{
    Serial.begin(115200);

    // Set I2C pins for communicating with MAX17055
    Wire1.setSDA(SDA);
    Wire1.setSCL(SCL);
    Wire1.begin();

    // Configure MAX17055
    max17055.setResistSensor(0.01); 
    max17055.setCapacity(4400);
    max17055.setChargeTermination(44);
    max17055.setEmptyVoltage(3.3);
}

void loop() 
{
      // Read data and print to USB
      Serial.print("Voltage: ");Serial.println(max17055.getInstantaneousVoltage());
      Serial.print("Avg Voltage: ");Serial.println(max17055.getAvgVoltage());
      Serial.print("Current: ");Serial.println(max17055.getInstantaneousCurrent());
      Serial.print("Avg Current: ");Serial.println(max17055.getAvgCurrent());
      Serial.print("Battery Capactity: ");Serial.println(max17055.getCalculatedCapacity());
      Serial.print("Battery Age: ");Serial.println(max17055.getBatteryAge());
      Serial.print("Number of Cycles: ");Serial.println(max17055.getChargeCycle());
      Serial.print("SOC: ");Serial.println(max17055.getSOC());
      Serial.print("Temperature (C): ");Serial.println(max17055.getTemp());

      delay(5000);
}
