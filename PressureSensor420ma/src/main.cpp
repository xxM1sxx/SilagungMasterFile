/***********************************************************
 DFRobot Gravity: Analog Current to Voltage Converter(For 4~20mA Application)
 SKU:SEN0262

 GNU Lesser General Public License.
 See <http://www.gnu.org/licenses/> for details.
 All above must be included in any redistribution
 ****************************************************/
#include <Arduino.h>

#define CurrentSensorPin  13
#define VREF 3300 // ADC's reference voltage on your Arduino,typical value:5000mV

unsigned int voltage; //unit:mV
float current;  //unit:mA

void setup()
{
   Serial.begin(115200);
}

void loop()
{
    voltage = analogRead(CurrentSensorPin)/4095.0*VREF;
    Serial.print("voltage:");
    Serial.print(voltage);
    Serial.print("mV  ");
    current = voltage/120.0;  //Sense Resistor:120ohm
    Serial.print("current:");
    Serial.print(current);
    Serial.println("mA");
    delay(2000);
}
