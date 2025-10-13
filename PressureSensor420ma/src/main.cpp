#include <Arduino.h>
#define CurrentSensorPin  13
#define VREF 3300 // ADC's reference voltage on ESP32-S3: 3300mV
#define SAMPLES 20   // Increased samples for better averaging
#define FILTER_ALPHA 0.1  // Low-pass filter coefficient (0.1 = heavy filtering)

// Pressure sensor specifications: 0-10 Bar, 4-20mA
#define MIN_CURRENT 4.0   // 4mA = 0 Bar
#define MAX_CURRENT 20.0  // 20mA = 10 Bar
#define MAX_PRESSURE 10.0 // 10 Bar

unsigned long voltageSum = 0; // Sum for averaging
unsigned int voltage; //unit:mV
float current;  //unit:mA
float filteredCurrent = 0.0; // Filtered current value
float pressure; // unit:Bar

void setup()
{
   Serial.begin(115200);
   
   // Configure ADC for ESP32-S3
   analogReadResolution(12); // 12-bit resolution
   analogSetAttenuation(ADC_11db); // 0-3.3V range
   
   delay(2000); // Sensor stabilization
}

float currentToPressure(float currentmA) {
    // Convert 4-20mA to 0-10 Bar
    if(currentmA < MIN_CURRENT) {
        return 0.0; // Below 4mA = fault condition
    }
    
    // Linear conversion: (current - 4mA) / (20mA - 4mA) * 10 Bar
    float pressureBar = ((currentmA - MIN_CURRENT) / (MAX_CURRENT - MIN_CURRENT)) * MAX_PRESSURE;
    
    // Clamp to valid range
    if(pressureBar < 0) pressureBar = 0;
    if(pressureBar > MAX_PRESSURE) pressureBar = MAX_PRESSURE;
    
    return pressureBar;
}

void loop()
{
    voltageSum = 0;
    
    // Read multiple samples with better timing
    for (int i = 0; i < SAMPLES; i++) {
        voltageSum += analogRead(CurrentSensorPin);
        delay(5); // Increased delay for better sample independence
    }
    
    // Calculate average voltage
    voltage = (voltageSum / SAMPLES) / 4096.0 * VREF;
    
    // Convert to current
    current = voltage / 120.0;  // Sense Resistor: 120ohm
    
    // Apply exponential moving average (low-pass filter)
    if (filteredCurrent == 0.0) {
        filteredCurrent = current; // Initialize on first reading
    } else {
        filteredCurrent = (FILTER_ALPHA * current) + ((1.0 - FILTER_ALPHA) * filteredCurrent);
    }
    
    // Convert filtered current to pressure
    pressure = currentToPressure(filteredCurrent);
    
    // Simple output: Current -> Voltage -> Pressure
    Serial.print("Current: ");
    Serial.print(filteredCurrent, 2);
    Serial.print(" mA | Voltage: ");
    Serial.print(voltage);
    Serial.print(" mV | Pressure: ");
    Serial.print(pressure, 2);
    Serial.println(" Bar");
    
    delay(1000);
}