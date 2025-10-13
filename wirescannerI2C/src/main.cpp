#include <Arduino.h>
#include <Wire.h>

// put function declarations here:
int myFunction(int, int);

void setup() {
  Wire.begin();
  Serial.begin(115200);
  
  Serial.println("Memulai scanning alamat I2C...");
  
  byte error, address;
  int deviceCount = 0;
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("Perangkat I2C ditemukan pada alamat 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println();
      deviceCount++;
    }
    else if (error == 4) {
      Serial.print("Error tidak diketahui pada alamat 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("Tidak ada perangkat I2C ditemukan");
  } else {
    Serial.print("Ditemukan ");
    Serial.print(deviceCount);
    Serial.println(" perangkat I2C");
  }
}

void loop() {
  // Tidak ada yang perlu dilakukan secara berulang
  delay(5000);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}