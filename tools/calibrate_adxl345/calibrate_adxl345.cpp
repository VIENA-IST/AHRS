#include <ADXL345.h>
#include <Arduino.h>

ADXL345 accel;

int16_t ax, ay, az;
double ax_stOff=0, ay_stOff=0, az_stOff=0;
#define GRAVITY 256.0f

#define LED_PIN 13 // (Arduino is 13, Teensy is 6)
bool blinkState = false;
int I;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(500000);
    // initialize device
    Serial.println("Initializing I2C devices...");
    accel.initialize();
    // verify connection
    Serial.println("Testing device connections...");
    if(accel.testConnection()){
      Serial.println("ADXL345 connection successful");

      // set range to +- 16g
      accel.setRange(ADXL345_RANGE_16G);
      accel.setFullResolution(true);
      accel.setRate(ADXL345_RATE_100);
      accel.setLowPowerEnabled(false);
      accel.setMeasureEnabled(true);
      double nSamples = 1000.0;
      delay(100);
      for(I = 0; I < nSamples; I++){
    	accel.getAcceleration(&ax, &ay, &az);
    	ax_stOff += ax;
    	ay_stOff += ay;
    	az_stOff += az;
			if (I % 20 == 0)
				Serial.print("I = "), Serial.println(I);
      }
      ax_stOff /= nSamples;
      ay_stOff /= nSamples;
      az_stOff /= nSamples;
      Serial.println("Self tests off, offsets:");
      Serial.println(ax_stOff);
      Serial.println(ay_stOff);
      Serial.println(az_stOff);



    }else{
     Serial.println("ADXL345 connection failed");
   }

}

void loop() {
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    delay(1000);
}
