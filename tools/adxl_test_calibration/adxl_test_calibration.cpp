#include <ADXL345.h>
#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include <EEPROM.h>

ADXL345 accel;

int16_t ax, ay, az;
double acc[3];
#define GRAVITY 256.0f

#define RAZOR_ID 2
// Sensor calibration scale and offset values
#if RAZOR_ID == 1
#define ACCEL_X_OFFSET 	 13.83f
#define ACCEL_Y_OFFSET 	 -3.18f
#define ACCEL_Z_OFFSET   -3.47f
#define ACCEL_X_SCALE 	266.04f
#define ACCEL_Y_SCALE 	266.27f
#define ACCEL_Z_SCALE 	253.66f
#endif

#if RAZOR_ID == 2
#define ACCEL_X_OFFSET 	  7.29f
#define ACCEL_Y_OFFSET 	  6.74f
#define ACCEL_Z_OFFSET  -38.03f
#define ACCEL_X_SCALE 	265.78f
#define ACCEL_Y_SCALE 	266.54f
#define ACCEL_Z_SCALE 	254.37f
#endif
// Sensor data output interval in milliseconds
// This may not work, if faster than 20ms (=50Hz)
// Code is tuned for 20ms, so better leave it like that
#define OUTPUT__DATA_INTERVAL 20  // in milliseconds

unsigned long timestamp;
unsigned long timestamp_old;

// hold values of sensor settings
struct AdxlSettings {
	byte id;
	float ax_offset;
	float ay_offset;
	float az_offset;
	float ax_scale;
	float ay_scale;
	float az_scale;
};

#define LED_PIN 13 // (Arduino is 13, Teensy is 6)

// variables
bool blinkState = false;
bool printVals = true;
char *buffer = (char*) calloc(100, sizeof(char));
char *axString = (char*) calloc(7, sizeof(char));
char *ayString = (char*) calloc(7, sizeof(char));
char *azString = (char*) calloc(7, sizeof(char));
char *axsString = (char*) calloc(7, sizeof(char));
char *aysString = (char*) calloc(7, sizeof(char));
char *azsString = (char*) calloc(7, sizeof(char));
char parseCmd = '\0';
struct AdxlSettings currentSettings;

// prototypes
char * sprintFloat(float, char*);

// Functions
void getAdxlSettings(struct AdxlSettings curr) {
	memset(&curr, 0, sizeof(struct AdxlSettings));
	EEPROM.get(0, curr);
	Serial.println("Values in EEPROM...");
	Serial.println(
			"ID\tAX_Offset\tAY_Offset\tAZ_Offset\tAX_Scale\tAY_Scale\tAZ_Scale");
	memset(buffer, 0, 100 * sizeof(char));
	sprintf(buffer, "%d\t%s\t%s\t%s\t%s\t%s\t%s", (int) curr.id,
			sprintFloat(curr.ax_offset, axString),
			sprintFloat(curr.ay_offset, ayString),
			sprintFloat(curr.az_offset, azString),
			sprintFloat(curr.ax_scale, axsString),
			sprintFloat(curr.ay_scale, aysString),
			sprintFloat(curr.az_scale, azsString));
	Serial.println(buffer);
}
void setAdxlSettings(struct AdxlSettings curr) {
	EEPROM.put(0, curr);
	Serial.println("Verifying values in EEPROM...");
	getAdxlSettings(curr);
}
void compensate_sensor_errors() {
	// Compensate accelerometer error
	acc[0] = (acc[0] - ACCEL_X_OFFSET) / ACCEL_X_SCALE;
	acc[1] = (acc[1] - ACCEL_Y_OFFSET) / ACCEL_Y_SCALE;
	acc[2] = (acc[2] - ACCEL_Z_OFFSET) / ACCEL_Z_SCALE;
}

void setup() {
	// join I2C bus (I2Cdev library doesn't do this automatically)
	Wire.begin();
	// configure LED for output
	pinMode(LED_PIN, OUTPUT);
	// initialize serial communication
	// (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
	// it's really up to you depending on your project)
	Serial.begin(500000);
	delay(50);  // Give sensors enough time to start
	// initialize device
	Serial.println("Initializing I2C devices...");
	accel.initialize();
	// verify connection
	Serial.println("Testing device connections...");
	if (accel.testConnection()) {
		Serial.println("ADXL345 connection successful");

		// set range to +- 16g
		accel.setRange(ADXL345_RANGE_16G);
		// set full resolution
		accel.setFullResolution(true);
		// set rate 100
		accel.setRate(ADXL345_RATE_100);
		// disable Low Power flag
		accel.setLowPowerEnabled(false);
		// enable Measure flag
		accel.setMeasureEnabled(true);

	} else {
		Serial.println("ADXL345 connection failed");
	}
	timestamp = millis();

}

char * sprintFloat(float val, char *accString) {
	memset(accString, 0, 7 * sizeof(char));
	return dtostrf(val, 6, 2, accString);

}

void loop() {
	// print values?
	if (printVals)
		// Time to read the sensors again?
		if ((millis() - timestamp) >= OUTPUT__DATA_INTERVAL) {
			timestamp_old = timestamp;
			timestamp = millis();
			accel.getAcceleration(&ax, &ay, &az);
			acc[0] = ax;
			acc[1] = ay;
			acc[2] = az;
			compensate_sensor_errors();
			memset(buffer, 0, 100 * sizeof(char));
			sprintf(buffer, "%s\t%s\t%s", sprintFloat(acc[0], axString),
					sprintFloat(acc[1], ayString),
					sprintFloat(acc[2], azString));

			Serial.println(buffer);
		}
	if (Serial.available() > 1) {
		parseCmd = Serial.read();
		switch (parseCmd) {
		case 's':
			// drop \n
			Serial.read();
			Serial.println("Storing values in eeprom...");
			Serial.println(
					"ID\tAX_Offset\tAY_Offset\tAZ_Offset\tAX_Scale\tAY_Scale\tAZ_Scale");
			memset(buffer, 0, 100 * sizeof(char));
			sprintf(buffer, "%d\t%s\t%s\t%s\t%s\t%s\t%s",
			RAZOR_ID, sprintFloat(ACCEL_X_OFFSET, axString),
					sprintFloat(ACCEL_Y_OFFSET, ayString),
					sprintFloat(ACCEL_Z_OFFSET, azString),
					sprintFloat(ACCEL_X_SCALE, axsString),
					sprintFloat(ACCEL_Y_SCALE, aysString),
					sprintFloat(ACCEL_Z_SCALE, azsString));
			Serial.println(buffer);
			currentSettings.id = RAZOR_ID;
			currentSettings.ax_offset = ACCEL_X_OFFSET;
			currentSettings.ay_offset = ACCEL_Y_OFFSET;
			currentSettings.az_offset = ACCEL_Z_OFFSET;
			currentSettings.ax_scale = ACCEL_X_SCALE;
			currentSettings.ay_scale = ACCEL_Y_SCALE;
			currentSettings.az_scale = ACCEL_Z_SCALE;
			setAdxlSettings(currentSettings);
			printVals = false;
			break;
		case 'r':
			printVals = false;
			getAdxlSettings(currentSettings);
			break;
		case 'p':
			printVals = true;
			break;
		}

	}
	// blink LED to indicate activity
	blinkState = !blinkState;
	digitalWrite(LED_PIN, blinkState);
}
