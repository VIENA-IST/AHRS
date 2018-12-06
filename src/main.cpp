#include <Arduino.h>
#include <HardwareSerial.h>
#include <Razor.h>
#include <stdint.h>
#include <Wire.h>
#include <MadgwickAHRS.h>

#define DEBUG 0

// Sensor data output interval in milliseconds
// This may not work, if faster than 20ms (=50Hz)
// Code is tuned for 20ms, so better leave it like that
#define OUTPUT__DATA_INTERVAL 20 // in milliseconds

unsigned long timestamp;
unsigned long timestamp_old;

#define LED_PIN 13 // (Arduino is 13, Teensy is 6)

// variables
char *buffer = (char *)calloc(100, sizeof(char));
char *axString = (char *)calloc(7, sizeof(char));
char *ayString = (char *)calloc(7, sizeof(char));
char *azString = (char *)calloc(7, sizeof(char));

char *gxString = (char *)calloc(7, sizeof(char));
char *gyString = (char *)calloc(7, sizeof(char));
char *gzString = (char *)calloc(7, sizeof(char));

char *mxString = (char *)calloc(7, sizeof(char));
char *myString = (char *)calloc(7, sizeof(char));
char *mzString = (char *)calloc(7, sizeof(char));

ADXL345 accel;
ITG3200 gyro;
HMC5843 mag;
Razor razor;
Madgwick filter;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

float roll, pitch, heading;

void setup()
{
	float warmup = 100.0;
	// drop x samples or average x samples
	int I;
	int gx_off = 0, gy_off = 0, gz_off = 0; //temp to store offsets
	// join I2C bus (I2Cdev library doesn't do this automatically)
	Wire.begin();
	Wire.setClock(400000L);
	// configure LED for output
	pinMode(LED_PIN, OUTPUT);
	// initialize serial communication
	// (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
	// it's really up to you depending on your project)
	Serial.begin(500000);
	filter.begin(50);
	delay(50); // Give sensors enough time to start

	// initialize device
	Serial.println("Initializing I2C devices...");
	accel.initialize();
	delay(20);
	gyro.initialize();
	delay(20);
	mag.initialize();
	delay(20); // Give sensors enough time to setup

	// verify connection to sensors
	Serial.println("Testing device connections...");
	if (accel.testConnection())
	{
		Serial.println("ADXL345 connection successful");
		// config ADXL
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
	}
	else
	{
		Serial.println("ADXL345 connection failed");
	}
	// now test ITG3200
	if (gyro.testConnection())
	{
		Serial.println("ITG3200 connection successful");
		// config ITG3200
		gyro.setDLPFBandwidth(ITG3200_DLPF_BW_10);
		// drop first samples to achieve temperature stability;
		for (I = 0; I < warmup; I++)
		{
			gyro.getRotationX();
			gyro.getRotationY();
			gyro.getRotationZ();
			delay(20);
		}
		// get offsets with warmup samples
		for (I = 0; I < warmup; I++)
		{
			gyro.getRotation(&gx, &gy, &gz);
			gx_off += gx;
			gy_off += gy;
			gz_off += gz;
			delay(20);
		}

		razor.gyroSettings.gx_offset = gx_off / warmup;
		razor.gyroSettings.gy_offset = gy_off / warmup;
		razor.gyroSettings.gz_offset = gz_off / warmup;
	}
	else
	{
		Serial.println("ITG3200 connection failed");
	}
	if (mag.testConnection())
	{
		Serial.println("HMC5843 connection successful");
		// set to 50Hz
		mag.setDataRate(HMC5843_RATE_50);
		// set continous mode
		mag.setMode(HMC5843_MODE_CONTINUOUS);
	}
	else
	{
		Serial.println("HMC5843 connection failed");
	}

	// load values
	razor.loadAccSettings(razor.idxAcc);
	timestamp = millis();
	digitalWrite(LED_PIN, 1);
}

void loop()
{
	timestamp = millis();
	// Time to read the sensors again?
	if ((timestamp - timestamp_old) >= (OUTPUT__DATA_INTERVAL - 9))
	{
		// Serial.print(millis() - timestamp_old);
		accel.getAcceleration(&ax, &ay, &az);
		gyro.getRotation(&gx, &gy, &gz);
		mag.getHeading(&mx, &my, &mz);
		razor.outputPacket.acc[0] = ax;
		razor.outputPacket.acc[1] = ay;
		razor.outputPacket.acc[2] = az;
		razor.outputPacket.gyro[0] = gx;
		razor.outputPacket.gyro[1] = gy;
		razor.outputPacket.gyro[2] = gz;
		razor.outputPacket.mag[0] = mx;
		razor.outputPacket.mag[1] = my;
		razor.outputPacket.mag[2] = mz;
		razor.compensate_sensor_errors();

		filter.update(razor.outputPacket.gyro[0], razor.outputPacket.gyro[1],
					  razor.outputPacket.gyro[2], razor.outputPacket.acc[0],
					  razor.outputPacket.acc[1], razor.outputPacket.acc[2],
					  razor.outputPacket.mag[0], razor.outputPacket.mag[1],
					  razor.outputPacket.mag[2]);

		razor.outputPacket.euler[2] = filter.getRoll();
		razor.outputPacket.euler[1] = filter.getPitch();
		razor.outputPacket.euler[0] = filter.getYaw();
		razor.sendPacket();
		timestamp_old = timestamp;
	}
#if DEBUG
	delay(1000);
#endif
}
