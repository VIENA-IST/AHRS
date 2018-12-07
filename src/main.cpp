#include <Arduino.h>
#include <HardwareSerial.h>
#include <Razor.h>
#include <stdint.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <MadgwickAHRS_XY.h>
#include <stdio.h>

#ifndef DEBUG
#define DEBUG 0
#endif
#if DEBUG
#define DEBUG_MSG(...) Serial.println(__VA_ARGS__)
#endif
#ifndef DEBUG_MSG
#define DEBUG_MSG(...)
#endif

// Sensor data output interval in milliseconds
// This may not work, if faster than 20ms (=50Hz)
// Code is tuned for 20ms, so better leave it like that

#define SAMPLEFREQ 50.0f
#define CUTFREQ 4.0f
#define ALPHA CUTFREQ/(CUTFREQ+SAMPLEFREQ)
#define LED_PIN 13 // (Arduino is 13, Teensy is 6)

// variables

ADXL345 accel;
ITG3200 gyro;
HMC5843 mag;
Razor razor;
Madgwick filter;

/*
 * will store the connection sucessfull to each of the sensor in the following
 * form, as a result of each sensor testConnection() result
 * +----+----+----+----+----+----+----+----+
 * |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
 * +----+----+----+----+----+----+----+----+
 * | 0  | 0  | 0  | 0  | 0  |Gyro|Mag |Acc |
 * +----+----+----+----+----+----+----+----+
 *
 */
uint8_t sensorStatus = 0;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
// vars to store previous values to be used in the online lowpass function
//int16_t axPrevious, ayPrevious, azPrevious;
//int16_t gxPrevious, gyPrevious, gzPrevious;
//int16_t mxPrevious, myPrevious, mzPrevious;

float roll, pitch, heading;
float meanAcc = 0, meanMagXY = 0;

volatile bool readSensors = false;


/*
 * Timer interrupt function on TIMER1 to alert it is time to read sensors
 */
ISR (TIMER1_COMPA_vect) {
	readSensors = true;
}

void onlineLowPass(const float lastVal[3], const float newVal[3], float out[3]) {
	// use an online lowpass approximation
	for(int I = 0; I<3; I++)
		out[I] = ((1-ALPHA)*lastVal[I]+ALPHA*newVal[I]);
}

void setup() {
	// use multiple of base 2 for precision and faster computation
	float warmup = 128.0;
	float *aux;
	// drop I samples or average I samples
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
	delay(50);  // Give sensors enough time to start

	// initialize device
	Serial.println(F("Initializing I2C devices..."));
	accel.initialize();
	delay(50);
	gyro.initialize();
	delay(50);
	mag.initialize();
	delay(50);  // Give sensors enough time to setup

	// verify connection to sensors
	Serial.println(F("Testing device connections..."));
	if(accel.testConnection()){
		bitWrite(sensorStatus, 0, 1);
		Serial.println(F("ADXL345 connection successful"));
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
	}else{
		Serial.println(F("ADXL345 connection failed"));
	}
	// now test ITG3200
	if(gyro.testConnection()){
		bitWrite(sensorStatus, 2, 1);
		Serial.println(F("ITG3200 connection successful"));
		// config ITG3200
		gyro.setDLPFBandwidth(ITG3200_DLPF_BW_10);
		// drop first samples to achieve temperature stability;
		for(I = 0; I<warmup; I++){
			gyro.getRotationX();
			gyro.getRotationY();
			gyro.getRotationZ();
			delay(20);
		}
		// get offsets with warmup samples
		for(I = 0; I<warmup; I++){
			gyro.getRotation(&gx, &gy, &gz);
			gx_off += gx;
			gy_off += gy;
			gz_off += gz;
			delay(20);
		}

		razor.gyroSettings.gx_offset = gx_off/warmup;
		razor.gyroSettings.gy_offset = gy_off/warmup;
		razor.gyroSettings.gz_offset = gz_off/warmup;

	}else{
		Serial.println(F("ITG3200 connection failed"));
	}
	if(mag.testConnection()){
		bitWrite(sensorStatus, 1, 1);
		Serial.println(F("HMC5843 connection successful"));
		// set to 50Hz
		mag.setDataRate(HMC5843_RATE_50);
		// set continous mode
		mag.setMode(HMC5843_MODE_CONTINUOUS);
	}else{
		Serial.println(F("HMC5843 connection failed"));
	}
	/***************************************************************************
	 * break execution of code if any of the sensor status is not equal to 1
	 * and flashs built in led.
	 * Also prints sensorStatus as binary output for debug
	 **************************************************************************/
	if(sensorStatus<7){
		while(1){
			Serial.print(F("Failed connection to some senrsor: sensorStatus="));
			Serial.println(sensorStatus, BIN);
			digitalWrite(LED_PIN, !digitalRead(LED_PIN));
			delay(500);

		}
	}
	// load values
	razor.loadAccSettings(razor.idxAcc);
	// razor.loadMagSettings(razor.idxMag);


	/***************************************************************************
	 * calculate meanAcc and meanMag
	 **************************************************************************/
	for(I = 0; I<warmup; I++){
		accel.getAcceleration(&ax, &ay, &az);
		mag.getHeading(&mx, &my, &mz);
		razor.correctRawAcc(ax, ay, az);
		razor.correctRawMag(mx, my, mz);
		aux = razor.outputPacket.acc;
		meanAcc += sqrtf(aux[0]*aux[0]+aux[1]*aux[1]+aux[2]*aux[2]);
		aux = razor.outputPacket.mag;
		meanMagXY += sqrtf(aux[0]*aux[0]+aux[1]*aux[1]);
		delay(20);
	}
	meanAcc = meanAcc/warmup;
	meanMagXY = meanMagXY/warmup;

#if DEBUG
	DEBUG_MSG("meanAcc=");
	DEBUG_MSG(meanAcc);
	DEBUG_MSG("meanMagXY=");
	DEBUG_MSG(meanMagXY);
#endif
	digitalWrite(LED_PIN, 1);
	/***************************************************************************
	 *  initialize Timer1
	 * TIMER 1 for interrupt frequency 50 Hz:
	 **************************************************************************/
	cli();
	// stop interrupts
	TCCR1A = 0; // set entire TCCR1A register to 0
	TCCR1B = 0; // same for TCCR1B
	TCNT1 = 0;  // initialize counter value to 0
	// set compare match register for 50 Hz increments
	OCR1A = 19999; // = 8000000 / (8 * 50) - 1 (must be <65536)
	// turn on CTC mode
	TCCR1B |= (1<<WGM12);
	// Set CS12, CS11 and CS10 bits for 8 prescaler
	TCCR1B |= (0<<CS12)|(1<<CS11)|(0<<CS10);
	// enable timer compare interrupt
	TIMSK1 |= (1<<OCIE1A);
	// allow interrupts
	sei();
	//***************************************************************************
}

void loop() {
	static float lastAcc[3] = { 0.0f, 0.0f, 0.0f };
	static float lastGyro[3] = { 0.0f, 0.0f, 0.0f };
	static float lastMag[3] = { 0.0f, 0.0f, 0.0f };
	// Time to read the sensors again?
	if(readSensors){
		accel.getAcceleration(&ax, &ay, &az);
		gyro.getRotation(&gx, &gy, &gz);
		mag.getHeading(&mx, &my, &mz);
		razor.correctRawAcc(ax, ay, az);
		razor.correctRawGyro(gx, gy, gz);
		razor.correctRawMag(mx, my, mz);
		onlineLowPass(lastAcc, razor.outputPacket.acc, razor.outputPacket.acc);
		onlineLowPass(lastGyro, razor.outputPacket.gyro, razor.outputPacket.gyro);
		onlineLowPass(lastMag, razor.outputPacket.mag, razor.outputPacket.mag);
#if DEBUG
		DEBUG_MSG(lastAcc[0]);
		DEBUG_MSG(lastAcc[1]);
		DEBUG_MSG(lastAcc[2]);
		DEBUG_MSG(lastGyro[0]);
		DEBUG_MSG(lastGyro[1]);
		DEBUG_MSG(lastGyro[2]);
		DEBUG_MSG(lastMag[0]);
		DEBUG_MSG(lastMag[1]);
		DEBUG_MSG(lastMag[2]);
#endif
//		filter.update(razor.outputPacket.gyro[0], razor.outputPacket.gyro[1],
//				razor.outputPacket.gyro[2], razor.outputPacket.acc[0],
//				razor.outputPacket.acc[1], razor.outputPacket.acc[2],
//				razor.outputPacket.mag[0], razor.outputPacket.mag[1]);
//		filter.updateGyro(razor.outputPacket.gyro[0],
//						razor.outputPacket.gyro[1], razor.outputPacket.gyro[2]);
		filter.updateIMU(razor.outputPacket.gyro[0],
				razor.outputPacket.gyro[1], razor.outputPacket.gyro[2],
				razor.outputPacket.acc[0],
				razor.outputPacket.acc[1], razor.outputPacket.acc[2]);
		razor.outputPacket.euler[2] = filter.getPhi();
		razor.outputPacket.euler[1] = filter.getTheta();
		razor.outputPacket.euler[0] = filter.getPsi();
#if DEBUG
		razor.printPacket();
#else
		razor.sendPacket();
#endif
		memcpy(lastAcc, razor.outputPacket.acc, 3*sizeof(float));
		memcpy(lastGyro, razor.outputPacket.gyro, 3*sizeof(float));
		memcpy(lastMag, razor.outputPacket.mag, 3*sizeof(float));
		readSensors = false;
	}
#if DEBUG
	delay(1000);
#endif
}

