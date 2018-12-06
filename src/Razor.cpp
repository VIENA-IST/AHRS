/*
 * Razor.cpp
 *
 *  Created on: Dec 6, 2016
 *      Author: bruno
 */

#include <Arduino.h>
#include <EEPROM.h>
#include <Razor.h>
#include <Wire.h>

#define DEBUG 0
#define GYRO_GAIN 14.3750f

char * sprintFloat(float val, char *accString) {
	memset(accString, 0, 7 * sizeof(char));
	return dtostrf(val, 6, 2, accString);

}
// Multiply 3x3 matrix with vector: out = a * b
// out has to different from b (no in-place)!
void matrix_Vector_Multiply(const float a[3][3], const float b[3],
		double out[3]) {
	for (int x = 0; x < 3; x++) {
		out[x] = a[x][0] * b[0] + a[x][1] * b[1] + a[x][2] * b[2];
	}
}

// ================= 8-BIT CRC ===================
//  code taken from https://github.com/FrankBoesing/FastCRC
/** Constructor
 */
FastCRC8::FastCRC8() {
	seed = 0x00;
}
FastCRC8::~FastCRC8() {
	/// TODO Auto-generated destructor stub
}
;

/** MAXIM 8-Bit CRC
 * equivalent to _crc_ibutton_update() in crc16.h from avr_libc
 * @param data Pointer to Data
 * @param datalen Length of Data
 * @return CRC value
 */
uint8_t FastCRC8::maxim_upd(const uint8_t *data, uint16_t datalen) {
	uint8_t crc = seed;
	if (datalen)
		do {
			crc = pgm_read_byte(&crc_table_maxim[crc ^ *data]);
			data++;
		} while (--datalen);
	seed = crc;
	return crc;
}
uint8_t FastCRC8::maxim(const uint8_t *data, const uint16_t datalen) {
	// poly=0x31 init=0x00 refin=true refout=true xorout=0x00  check=0xa1
	seed = 0x00;
	return maxim_upd(data, datalen);
}

Razor::Razor(const int idx) {
	this->idxAcc = idx + sizeof(byte);
	this->idxGyro = this->idxAcc + sizeof(struct AdxlSettings);
	this->idxMag = this->idxGyro + sizeof(struct Itg3200Settings);
	setId(EEPROM.read(idx));
	this->outputPacket.acc[0] = 0;
	this->outputPacket.acc[1] = 0;
	this->outputPacket.acc[2] = 0;
	this->outputPacket.gyro[0] = 0;
	this->outputPacket.gyro[1] = 0;
	this->outputPacket.gyro[2] = 0;
	this->outputPacket.mag[0] = 0;
	this->outputPacket.mag[1] = 0;
	this->outputPacket.mag[2] = 0;
	memset(&this->accSettings, 0, sizeof(struct AdxlSettings));
	this->accSettings.ax_scale = 256.0;
	this->accSettings.ay_scale = 256.0;
	this->accSettings.az_scale = 256.0;
	memset(&this->gyroSettings, 0, sizeof(struct Itg3200Settings));
	memset(&this->magSettings, 0, sizeof(struct Hmc5843Settings));
	this->magSettings.mTransform[0][0] = 1;
	this->magSettings.mTransform[1][1] = 1;
	this->magSettings.mTransform[2][2] = 1;
}
Razor::~Razor() {
	/// TODO Auto-generated destructor stub
}

void Razor::loadAccSettings(const int idx) {
	struct AdxlSettings curr;
	memset(&curr, 0, sizeof(struct AdxlSettings));
	EEPROM.get(idx, curr);
	setAccSettings(curr);
}

void Razor::storeAccSettings(const int idx) {
	struct AdxlSettings curr = getAccSettings();
	EEPROM.put(idx, curr);
}

void Razor::loadMagSettings(const int idx) {
	struct Hmc5843Settings curr;
	memset(&curr, 0, sizeof(struct Hmc5843Settings));
	EEPROM.get(idx, curr);
	setMagSettings(curr);
}

void Razor::storeMagSettings(const int idx) {
	struct Hmc5843Settings curr = getMagSettings();
	EEPROM.put(idx, curr);
}

void Razor::compensate_sensor_errors() {
	// Compensate accelerometer error
	this->outputPacket.acc[0] = (this->outputPacket.acc[0]
			- this->accSettings.ax_offset) / this->accSettings.ax_scale;
	this->outputPacket.acc[1] = (this->outputPacket.acc[1]
			- this->accSettings.ay_offset) / this->accSettings.ay_scale;
	this->outputPacket.acc[2] = (this->outputPacket.acc[2]
			- this->accSettings.az_offset) / this->accSettings.az_scale;
	// Compensate gyro error
	this->outputPacket.gyro[0] = (this->outputPacket.gyro[0]
			- this->gyroSettings.gx_offset) / GYRO_GAIN;
	this->outputPacket.gyro[1] = (this->outputPacket.gyro[1]
			- this->gyroSettings.gy_offset) / GYRO_GAIN;
	this->outputPacket.gyro[2] = (this->outputPacket.gyro[2]
			- this->gyroSettings.gz_offset) / GYRO_GAIN;
	// Compensate mag error
	matrix_Vector_Multiply(this->magSettings.mTransform,
			(float*) this->outputPacket.mag, this->outputPacket.mag);
	this->outputPacket.mag[0] = (this->outputPacket.mag[0]
			- this->magSettings.mx_offset);
	this->outputPacket.mag[1] = (this->outputPacket.mag[1]
			- this->magSettings.my_offset);
	this->outputPacket.mag[2] = (this->outputPacket.mag[2]
			- this->magSettings.mz_offset);
}

void Razor::printAccSettings() {
	struct AdxlSettings curr = getAccSettings();
	Serial.println("..............ACC OFFSETS...............");
	Serial.print("ax_offset: ");
	Serial.println(curr.ax_offset);
	Serial.print("ay_offset: ");
	Serial.println(curr.ay_offset);
	Serial.print("az_offset: ");
	Serial.println(curr.az_offset);
	Serial.println("..............ACC SCALES...............");
	Serial.print("ax_scale: ");
	Serial.println(curr.ax_scale);
	Serial.print("ay_scale: ");
	Serial.println(curr.ay_scale);
	Serial.print("az_scale: ");
	Serial.println(curr.az_scale);
}
void Razor::printGyroSettings() {
	struct Itg3200Settings curr = getGyroSettings();
	Serial.println("..............GYRO OFFSETS...............");
	Serial.print("gx_offset: ");
	Serial.println(curr.gx_offset);
	Serial.print("gy_offset: ");
	Serial.println(curr.gy_offset);
	Serial.print("gz_offset: ");
	Serial.println(curr.gz_offset);
}
void Razor::printMagSettings() {
	struct Hmc5843Settings curr = getMagSettings();
	int I;
	Serial.println("..............MAG OFFSETS...............");
	Serial.print("mx_offset: ");
	Serial.println(curr.mx_offset);
	Serial.print("my_offset: ");
	Serial.println(curr.my_offset);
	Serial.print("mz_offset: ");
	Serial.println(curr.mz_offset);
	Serial.println("......MAG TRANSFORMATION MATRIX.........");
	for (I = 0; I < 3; I++) {
		Serial.print(curr.mTransform[I][0]);
		Serial.print("\t");
		Serial.print(curr.mTransform[I][1]);
		Serial.print("\t");
		Serial.println(curr.mTransform[I][2]);
	}
}
void Razor::printPacket() {
	struct dataFrame *packet = &this->outputPacket;
	Serial.println("--------------Current packet-------------");
	Serial.println(packet->sync01, HEX);
	Serial.println(packet->sync02, HEX);
	Serial.println(packet->ID);
	// Serial.println(packet->index);
	Serial.print(packet->acc[0]);
	Serial.print('\t');
	Serial.print(packet->acc[1]);
	Serial.print('\t');
	Serial.print(packet->acc[2]);
	Serial.println('\t');
	Serial.print(packet->gyro[0]);
	Serial.print('\t');
	Serial.print(packet->gyro[1]);
	Serial.print('\t');
	Serial.print(packet->gyro[2]);
	Serial.println('\t');
	Serial.print(packet->mag[0]);
	Serial.print('\t');
	Serial.print(packet->mag[1]);
	Serial.print('\t');
	Serial.print(packet->mag[2]);
	Serial.println('\t');
	Serial.print(packet->euler[0]);
	Serial.println('\t');
	Serial.print(packet->euler[1]);
	Serial.println('\t');
	Serial.print(packet->euler[2]);
	Serial.println('\t');
	Serial.println(packet->crc8, HEX);
	Serial.println("---------------------------------------");
}

void Razor::sendPacket() {
	struct dataFrame *packet = &this->outputPacket;
#if DEBUG
	this->printPacket();
#endif
	packet->crc8 = 0;
	packet->crc8 = crc.maxim((uint8_t*) packet, this->packetSize);
#if DEBUG
	this->printPacket();
#endif
	Serial.write((uint8_t*) packet, this->packetSize);
	// packet->index++;

}
