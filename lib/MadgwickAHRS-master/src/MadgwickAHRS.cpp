// VERSION 2.0 - Adapting to VIENA PROJECT
//=============================================================================================
// MadgwickAHRS.c
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=============================================================================================

//-------------------------------------------------------------------------------------------
// Header files

#include "MadgwickAHRS.h"
#include <math.h>

//-------------------------------------------------------------------------------------------
// Definitions

#define sampleFreqDef 512.0f // sample frequency in Hz
#define betaDef 0.4f		 // 2 * proportional gain

//============================================================================================
// Functions

//-------------------------------------------------------------------------------------------
// AHRS algorithm update

Madgwick::Madgwick()
{
	beta = betaDef;
	q1 = 1.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	q4 = 0.0f;
	invSampleFreq = 1.0f / sampleFreqDef;
	anglesComputed = 0;
	yaw = 0.0f;
	pitch = 0.0f;
	roll = 0.0f
}

// Update: uses both sensor readings, the magnetometer and the
// accelerometer to make the corrections. Use in cases where
// readings are within expected range.
void Madgwick::update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float F1, F2, F3, F4, F5, F6;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
	{
		updateIMU(gx, gy, gz, ax, ay, az);
		return;
	}

	// Convert gyroscope degrees/sec to radians/sec
	gx *= 0.0174533f;
	gy *= 0.0174533f;
	gz *= 0.0174533f;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz);
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy);
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx);
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Objective Function F
		//magnetometer
		F1 = 2.0f * (q1 * q4 + q2 * q3) - mx;
		F2 = 2.0f * (0.5f - q2 * q2 - q4 * q4) - my;
		F3 = 2.0f * (q3 * q4 - q1 * q2);
		//accelerometer
		F4 = 2.0f * (q2 * q4 - q1 * q3) - ax;
		F5 = 2.0f * (q1 * q2 + q3 * q4) - ay;
		F6 = 2.0f * (0.5f - q2 * q2 - q3 * q3) - az;

		// Jacobian Matrix Transposed (J')
		// -2q2    0 2q4    0 2q2 -2q3
		// -2q1 -4q2 2q3 -4q2 2q1  2q4
		//  2q4    0 2q2 -4q3 2q4 -2q1
		//  2q3 -4q4 2q1    0 2q3  2q2

		// Gradient decent algorithm corrective step (J'*F)
		s0 = -2.0f * q2 * F1 + 2.0f * q4 * F3 + 2.0f * q2 * F5 - 2.0f * q3 * F6;
		s1 = -2.0f * q1 * F1 - 4.0f * q2 * F2 + 2.0f * q3 * F3 - 4.0f * q2 * F4 + 2.0f * q1 * F5 + 2.0f * q4 * F6;
		s2 = 2.0f * q4 * F1 + 2.0f * q2 * F3 - 4.0f * q3 * F4 + 2.0f * q4 * F5 - 2.0f * q1 * F6;
		s3 = 2.0f * q3 * F1 - 4.0f * q4 * F2 + 2.0f * q1 * F3 + 2.0f * q3 * F5 + 2.0f * q2 * F6;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q1 += qDot1 * invSampleFreq;
	q2 += qDot2 * invSampleFreq;
	q3 += qDot3 * invSampleFreq;
	q4 += qDot4 * invSampleFreq;

	// Normalise quaternion
	recipNorm = invSqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	q4 *= recipNorm;
	anglesComputed = 0;
}

//-------------------------------------------------------------------------------------------
// UpdateIMU: use only when no valid magnetometer readings are
// present ( or no sensor at all) in case unexpected magnetic
// interference.
void Madgwick::updateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q1, _2q2, _2q3, _2q4, _4q1, _4q2, _4q3, _8q2, _8q3, q1q1, q2q2, q3q3, q4q4;

	// Convert gyroscope degrees/sec to radians/sec
	gx *= 0.0174533f;
	gy *= 0.0174533f;
	gz *= 0.0174533f;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz);
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy);
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx);
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q4 = 2.0f * q4;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_4q3 = 4.0f * q3;
		_8q2 = 8.0f * q2;
		_8q3 = 8.0f * q3;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;
		q4q4 = q4 * q4;

		// Gradient decent algorithm corrective step
		s0 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay;
		s1 = _4q2 * q4q4 - _2q4 * ax + 4.0f * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az;
		s2 = 4.0f * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az;
		s3 = 4.0f * q2q2 * q4 - _2q2 * ax + 4.0f * q3q3 * q4 - _2q3 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q1 += qDot1 * invSampleFreq;
	q2 += qDot2 * invSampleFreq;
	q3 += qDot3 * invSampleFreq;
	q4 += qDot4 * invSampleFreq;

	// Normalise quaternion
	recipNorm = invSqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	q4 *= recipNorm;
	anglesComputed = 0;
}

//-------------------------------------------------------------------------------------------
// UpdateMAG: use only when no valid accelerometer readings are
// present in case of external accelerations.
void Madgwick::updateMAG(float gx, float gy, float gz, float mx, float my, float mz)
{
}

//-------------------------------------------------------------------------------------------
// UpdateGYRO: when no valid reading from both sensors, perform
// simple gyroscope integration without correction until valid
// readings become available.
void Madgwick::updateGYRO(float gx, float gy, float gz)
{
}
//-------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float Madgwick::invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long *)&y;

	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//-------------------------------------------------------------------------------------------

void Madgwick::computeAngles()
{
	roll = atan2f(q1 * q2 + q3 * q4, 0.5f - q2 * q2 - q3 * q3);
	pitch = asinf(-2.0f * (q2 * q4 - q1 * q3));
	yaw = atan2f(q2 * q3 + q1 * q4, 0.5f - q3 * q3 - q4 * q4);
	anglesComputed = 1;
}
