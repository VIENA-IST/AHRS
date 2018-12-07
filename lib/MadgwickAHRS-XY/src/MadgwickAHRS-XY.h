//=============================================================================================
// MadgwickAHRS.h
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
//
//=============================================================================================
#ifndef MadgwickAHRS_XY_h
#define MadgwickAHRS_XY_h
#include <math.h>

//--------------------------------------------------------------------------------------------
// Variable declaration
class Madgwick
{
  private:
    static float invSqrt(float x);
    float beta; // algorithm gain
    float q1;
    float q2;
    float q3;
    float q4; // quaternion of sensor frame relative to auxiliary frame
    float invSampleFreq;
    float phi;
    float theta;
    float psi;
	//threasholds to detect disturbances in acc or mag
	float magMin;
	float magMax;
	float accMin;
	float accMax;
    char anglesComputed;
    void computeAngles();

    //-------------------------------------------------------------------------------------------
    // Function declarations
  public:
    Madgwick(void);
    void begin(float sampleFrequency) { invSampleFreq = 1.0f / sampleFrequency; }
	// called if all measurements are available
	void update(float gx, float gy, float gz, float ax, float ay, float az,
			float mx, float my);
	// called if magnetometer is not available or out of range
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);

	// called if accelerometer is out of range
	void updateMAG(float gx, float gy, float gz, float mx, float my);

	// estimate based on gyroscope only
	void updateGYRO(float gx, float gy, float gz);

    float getPhi()
    {
        if (!anglesComputed)
            computeAngles();
        return phi * 57.29578f;
    }
    float getTheta()
    {
        if (!anglesComputed)
            computeAngles();
        return theta * 57.29578f;
    }
    float getPsi()
    {
        if (!anglesComputed)
            computeAngles();
        return psi * 57.29578f;
    }
    float getPhiRadians()
    {
        if (!anglesComputed)
            computeAngles();
        return phi;
    }
    float getThetaRadians()
    {
        if (!anglesComputed)
            computeAngles();
        return theta;
    }
    float getYPsiRadians()
    {
        if (!anglesComputed)
            computeAngles();
        return psi;
    }
};
#endif
