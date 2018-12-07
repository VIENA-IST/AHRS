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
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h
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
    char anglesComputed;
    void computeAngles();

    //-------------------------------------------------------------------------------------------
    // Function declarations
  public:
    Madgwick(void);
    void begin(float sampleFrequency) { invSampleFreq = 1.0f / sampleFrequency; }
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    void updateMAG(float gx, float gy, float gz, float mx, float my);
    void updateGYRO(float gx, float gy, float gz);
    //float getPitch(){return atan2f(2.0f * q2 * q3 - 2.0f * q0 * q1, 2.0f * q0 * q0 + 2.0f * q3 * q3 - 1.0f);};
    //float getRoll(){return -1.0f * asinf(2.0f * q1 * q3 + 2.0f * q0 * q2);};
    //float getYaw(){return atan2f(2.0f * q1 * q2 - 2.0f * q0 * q3, 2.0f * q0 * q0 + 2.0f * q1 * q1 - 1.0f);};
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
