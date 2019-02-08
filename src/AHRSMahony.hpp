#include "math.h"
#include <stdio.h>
#include <eigen3/Eigen/Dense>


#define LOCAL_MAG_X 225.1 //miligauss
#define LOCAL_MAG_Y 5.75
#define LOCAL_MAG_Z 415.2

using namespace Eigen;


class AHRSMahony{

    // constructor
        AHRSMahony()
        {
            time_step = 0.1;
            magFieldEarth = Vector3f(LOCAL_MAG_X,LOCAL_MAG_Y,LOCAL_MAG_Z);
        }

    typedef Matrix<float,4,1> Pose;
    typedef Matrix<float,3,1> Vector3f;


    public:

    // Maths Functions
    Matrix3f quatToRotationMatrix(Pose quat);  // transforme quaternion to rotation matrix
    Pose normalizeQuaternion(Pose quat);  // normalize quaternion
    Pose crossfunction(Pose q1,Pose q2);  // quaternion cross function

    // Filter Methods
    void updateInertialData();
    void fuseInertialData();
    void updateFilter();


    struct 
    {
        bool mag_data_updated;
        bool gyro_data_updated;
        bool acc_data_updated;
        bool filter_updated;

    } flags_filter;
    

    private:

    float time_step;
    Vector3f magFieldEarth;
    Matrix3f rotationMatrix;
    Vector3f mag_buffer;
    Vector3f acc_buffer;
    Vector3f gyro_buffer;
    Vector3f bias;
    Pose quat;
    Pose update_quat;
    float _kI;
    float _kP;
    float k1;
    float k2;

}