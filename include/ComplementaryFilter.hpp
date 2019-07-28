#include <cmath>
#include <stdio.h>
#include <vector>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/math/special_functions.hpp>


#define LOCAL_MAG_X 225.1 //miligauss
#define LOCAL_MAG_Y 5.75
#define LOCAL_MAG_Z 415.2

constexpr double gravity = 9.81;

using namespace std;



class ComplementaryFilter{

        // constructor
            ComplementaryFilter()
            {
                mag = Eigen::Vector3d(0,0,0);
                acc = Eigen::Vector3d(0,0,0);
                gyro = Eigen::Vector3d(0,0,0);
            }

        typedef Eigen::Matrix<double,4,1> Pose;

        struct flags
        {
            bool mag_data_available;
            bool mag_data_updated;
            bool gyro_data_updated;
            bool acc_data_updated;
            bool filter_update;
        };

        struct parameters
        {
            double k1;
            double k2;
            double k3;
            double k4;
            double kb;
            double sat;
        };
        


        public:

        // Maths Functions
        Eigen::Matrix3d quatToRotationMatrix(Pose quat);  // transforme quaternion to rotation matrix
        // Pose normalizeQuaternion(Pose quat);  // normalize quaternion
        // Pose crossfunction(Pose q1,Pose q2);  // quaternion cross function
        //Eigen::Vector3d saturator(Eigen::Vector3d x,double d){return std::min(1.0,d/x.norm()*x);}

        // Filter Methods
        void updateSensorData(Eigen::Vector3d gyro, Eigen::Vector3d accel, Eigen::Vector3d mag);
        void fuseInertialData();
        void updateFilter();



    

        private:

        float time_step;
        Eigen::Vector3d magFieldEarth;
        Eigen::Matrix3d rotationMatrix;
        Eigen::Vector3d mag;
        Eigen::Vector3d accel;
        Eigen::Vector3d gyro;
        Eigen::Vector3d bias;
        Pose quat;
        Pose update_quat;
        Eigen::Vector3d sigR;
        Eigen::Vector3d sigB;
        Eigen::Vector3d gyro_estimate;
        Eigen::Quaterniond quaternion;
        Eigen::Matrix3d rotation_matrix;

        // filter flags
        flags filterFlags;

        // filter coefficients
        parameters params;
        double dt;

}
