#ifndef _COMPLEMENTARY_FILTER_HPP_
#define _COMPLEMENTARY_FILTER_HPP_

#include <cmath>
#include <stdio.h>
#include <vector>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/math/special_functions.hpp>


// #define LOCAL_MAG_X 225.1 //miligauss
// #define LOCAL_MAG_Y 5.75
// #define LOCAL_MAG_Z 415.2

constexpr double gravity = 9.80665;
constexpr double local_mx = 208.762; //in miligauss
constexpr double local_my = 3.448;
constexpr double local_mz = 434.129;

using namespace std;


    class ComplementaryFilter{

        // constructor
        public:

        ComplementaryFilter()
            {
                is_init = false;
                data_fused = false;
                filter_updated = false;
                magFieldEarth = Eigen::Vector3d(local_mx,local_my,local_mz);
                mag = Eigen::Vector3d(0,0,0);
                accel = Eigen::Vector3d(0,0,0);
                gyro = Eigen::Vector3d(0,0,0);
            }

        struct parameters
        {
            double k1;
            double k2;
            double k3;
            double k4;
            double kb;
            double sat;
        };
        
        // Maths Functions
        Eigen::Matrix3d quatToRotationMatrix(Eigen::Vector4d quat);  // transforme quaternion to rotation matrix

        // Filter Methods
        void initFilter();
        void updateSensorData(Eigen::Vector3d gyro, Eigen::Vector3d accel, Eigen::Vector3d mag);
        void fuseInertialData();
        void updateFilter();

        private:

        // filter variables;
        double dt; 
        Eigen::Vector3d magFieldEarth;
        Eigen::Matrix3d rotationMatrix;
        Eigen::Vector3d mag;
        Eigen::Vector3d accel;
        Eigen::Vector3d gyro;
        Eigen::Vector3d bias;
        Eigen::Vector3d sigR;
        Eigen::Vector3d sigB;
        Eigen::Vector3d gyro_estimate;
        Eigen::Quaterniond quaternion;
        Eigen::Matrix3d rotation_matrix;

        // filter flags
        bool is_init;
        bool data_fused;
        bool filter_updated;

        // filter parameters
        parameters params;

    };


#endif