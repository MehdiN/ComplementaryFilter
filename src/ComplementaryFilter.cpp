#include "ComplementaryFilter.hpp"

/*
||===========================================||
||===============Main Functions==============||
||===========================================||
*/


void ComplementaryFilter::updateInertialData(){

    bool is_compass_available = false;
    bool is_accel_available = false;
    bool is_gyro_available = false;

    if(is_compass_available){
        // TODO: get data from compass sensor and load to buffer;
        this->flags_filter.mag_data_available = true;
    } else {
        this->flags_filter.mag_data_available = true;
    }

    if(is_accelerometer_available){
        // TODO: get data from accel sensor and load to buffer;
        flags_filter.acc_data_available = true;
    } else {
        flags_filter.acc_data_available = true;
    }

    if(is_accelerometer_available){
        // TODO: get data from gyro sensor and load to buffer;
        flags_filter.gyro_data_available = true;
    } else {
        flags_filter.gyro_data_available = true;
    }


}


void ComplementaryFilter::fuseInertialData(){

    float acc_h[3] = {0};
    float mag_h[3] = {0};
    float acc[3] = {0};
    float mag[3] = {0};

    float fusion_buffer[3];

    Vector3f downVector(0,0,1);
    
    acc_h[0] = downVector(0)*rotationMatrix(0,0) + downVector(1)*rotationMatrix(1,0) + downVector(2)*rotationMatrix(2,0);
    acc_h[1] = downVector(0)*rotationMatrix(0,1) + downVector(1)*rotationMatrix(1,1) + downVector(2)*rotationMatrix(2,1);
    acc_h[2] = downVector(0)*rotationMatrix(0,2) + downVector(1)*rotationMatrix(1,2) + downVector(2)*rotationMatrix(2,2);



    magFieldEarth.normalize();
    mag_h[0] = magFieldEarth(0)*rotationMatrix(0,0) + magFieldEarth(1)*rotationMatrix(1,0) + magFieldEarth(2)*rotationMatrix(2,0);
    mag_h[1] = magFieldEarth(0)*rotationMatrix(0,1) + magFieldEarth(1)*rotationMatrix(1,1) + magFieldEarth(2)*rotationMatrix(2,1);
    mag_h[2] = magFieldEarth(0)*rotationMatrix(0,2) + magFieldEarth(1)*rotationMatrix(1,2) + magFieldEarth(2)*rotationMatrix(2,2);


    // Normalize sensor data

    acc_buffer.normalize();
    mag_buffer.normalize();

    // FUSION STEP 1

    fusion_buffer[0] = -0.5*k1*(-mag[1]*mag_h[2] + mag[2]*mag_h[1]) - 0.5*k2*(-acc[1]*acc_h[2] + acc[2]*acc_h[1]);
    fusion_buffer[1] = -0.5*k1*(-mag[0]*mag_h[1] + mag[1]*mag_h[0]) - 0.5*k2*(-acc[0]*acc_h[1] + acc[1]*acc_h[0]);
    fusion_buffer[2] = -0.5*k1*(mag[0]*mag_h[2] - mag[2]*mag_h[0]) - 0.5*k2*(acc[0]*acc_h[2] - acc[2]*acc_h[0]);

    // FUSION STEP 2

    fusion_buffer[0] = gyro[0] - bias(0) + kP*fusion_buffer[0]
    fusion_buffer[0] = gyro[0] - bias(1) + kP*fusion_buffer[1]
    fusion_buffer[0] = gyro[0] - bias(2) + kP*fusion_buffer[2]

    // return fusion_buffer

}


void AHRHMahony::updateFilter()
{
    // Todo 
}



/*
||================================================||
||===============Algebraic Functions==============||
||================================================||
*/

ComplementaryFilter::Pose ComplementaryFilter::normalizeQuaternion(Pose quat){
    return quat.normalized();
}


ComplementaryFilter::Pose ComplementaryFilter::crossfunction(Pose q1,Pose q2){

    Pose P;

    P(0) = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    P(1) = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    P(2) = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    P(3) = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];

    return P;
}


Matrix3f ComplementaryFilter::quatToRotationMatrix(Pose q){

    Matrix3f R;

    R(0,0) = -2*pow(q(2), 2) - 2*pow(q(3), 2) + 1;
    R(0,1) = -2*q(0)*q(3) + 2*q(1)*q(2);
    R(0,2) = 2*q(0)*q(2) + 2*q(1)*q(3);
    R(1,1) = 2*q(0)*q(3) + 2*q(1)*q(2);
    R(1,1) = -2*pow(q(1), 2) - 2*pow(q(3), 2) + 1;
    R(1,2) = -2*q(0)*q(1) + 2*pow(q(3), 2);
    R(2,0) = -2*q(0)*q(2) + 2*q(1)*q(3);
    R(2,1) = 2*q(0)*q(1) + 2*q(2)*q(3);
    R(2,2) = -2*pow(q(1), 2) - 2*pow(q(3), 2) + 1;

    return R;

}