#include "ComplementaryFilter.hpp"

/*
||===========================================||
||===============Filter Methods==============||
||===========================================||
*/


void ComplementaryFilter::initFilter()
{


    is_init = true;
}


void ComplementaryFilter::updateSensorData(Eigen::Vector3d gyro, Eigen::Vector3d accel, Eigen::Vector3d mag)
{

    this->mag = mag;
    this->gyro = gyro;
    this->accel = accel;
    
}

void ComplementaryFilter::fuseInertialData()
{

    // local variables used for data fusion
    Eigen::Vector3d u_body,u_inertial,v_body,v_inertial,v_estimate,u_estimate; 
    Eigen::Matrix3d projUb,projUi;

    u_body = -1.0 * accel / gravity;
    u_inertial = Eigen::Vector3f(0,0,1);

    // compute the orthogonal projection of vector uBody and uInertial
    projUb = pow(u_body.norm(),2)*Eigen::Matrix3d::Identity() - u_body * (u_body.transpose());
    projUi = pow(u_body.norm(),2)*Eigen::Matrix3d::Identity() - u_body * (u_body.transpose());

    v_body = (projUb*mag)/(projUi*magFieldEarth).norm();
    v_inertial = (projUi*magFieldEarth)/(projUi*magFieldEarth).norm();
    
    // Compute gravity and magnetic field estimate in body frame
    u_estimate = quaternion.toRotationMatrix().transpose()*u_inertial;
    v_estimate = quaternion.toRotationMatrix().transpose()*v_inertial;

    // Fuse the data;
    sigR = params.k1 * u_body.cross(u_estimate) + params.k2 * ( u_estimate.dot(u_estimate.transpose()) ).dot(v_body.cross(v_estimate)) ;
    sigB = params.k3 * u_body.cross(u_estimate) - params.k4 * v_body.cross(v_estimate);
    
    // Correct the gyro reading with the previous bias and the fused data
    gyro_estimate = gyro - bias + sigR;

}


void ComplementaryFilter::updateFilter()
{

    Eigen::Matrix4d A_rate;

    A_rate <<   0,                  -gyro_estimate(0),  -gyro_estimate(1),  -gyro_estimate(2),
                gyro_estimate(0),                   0,   gyro_estimate(2),  -gyro_estimate(1),
                gyro_estimate(1),   -gyro_estimate(2),                  0,   gyro_estimate(0),
                gyro_estimate(2),    gyro_estimate(1),   -gyro_estimate(0),                 0;


    quaternion = (cos( 0.5*dt*gyro_estimate.norm() ) * Eigen::Matrix4d::Identity() + 0.5*dt*boost::math::sinc_pi(0.5*dt*gyro_estimate.norm())*A_rate)*quaternion;

    Eigen::Vector3d bias_sat = std::min(1.0,params.sat/bias.norm())*bias;
    bias += dt * (-params.kb*bias + params.kb*bias_sat + sigB );

    quaternion.normalize();

}

/*
||================================================||
||===============Algebraic Functions==============||
||================================================||
*/


Eigen::Matrix3d ComplementaryFilter::quatToRotationMatrix(Eigen::Vector4d q)
{

    Eigen::Matrix3d R;

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