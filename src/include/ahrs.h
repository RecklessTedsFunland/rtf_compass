#pragma once

#include<vector.h>
#include<matrix.h>


class Sensor {
public:
    double range;
    Vec3 bias;
    Vec3 cov;
};

//-------------------------------------------------------------------------------------
// AHRS algorithm update
// Note: [q0 q1 q2 q3] = [w x y z]
// beta = sqrt(3/4)*wb
// wb: represents the estimated mean zero gyroscope measurement error of each axis
// gamma = sqrt(3/4)*wc
// wc: represents the estimated rate of gyroscope bias drift in each axis
//-------------------------------------------------------------------------------------
class AHRS {
public:
    AHRS();

    // void setUseMags(bool b);
    //
    // bool getUseMags(void);
    //
    // void setBeta(double b);
    //
    // inline double getBeta(void);

    void reset(void);

    void update(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz, double dt);

    //----------------------------------------------------------------------------------
    // IMU algorithm update
    void update(double gx, double gy, double gz, double ax, double ay, double az, double dt);

    //inline double normQ(){ return sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3); }


    // calc Euler, returns rads
    inline double roll(){ return atan2(2.0*q2*q3-2.0*q0*q1,2.0*q0*q0+2.0*q3*q3-1.0); }
    inline double pitch(){ return -asin(2.0*q1*q3+2.0*q0*q2); }
    inline double yaw(){ return atan2(2.0*q1*q2-2.0*q0*q3,2.0*q0*q0+2.0*q1*q1-1.0); }


    // geometry_msgs::Quaternion quaterion(){
    //     geometry_msgs::Quaternion q;
    //     q.x = q1;
    //     q.y = q2;
    //     q.z = q3;
    //     q.w = q0;
    //
    //     return q;
    // }

protected:

    double invSqrt(const double x);

    double q0,q1,q2,q3;
    double beta;
    bool useMags;
};
