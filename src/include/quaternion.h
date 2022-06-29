#pragma once

// #include <stdlib.h>
// #include <string.h>
// #include <stdint.h>
#include <math.h>

#include <matrix.h>
#include <vector.h>

class Quaternion
{
public:
    Quaternion(): w(1.0), x(0.0), y(0.0), z(0.0) {}

    Quaternion(double w, double x, double y, double z):
        w(w), x(x), y(y), z(z) {}

    Quaternion(const Vec3& axis, double theta){
        w = cos(theta/2.0);

        double s = sin(theta/2.0);
        x = axis.x * s;
        y = axis.y * s;
        z = axis.z * s;
    }

    double magnitude() const {
        return sqrt(w*w + x*x + y*y + z*z);
    }
    void normalize(){
        *this = this->scale(1.0/magnitude());
    }
    Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }

    //
    // void fromMatrix(const Matrix& m)
    // {
    //     double tr = m.trace();
    //
    //     double S;
    //     if (tr > 0)
    //     {
    //         S = sqrt(tr+1.0) * 2;
    //         w = 0.25 * S;
    //         x = (m(2, 1) - m(1, 2)) / S;
    //         y = (m(0, 2) - m(2, 0)) / S;
    //         z = (m(1, 0) - m(0, 1)) / S;
    //     }
    //     else if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2))
    //     {
    //         S = sqrt(1.0 + m(0, 0) - m(1, 1) - m(2, 2)) * 2;
    //         w = (m(2, 1) - m(1, 2)) / S;
    //         x = 0.25 * S;
    //         y = (m(0, 1) + m(1, 0)) / S;
    //         z = (m(0, 2) + m(2, 0)) / S;
    //     }
    //     else if (m(1, 1) > m(2, 2))
    //     {
    //         S = sqrt(1.0 + m(1, 1) - m(0, 0) - m(2, 2)) * 2;
    //         w = (m(0, 2) - m(2, 0)) / S;
    //         x = (m(0, 1) + m(1, 0)) / S;
    //         y = 0.25 * S;
    //         z = (m(1, 2) + m(2, 1)) / S;
    //     }
    //     else
    //     {
    //         S = sqrt(1.0 + m(2, 2) - m(0, 0) - m(1, 1)) * 2;
    //         w = (m(1, 0) - m(0, 1)) / S;
    //         x = (m(0, 2) + m(2, 0)) / S;
    //         y = (m(1, 2) + m(2, 1)) / S;
    //         z = 0.25 * S;
    //     }
    // }
    //
    // Matrix toMatrix() const
    // {
    //     Matrix ret;
    //     ret.cell(0, 0) = 1 - 2*y*y - 2*z*z;
    //     ret.cell(0, 1) = 2*x*y - 2*w*z;
    //     ret.cell(0, 2) = 2*x*z + 2*w*y;
    //
    //     ret.cell(1, 0) = 2*x*y + 2*w*z;
    //     ret.cell(1, 1) = 1 - 2*x*x - 2*z*z;
    //     ret.cell(1, 2) = 2*y*z - 2*w*x;
    //
    //     ret.cell(2, 0) = 2*x*z - 2*w*y;
    //     ret.cell(2, 1) = 2*y*z + 2*w*x;
    //     ret.cell(2, 2) = 1 - 2*x*x - 2*y*y;
    //     return ret;
    // }


    // Returns euler angles that represent the quaternion.  Angles are
    // returned in rotation order and right-handed about the specified
    // axes:
    //
    //   v[0] is applied 1st about z (ie, roll)
    //   v[1] is applied 2nd about y (ie, pitch)
    //   v[2] is applied 3rd about x (ie, yaw)
    //
    // Note that this means result.x() is not a rotation about x;
    // similarly for result.z().
    //
    Vec3 toEuler() const
    {
        Vec3 ret;
        double sqw = w*w;
        double sqx = x*x;
        double sqy = y*y;
        double sqz = z*z;

        ret.x = atan2(2.0*(x*y+z*w),(sqx-sqy-sqz+sqw));
        ret.y = asin(-2.0*(x*z-y*w)/(sqx+sqy+sqz+sqw));
        ret.z = atan2(2.0*(y*z+x*w),(-sqx-sqy+sqz+sqw));

        return ret;
    }

    // Vector rotateVector(const Vector& v) const
    // {
    //     Vector qv(x, y, z);
    //     Vector t = qv.cross(v) * 2.0;
    //     return v + t*w + qv.cross(t);
    // }


    Quaternion operator*(const Quaternion& q) const
    {
        return Quaternion(
            w*q.w - x*q.x - y*q.y - z*q.z,
            w*q.x + x*q.w + y*q.z - z*q.y,
            w*q.y - x*q.z + y*q.w + z*q.x,
            w*q.z + x*q.y - y*q.x + z*q.w
        );
    }

    Quaternion operator+(const Quaternion& q) const {
        return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
    }

    Quaternion operator-(const Quaternion& q) const {
        return Quaternion(w - q.w, x - q.x, y - q.y, z - q.z);
    }

    Quaternion operator/(double scalar) const {
        return Quaternion(w / scalar, x / scalar, y / scalar, z / scalar);
    }

    Quaternion operator*(double scalar) const {
        return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
    }

    Quaternion scale(double scalar) const {
        return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
    }

    double w, x, y, z;
};
