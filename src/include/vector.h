#pragma once

#include <iostream>
#include <math.h>

class Vec3 {
public:
    Vec3(): x(0.), y(0.), z(0.) {}
    // Vec3(std::initializer_list<double> a): x(a[0]), y(a[1]), z(a[2]) {}
    Vec3(double a, double b, double c): x(a), y(b), z(c) {}
    Vec3 cross(const Vec3& v) const {
        return Vec3(
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x
        );
    }

    double dot(const Vec3& v) const {
        return x*v.x + y*v.y + z*v.z;
    }

    Vec3& operator=(const Vec3& v) {
        x = v.x;
        y = v.y;
        z = v.z;
        return *this;
    }

    double& operator[](int n) {
        if (n == 0) return x;
        if (n == 1) return y;
        // if (n == 2) return z;
        return z;
    }

    double magnitude() const {
        return sqrt(x*x + y*y + z*z);
    }

    void normalize() {
        double mag = magnitude();
        if (isnan(mag) || mag == 0.0)
            return;

        mag = 1.0/mag;
        x *= mag;
        y *= mag;
        z *= mag;
    }

    Vec3 operator+(const Vec3& v) const {
        return Vec3(
            x+v.x,
            y+v.y,
            z+v.z
        );
    }

    Vec3 operator-(const Vec3& v) const {
        return Vec3(
            x-v.x,
            y-v.y,
            z-v.z
        );
    }

    Vec3 operator*(double scalar) const {
        return Vec3(
            x*scalar,
            y*scalar,
            z*scalar
        );
    }

    Vec3 operator/(double scalar) const {
        return Vec3(
            x/scalar,
            y/scalar,
            z/scalar
        );
    }

    double x, y, z;
};

std::ostream& operator<< (std::ostream &out, Vec3 const& v) {
    out << '[' << v.x << ' ' << v.y << ' ' << v.z << ']';
    return out;
}
