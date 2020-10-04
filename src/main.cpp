#include <vector.h>
#include <matrix.h>
#include <ahrs.h>
#include <iostream>

using namespace std;

int main(){
    Vec3 a(1,2,3);
    Vec3 b(4,5,6);
    cout << a << endl;
    cout << a+b << endl;

    Mat3 m;
    cout << m << endl;

    for (int i=0; i<9; ++i) m.p[i] = double(i);
    cout << m << endl;
    cout << m.T() << endl;

    return 0;
}
