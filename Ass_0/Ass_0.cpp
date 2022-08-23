#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

#include "opencv2/core.hpp"

int main() {

    // // Basic Example of cpp
    // std::cout << "Example of cpp \n";
    // float a = 1.0, b = 2.0;
    // std::cout << a << std::endl;
    // std::cout << a/b << std::endl;
    // std::cout << std::sqrt(b) << std::endl;
    // std::cout << std::acos(-1) << std::endl;
    // std::cout << std::sin(30.0/180.0*acos(-1)) << std::endl;

    // // Example of vector
    // std::cout << "Example of vector \n";
    // // vector definition
    // Eigen::Vector3f v(1.0f,2.0f,3.0f);
    // Eigen::Vector3f w(1.0f,0.0f,0.0f);
    // // vector output
    // std::cout << "Example of output \n";
    // std::cout << v << std::endl;
    // // vector add
    // std::cout << "Example of add \n";
    // std::cout << v + w << std::endl;
    // // vector scalar multiply
    // std::cout << "Example of scalar multiply \n";
    // std::cout << v * 3.0f << std::endl;
    // std::cout << 2.0f * v << std::endl;

    // // Example of matrix
    // std::cout << "Example of matrix \n";
    // // matrix definition
    // Eigen::Matrix3f i,j;
    // i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    // j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    // // matrix output
    // std::cout << "Example of output \n";
    // std::cout << i << std::endl;
    // matrix add i + j
    // matrix scalar multiply i * 2.0
    // matrix multiply i * j
    // matrix multiply vector i * v


    /*
    * PA 0
    */
    std::cout << "Define hg_point P(2,1,0,1) (*Using hg)" << std::endl;
    Eigen::Matrix4f hg_P;
    hg_P << 2, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 1;
    // Eigen::Vector3f p(2.0f,1.0f,1.0f);
    std::cout << "hg_P =" << std::endl <<
        hg_P << std::endl;

    std::cout << "Define rotation matrix M to roate -45d" << std::endl;
    Eigen::Matrix4f hg_R;
    double degree = 45.0 / 180.0 * M_PI;
    hg_R << cos(degree), -sin(degree), 0, 0,
        sin(degree), cos(degree), 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 1;
    std::cout << "hg_R =" << std::endl <<
        hg_R << std::endl;

    std::cout << "Define Translation matrix T to translate(1,2)" << std::endl;
    Eigen::Matrix4f hg_T;
    hg_T << 1, 0, 0, 1,
        0, 1, 0, 2,
        0, 0, 0, 0,
        0, 0, 0, 1;
    std::cout << "hg_T =" << std::endl <<
        hg_T << std::endl;

    std::cout << "Tranformation Process (Carefully to the Order: Translate -> Rotate -> p)" << std::endl;
    // TO DO: M * P
    hg_P = hg_T * hg_R * hg_P;
    std::cout << "p_prime =" << std::endl <<
        hg_P << std::endl;

    return 0;
}