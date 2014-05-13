// Filename:  testquaternion.cpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:

#include <Eigen/Lgsm>
#include <iostream>
#include "ISIRTaskManager/mathutils.hpp"

int main(int argc, char** argv){
    Eigen::Rotation3d q = RollPitchYaw2Quaternion(1.2, -0.5, 3.6);
    std::cout << q << std::endl ;
    Eigen::Vector3d v(1.0, 1.2, 1.3);
    Eigen::Displacementd d(v, q);
    std::cout << d << std::endl;
    Eigen::Displacementd d2;

    d2.x() = 1.0;
    d2.y() = 1.2;
    d2.z() = 1.3;
    d2.qx() = 0;
    d2.qy() = 0;
    d2.qz() = 0;
    d2.qw() = 1.0;
    std::cout << d2 << std::endl;
    Eigen::AngularVelocityd avd(v);
    return 0;
}
