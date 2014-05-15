// Filename:  mathutils.cpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:

#include "ISIRTaskManager/mathutils.hpp"

Eigen::Rotation3d RollPitchYaw2Quaternion(double r, double p, double y){
    double cph = std::cos(r/2.0);
    double sph = std::sin(r/2.0);

    double cth = std::cos(p/2.0);
    double sth = std::sin(p/2.0);

    double cps = std::cos(y/2.0);
    double sps = std::sin(y/2.0);
    
    double q0 = cph*cth*cps + sph*sth*sps;
    double q1 = sph*cth*cps - cph*sth*sps;
    double q2 = cph*sth*cps + sph*cth*sps;
    double q3 = cph*cth*sps - sph*sth*cps;

    Eigen::Rotation3d q(q0, q1, q2, q3);
    return q;
}

Eigen::Rotation3d RollPitchYaw2Quaternion(Eigen::Vector3d rpy){
    return RollPitchYaw2Quaternion(rpy[0], rpy[1], rpy[2]);
}


