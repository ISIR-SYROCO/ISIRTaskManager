// Filename:  mathutils.hpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description: 

#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <cmath>
#include <Eigen/Lgsm>

Eigen::Rotation3d RollPitchYaw2Quaternion(double r, double p, double y);
Eigen::Rotation3d RollPitchYaw2Quaternion(Eigen::Vector3d rpy);
Eigen::Vector3d Quaternion2RollPitchYaw(Eigen::Rotation3d q);

#endif
