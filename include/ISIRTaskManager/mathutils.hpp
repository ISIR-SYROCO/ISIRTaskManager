// Filename:  mathutils.hpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description: 

#include <cmath>
#include <Eigen/Lgsm>

Eigen::Rotation3d RollPitchYaw2Quaternion(double r, double p, double y);
