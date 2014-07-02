// Filename:  trajectory.hpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  Trajectory handler for ISIRTask

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <orc/control/FullState.h>
#include <vector>
#include <iostream>

enum desired_objective_t {
    position,
    velocity,
    acceleration,
    torque
};

class Trajectory{
    public:
        //Trajectory(){};
        //~Trajectory(){};
        virtual void update() = 0;
};

class TrajectoryReaderJointAbstract : public Trajectory{
    public:
        unsigned int size_q;
        desired_objective_t objective;
        std::vector<Eigen::VectorXd> data;
        std::vector<Eigen::VectorXd>::iterator current_data;

        void dumpFile(std::string filepath);
        void resetIterator();
};

class TrajectoryReaderFullJoint : public TrajectoryReaderJointAbstract{
    public:
        TrajectoryReaderFullJoint(orc::FullTargetState* targetstate, desired_objective_t obj, unsigned int internaldof, std::string filepath);
        ~TrajectoryReaderFullJoint();

        orc::FullTargetState* FTS;

        void update();
};

#endif
