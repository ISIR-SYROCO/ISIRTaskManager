// Filename:  trajectory.hpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  Trajectory handler for ISIRTask

#include "taskxmlparser.hpp"

class TrajectoryReaderAbstract{
    public:
        virtual void update() = 0;
        virtual void dumpFile(std::string filepath) = 0;
};

class TrajectoryReaderJointAbstract{
    public:
        unsigned int size_q;
        std::vector<Eigen::VectorXd> data;
        std::vector<Eigen::VectorXd>::iterator current_data;

        void dumpFile(std::string filepath);
        void resetIterator();
};

class TrajectoryReaderFullJoint : TrajectoryReaderJointAbstract{
    public:
        TrajectoryReaderFullJoint(fullstate_task_t* taskdescription, unsigned int internaldof, std::string filepath);
        ~TrajectoryReaderFullJoint();

        fullstate_task_t* taskdesc;

        void update();
};
