// Filename:  trajectory.cpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  


#include "ISIRTaskManager/trajectory.hpp"
#include <iostream>
#include <fstream>
#include <iterator>
#include <boost/filesystem.hpp>
using namespace boost::filesystem;

TrajectoryReaderFullJoint::TrajectoryReaderFullJoint(fullstate_task_t* taskdescription, unsigned int internaldof, std::string filepath){
    taskdesc = taskdescription;
    size_q = internaldof;
    dumpFile(filepath);
}

void TrajectoryReaderJointAbstract::dumpFile(std::string filepath){
    path p(filepath);
    if (exists(p)){
        if (is_regular_file(p)){
            std::ifstream traj_file(filepath.c_str());
            std::string line;
            while(std::getline(traj_file, line)){
                std::istringstream iss(line);
                Eigen::VectorXd data_line(size_q);
                for (unsigned int i=0; i<size_q; ++i){
                    iss >> data_line(i);
                }
                data.push_back(data_line);
            }
            return;
        }
        else{
            std::cout << p << " is a not a regular file\n";
            return;
        }
    }
    else{
        std::cout << p << " does not exist\n";
        return;
    }
}

void TrajectoryReaderFullJoint::update(){
    return;
}
