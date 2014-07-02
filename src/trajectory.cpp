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

//Abstract
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

void TrajectoryReaderJointAbstract::resetIterator(){
    if(!data.empty()){
        current_data = data.begin();
    }

}

//Fulljoint
TrajectoryReaderFullJoint::TrajectoryReaderFullJoint(orc::FullTargetState* targetstate, desired_objective_t obj, unsigned int internaldof, std::string filepath){
    FTS = targetstate;
    objective = obj;
    size_q = internaldof;
    dumpFile(filepath);
    resetIterator();
}

TrajectoryReaderFullJoint::~TrajectoryReaderFullJoint(){

}

void TrajectoryReaderFullJoint::update(){
    if (objective == position){
        if (current_data != data.end()){
            FTS->set_q(*current_data);
            current_data++;
        }
    }
    else if (objective == velocity){
        if (current_data != data.end()){
            FTS->set_qdot(*current_data);
            current_data++;
        }
    }
    else if (objective == acceleration){
        if (current_data != data.end()){
            FTS->set_qddot(*current_data);
            current_data++;
        }
    }
    else if (objective == torque){
        if (current_data != data.end()){
            FTS->set_tau(*current_data);
            current_data++;
        }
    }
    return;
}
