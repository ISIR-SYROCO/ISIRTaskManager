// Filename:  taskxmlparser.cpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:

#include "ISIRTaskManager/taskxmlparser.hpp"
#include "ISIRTaskManager/mathutils.hpp"
#include <boost/filesystem.hpp>
#include <boost/assign/list_of.hpp>
using namespace boost::assign;
using namespace boost::filesystem;
#include <iostream>
#include <orc/control/Feature.h>
#include <orc/control/FullState.h>
#include <orc/control/ControlFrame.h>
#include <orc/control/ControlEnum.h>

#include <orcisir/Features/ISIRFeature.h>


TaskXMLParser::TaskXMLParser(){

}

TaskXMLParser::~TaskXMLParser(){

}

TaskXMLParser::TaskXMLParser(std::string filepath, orcisir::ISIRController &controller)
    : ctrl(&controller){
    str_frame_dof_map = map_list_of
    ("X", orc::X)
    ("Y", orc::Y)
    ("XY", orc::XY)
    ("Z", orc::Z)
    ("XZ", orc::XZ)
    ("YZ", orc::YZ)
    ("XYZ", orc::XYZ);

    loadFile(filepath);
    std::cout << ctrl->getName() << std::endl;
}

bool TaskXMLParser::loadFile(std::string filepath){
    path p(filepath);
    if (exists(p)){
        if (is_regular_file(p)){
            std::cout << p << " is a regular file\n";
            if (!taskfile.LoadFile(p.string())){
                std::cout << "Cannot load xml file\n";
                return false;
            }
            parse();
            return true;
        }
        else{
            std::cout << p << " is a not a regular file\n";
            return false;
        }
    }
    else{
        std::cout << p << " does not exist\n";
        return false;
    }
}

bool TaskXMLParser::parse(){
    TiXmlHandle docHandle(&taskfile);
    TiXmlElement* task = docHandle.FirstChild("tasks").FirstChild("task").ToElement();
    
    for(task; task; task = task->NextSiblingElement()){
        //Check if the task already exists
        boost::ptr_map< std::string, task_t >::iterator taskdesc_map_iter = taskdesc_map.find(task->Attribute("id"));
        if(taskdesc_map_iter == boost::end(taskdesc_map)){
            addTask(*task);
        }
        else{
            updateTask(*task, *taskdesc_map_iter->second);
        }
    }

    return true;
}

bool TaskXMLParser::parseTaskInfo(TiXmlElement const& task_node, task_t& taskdesc){
    if(task_node.Attribute("id") == NULL){
        std::cout << "Task must have id" << std::endl;
        return false;
    }
    else{
        taskdesc.id = task_node.Attribute("id"); 
    }

    if(task_node.Attribute("type") == NULL){
        taskdesc.type = "";
    }
    else{
        taskdesc.type = task_node.Attribute("type");
    }
    return true;
}

bool TaskXMLParser::parseParam(TiXmlElement const& param_node, task_t& taskdesc){
    if (&param_node == NULL){
        taskdesc.w = 1;
        taskdesc.kp = 0;
        taskdesc.kd = 0;
    }
    else{
        std::istringstream w_ss(param_node.Attribute("w"));
        w_ss >> taskdesc.w;

        std::istringstream kp_ss(param_node.Attribute("kp"));
        kp_ss >> taskdesc.kp;

        std::istringstream kd_ss(param_node.Attribute("kd"));
        kd_ss >> taskdesc.kd;
    }

    /*
    std::cout << taskdesc.id << ":" << taskdesc.type << "\n\t" 
                         << " w = " << taskdesc.w << "\n\t"
                         << " kp = " << taskdesc.kp << "\n\t"
                         << " kd = " << taskdesc.kd << "\n";
    */
    return true;
}

template<class T>
void TaskXMLParser::parseLocalOffset(TiXmlElement const& local_offset_node, T& taskdesc){
    if( &local_offset_node == NULL ){
        taskdesc.offset.x() = 0;
        taskdesc.offset.y() = 0;
        taskdesc.offset.z() = 0;
        taskdesc.offset.qx() = 0;
        taskdesc.offset.qy() = 0;
        taskdesc.offset.qz() = 0;
        taskdesc.offset.qw() = 1;
    }
    else{
        Eigen::Vector3d xyz;
        fillVector3(&local_offset_node, "xyz", xyz);

        Eigen::Vector3d rpy;
        fillVector3(&local_offset_node, "rpy", rpy);

        Eigen::Rotation3d rot = RollPitchYaw2Quaternion(rpy[0], rpy[1], rpy[2]);

        Eigen::Displacementd displ(xyz, rot);
        taskdesc.offset = displ;
    }
    return;
}

bool TaskXMLParser::parseFeatureFullState(TiXmlElement const& feature_node, fullstate_task_t& taskdesc){
    int part;
    TiXmlElement const* part_node = feature_node.FirstChildElement("part");
    if (part_node == NULL){
        std::cout << taskdesc.id << " : Part not specified in feature" << std::endl;
        return false;
    }
    std::string whatPart(part_node->Attribute("value"));
    if (whatPart == "INTERNAL"){
        part = orc::FullState::INTERNAL;
    }
    else if (whatPart == "FULL_STATE"){
        part = orc::FullState::FULL_STATE;
    }
    else if (whatPart == "FREE_FLYER"){
        part = orc::FullState::FREE_FLYER;
    }
    else{
        std::cout << taskdesc.id << " : incorrect part in feature" << std::endl;
        return false;
    }
    taskdesc.FMS = new orc::FullModelState(taskdesc.id+".FModelState", ctrl->getModel(), part);

    taskdesc.FTS = new orc::FullTargetState (taskdesc.id+".FTargetState", ctrl->getModel(), part);

    parseObjectiveFullState(feature_node, taskdesc);

    taskdesc.feat = new orc::FullStateFeature (taskdesc.id+".feat", *(taskdesc.FMS));
    taskdesc.featdes = new orc::FullStateFeature (taskdesc.id+"featdes", *(taskdesc.FTS));
    return true;
}

bool TaskXMLParser::parseObjectiveFullState(TiXmlElement const& feature_node, fullstate_task_t& taskdesc){
    TiXmlElement const* qdes_node = feature_node.FirstChildElement("objective")->FirstChildElement("q_des");
    if (fillVector(qdes_node, ctrl->getModel().nbInternalDofs(), taskdesc.q_des)){
        taskdesc.FTS->set_q(taskdesc.q_des);
    }

    //qd_des
    TiXmlElement const* qddes_node = feature_node.FirstChildElement("objective")->FirstChildElement("qd_des");
    if (fillVector(qddes_node, ctrl->getModel().nbInternalDofs(), taskdesc.qd_des)){
        taskdesc.FTS->set_qdot(taskdesc.qd_des);
    }

    //qdd_des
    TiXmlElement const* qdddes_node = feature_node.FirstChildElement("objective")->FirstChildElement("qdd_des");
    if (fillVector(qdddes_node, ctrl->getModel().nbInternalDofs(), taskdesc.qdd_des)){
        taskdesc.FTS->set_qddot(taskdesc.qdd_des);
    }

    //tau_des
    TiXmlElement const* taudes_node = feature_node.FirstChildElement("objective")->FirstChildElement("tau_des");
    if (fillVector(taudes_node, ctrl->getModel().nbInternalDofs(), taskdesc.tau_des)){
        taskdesc.FTS->set_tau(taskdesc.tau_des);
    }

    return true;
}

bool TaskXMLParser::fillVector(TiXmlElement const* node, unsigned int vector_size, Eigen::VectorXd& vector_result){
    vector_result.resize(vector_size);
    if (node != NULL){
        if (node->Attribute("value") != NULL){
            std::istringstream ss(node->Attribute("value"));
            for (unsigned int i=0; i<vector_size; i++){
                if(!ss.eof()){
                    ss >> vector_result[i];
                }
                else{
                    return false;
                }
            }
        }
        else{
            return false;
        }
        return true;
    }
    else{
        return false;
    }
}

bool TaskXMLParser::fillVector3(TiXmlElement const* node, const char * attribute, Eigen::Vector3d& vector_result){
    if (node != NULL){
        if (node->Attribute(attribute) != NULL){
            std::istringstream ss(node->Attribute(attribute));
            for (unsigned int i=0; i<3; i++){
                if(!ss.eof()){
                    ss >> vector_result[i];
                }
                else{
                    return false;
                }
            }
        }
        else{
            return false;
        }
        return true;
    }
    else{
        return false;
    }
}

void TaskXMLParser::initTask(orcisir::ISIRTask& task, task_t& taskdesc){
    if (taskdesc.type == "ACCELERATION"){
        task.initAsAccelerationTask();
        ctrl->addTask(task);
        task.activateAsObjective();
        task.setStiffness(taskdesc.kp);
        task.setDamping(taskdesc.kd);
        task.setWeight(taskdesc.w);
    }
    else if (taskdesc.type == "TORQUE"){
        task.initAsTorqueTask();
        ctrl->addTask(task);
        task.activateAsObjective();
        task.setStiffness(taskdesc.kp);
        task.setDamping(taskdesc.kd);
        task.setWeight(taskdesc.w);
    }
    else if (taskdesc.type == "FORCE"){
        std::cout << taskdesc.id << " : TODO" << taskdesc.type << std::endl;
    }
    else{
        std::cout << taskdesc.id << " : Unsupported " << taskdesc.type << std::endl;
    }

}

bool TaskXMLParser::parseFeaturePartialState(TiXmlElement const& feature_node, partialstate_task_t& taskdesc){
    int part;
    TiXmlElement const* part_node = feature_node.FirstChildElement("part");
    if (part_node == NULL){
        std::cout << taskdesc.id << " : Part not specified in feature" << std::endl;
        return false;
    }
    std::string whatPart(part_node->Attribute("value"));
    if (whatPart == "INTERNAL"){
        part = orc::FullState::INTERNAL;
    }
    else if (whatPart == "FULL_STATE"){
        part = orc::FullState::FULL_STATE;
    }
    else if (whatPart == "FREE_FLYER"){
        part = orc::FullState::FREE_FLYER;
    }
    else{
        std::cout << taskdesc.id << " : incorrect part in feature" << std::endl;
        return false;
    }

    TiXmlElement const* dofs_node = feature_node.FirstChildElement("dofs");
    if (dofs_node == NULL){
        std::cout << taskdesc.id << " : Selected dogs not specified in featuren node" << std::endl;
        return false;
    }

    if (dofs_node->Attribute("value") != NULL){
        std::istringstream dofs_ss(dofs_node->Attribute("value"));
        std::istream_iterator<int> begin_dofs(dofs_ss), end_dofs;
        std::vector<int> dofs_value(begin_dofs, end_dofs);
        taskdesc.sdofs.resize(dofs_value.size());
        for (unsigned int i=0; i<dofs_value.size(); i++){
            taskdesc.sdofs[i] = dofs_value[i];
        }
        
    }
    else{
        std::cout << taskdesc.id << " : dofs value missing" << std::endl;
        return false;
    }

    taskdesc.PMS = new orcisir::PartialModelState(taskdesc.id+".PModelState", ctrl->getModel(), taskdesc.sdofs, part);

    taskdesc.PTS = new orcisir::PartialTargetState(taskdesc.id+".PTargetState", ctrl->getModel(), taskdesc.sdofs, part);

    parseObjectivePartialState(feature_node, taskdesc);

    taskdesc.feat = new orcisir::PartialStateFeature (taskdesc.id+".feat", *(taskdesc.PMS));
    taskdesc.featdes = new orcisir::PartialStateFeature (taskdesc.id+"featdes", *(taskdesc.PTS));
    return true;
}

bool TaskXMLParser::parseObjectivePartialState(TiXmlElement const& feature_node, partialstate_task_t& taskdesc){
    TiXmlElement const* qdes_node = feature_node.FirstChildElement("objective")->FirstChildElement("q_des");
    if(fillVector(qdes_node, taskdesc.sdofs.size(), taskdesc.q_des)){
        taskdesc.PTS->set_q(taskdesc.q_des);
    }

    //qd_des
    TiXmlElement const* qddes_node = feature_node.FirstChildElement("objective")->FirstChildElement("qd_des");
    if (fillVector(qddes_node, taskdesc.sdofs.size(), taskdesc.qd_des)){
        taskdesc.PTS->set_qdot(taskdesc.qd_des);
    }


    //qdd_des
    TiXmlElement const* qdddes_node = feature_node.FirstChildElement("objective")->FirstChildElement("qdd_des");
    if (fillVector(qdddes_node, taskdesc.sdofs.size(), taskdesc.qdd_des)){
        taskdesc.PTS->set_qddot(taskdesc.qdd_des);
    }

    //tau_des
    TiXmlElement const* taudes_node = feature_node.FirstChildElement("objective")->FirstChildElement("tau_des");
    if (fillVector(taudes_node, taskdesc.sdofs.size(), taskdesc.tau_des)){
        taskdesc.PTS->set_tau(taskdesc.tau_des);
    }

    return true;
}

void TaskXMLParser::parseFrameTaskDofs(TiXmlElement const& dof_node, bool& rotation, orc::ECartesianDof& cartesian_dofs){
    std::string dofs(dof_node.Attribute("value"));
    if (dofs.at(0) == 'R'){
        rotation = true;
        dofs.erase(dofs.begin());
        if (!dofs.empty()){
            std::map<std::string, orc::ECartesianDof>::iterator it;
            it = str_frame_dof_map.find(dofs);
            if ( it != str_frame_dof_map.end() ){
                //Displacement feature
                cartesian_dofs = it->second;
            }
            else{
                //Invalid cartesian dofs
                cartesian_dofs = orc::NONE;
            }
        }
        else{
            //Orientation feature
            cartesian_dofs = orc::NONE;
        }
    }
    else{
        rotation = false;
        if (!dofs.empty()){
            std::map<std::string, orc::ECartesianDof>::iterator it;
            it = str_frame_dof_map.find(dofs);
            if ( it != str_frame_dof_map.end() ){
                //Position feature
                cartesian_dofs = it->second;
            }
            else{
                //Invalid cartesian dofs
                cartesian_dofs = orc::NONE;
            }
        }
        else{
            //Invalid cartesian
            cartesian_dofs = orc::NONE;
        }
    }
    return;
}

bool TaskXMLParser::parseObjectiveDisplacementFrame(TiXmlElement const& feature_node, frame_task_t& taskdesc){
    TiXmlElement const* objective_node = feature_node.FirstChildElement("objective");
    if (objective_node != NULL){
        TiXmlElement const* pos_des_node = objective_node->FirstChildElement("pos_des");
        if (pos_des_node != NULL){
            Eigen::Vector3d xyz, rpy;
            fillVector3(pos_des_node, "xyz", xyz);
            fillVector3(pos_des_node, "rpy", rpy);
            Eigen::Rotation3d rot = RollPitchYaw2Quaternion(rpy[0], rpy[1], rpy[2]);

            Eigen::Displacementd pos_des_displ(xyz, rot);
            taskdesc.position_des = pos_des_displ;
            taskdesc.TF->setPosition(taskdesc.position_des);
        }

        TiXmlElement const* vel_des_node = objective_node->FirstChildElement("vel_des");
        if (vel_des_node != NULL){
            Eigen::Vector3d xyz, rxyz;
            fillVector3(vel_des_node, "xyz", xyz);
            fillVector3(vel_des_node, "rxyz", rxyz);
            Eigen::AngularVelocityd avel(rxyz);

            Eigen::Twistd vel_des_twist(avel, xyz);
            taskdesc.velocity_des = vel_des_twist;
            taskdesc.TF->setVelocity(taskdesc.velocity_des);
        }

        TiXmlElement const* acc_des_node = objective_node->FirstChildElement("acc_des");
        if (acc_des_node != NULL){
            Eigen::Vector3d xyz, rxyz;
            fillVector3(acc_des_node, "xyz", xyz);
            fillVector3(acc_des_node, "rxyz", rxyz);
            Eigen::AngularVelocityd avel(rxyz);

            Eigen::Twistd acc_des_twist(avel, xyz);
            taskdesc.acceleration_des = acc_des_twist;
            taskdesc.TF->setAcceleration(taskdesc.acceleration_des);
        }

        TiXmlElement const* wrench_des_node = objective_node->FirstChildElement("wrench_des");
        if (wrench_des_node != NULL){
            Eigen::Vector3d force, torque;
            fillVector3(wrench_des_node, "force", force);
            fillVector3(wrench_des_node, "torque", torque);

            Eigen::Wrenchd wrench_des(torque, force);
            taskdesc.wrench_des = wrench_des;
            taskdesc.TF->setWrench(taskdesc.wrench_des);
        }

    }
    else{
        std::cout << taskdesc.id << " : No objective in feature node" << std::endl;
        return false;
    }


    return true;
}

bool TaskXMLParser::parseObjectiveOrientationFrame(TiXmlElement const& feature_node, frame_task_t& taskdesc){
    TiXmlElement const* objective_node = feature_node.FirstChildElement("objective");
    if (objective_node != NULL){
        TiXmlElement const* pos_des_node = objective_node->FirstChildElement("pos_des");
        if (pos_des_node != NULL){
            Eigen::Vector3d xyz, rpy;
            xyz << 0, 0, 0;
            fillVector3(pos_des_node, "rpy", rpy);
            Eigen::Rotation3d rot = RollPitchYaw2Quaternion(rpy[0], rpy[1], rpy[2]);

            Eigen::Displacementd pos_des_displ(xyz, rot);
            taskdesc.position_des = pos_des_displ;
            taskdesc.TF->setPosition(taskdesc.position_des);
        }

        TiXmlElement const* vel_des_node = objective_node->FirstChildElement("vel_des");
        if (vel_des_node != NULL){
            Eigen::Vector3d xyz, rxyz;
            xyz << 0, 0, 0;
            fillVector3(vel_des_node, "rxyz", rxyz);
            Eigen::AngularVelocityd avel(rxyz);

            Eigen::Twistd vel_des_twist(avel, xyz);
            taskdesc.velocity_des = vel_des_twist;
            taskdesc.TF->setVelocity(taskdesc.velocity_des);
        }

        TiXmlElement const* acc_des_node = objective_node->FirstChildElement("acc_des");
        if (acc_des_node != NULL){
            Eigen::Vector3d xyz, rxyz;
            xyz << 0, 0, 0;
            fillVector3(acc_des_node, "rxyz", rxyz);
            Eigen::AngularVelocityd avel(rxyz);

            Eigen::Twistd acc_des_twist(avel, xyz);
            taskdesc.acceleration_des = acc_des_twist;
            taskdesc.TF->setAcceleration(taskdesc.acceleration_des);
        }

        TiXmlElement const* wrench_des_node = objective_node->FirstChildElement("wrench_des");
        if (wrench_des_node != NULL){
            Eigen::Vector3d force, torque;
            fillVector3(wrench_des_node, "force", force);
            fillVector3(wrench_des_node, "torque", torque);

            Eigen::Wrenchd wrench_des(torque, force);
            taskdesc.wrench_des = wrench_des;
            taskdesc.TF->setWrench(taskdesc.wrench_des);
        }

    }
    else{
        std::cout << taskdesc.id << " : No objective in feature node" << std::endl;
        return false;
    }

    return true;
}

bool TaskXMLParser::parseObjectivePositionFrame(TiXmlElement const& feature_node, frame_task_t& taskdesc){
    TiXmlElement const* objective_node = feature_node.FirstChildElement("objective");
    if (objective_node != NULL){
        TiXmlElement const* pos_des_node = objective_node->FirstChildElement("pos_des");
        if (pos_des_node != NULL){
            Eigen::Vector3d xyz;
            fillVector3(pos_des_node, "xyz", xyz);

            Eigen::Rotation3d rot(1, 0, 0, 0);

            Eigen::Displacementd pos_des_displ(xyz, rot);
            taskdesc.position_des = pos_des_displ;
            taskdesc.TF->setPosition(taskdesc.position_des);
        }

        TiXmlElement const* vel_des_node = objective_node->FirstChildElement("vel_des");
        if (vel_des_node != NULL){
            Eigen::Vector3d xyz, rxyz;
            fillVector3(vel_des_node, "xyz", xyz);
            rxyz << 0, 0, 0;
            Eigen::AngularVelocityd avel(rxyz);

            Eigen::Twistd vel_des_twist(avel, xyz);
            taskdesc.velocity_des = vel_des_twist;
            taskdesc.TF->setVelocity(taskdesc.velocity_des);
        }

        TiXmlElement const* acc_des_node = objective_node->FirstChildElement("acc_des");
        if (acc_des_node != NULL){
            Eigen::Vector3d xyz, rxyz;
            xyz << 0, 0, 0;
            fillVector3(acc_des_node, "rxyz", rxyz);
            Eigen::AngularVelocityd avel(rxyz);

            Eigen::Twistd acc_des_twist(avel, xyz);
            taskdesc.acceleration_des = acc_des_twist;
            taskdesc.TF->setAcceleration(taskdesc.acceleration_des);
        }

        TiXmlElement const* wrench_des_node = objective_node->FirstChildElement("wrench_des");
        if (wrench_des_node != NULL){
            Eigen::Vector3d force, torque;
            fillVector3(wrench_des_node, "force", force);
            fillVector3(wrench_des_node, "torque", torque);

            Eigen::Wrenchd wrench_des(torque, force);
            taskdesc.wrench_des = wrench_des;
            taskdesc.TF->setWrench(taskdesc.wrench_des);
        }

    }
    else{
        std::cout << taskdesc.id << " : No objective in feature node" << std::endl;
        return false;
    }

    return true;
}

bool TaskXMLParser::parseObjectiveCoM(TiXmlElement const& feature_node, com_task_t& taskdesc){
    TiXmlElement const* objective_node = feature_node.FirstChildElement("objective");
    if (objective_node != NULL){
        TiXmlElement const* pos_des_node = objective_node->FirstChildElement("pos_des");
        if (pos_des_node != NULL){
            Eigen::Vector3d xyz;
            fillVector3(pos_des_node, "xyz", xyz);

            Eigen::Rotation3d rot(1, 0, 0, 0);

            Eigen::Displacementd pos_des_displ(xyz, rot);
            taskdesc.position_des = pos_des_displ;
            taskdesc.TF->setPosition(taskdesc.position_des);
        }

        TiXmlElement const* vel_des_node = objective_node->FirstChildElement("vel_des");
        if (vel_des_node != NULL){
            Eigen::Vector3d xyz, rxyz;
            fillVector3(vel_des_node, "xyz", xyz);
            rxyz << 0, 0, 0;
            Eigen::AngularVelocityd avel(rxyz);

            Eigen::Twistd vel_des_twist(avel, xyz);
            taskdesc.velocity_des = vel_des_twist;
            taskdesc.TF->setVelocity(taskdesc.velocity_des);
        }

        TiXmlElement const* acc_des_node = objective_node->FirstChildElement("acc_des");
        if (acc_des_node != NULL){
            Eigen::Vector3d xyz, rxyz;
            xyz << 0, 0, 0;
            fillVector3(acc_des_node, "rxyz", rxyz);
            Eigen::AngularVelocityd avel(rxyz);

            Eigen::Twistd acc_des_twist(avel, xyz);
            taskdesc.acceleration_des = acc_des_twist;
            taskdesc.TF->setAcceleration(taskdesc.acceleration_des);
        }

    }
    else{
        std::cout << taskdesc.id << " : No objective in feature node" << std::endl;
        return false;
    }

    return true;
}

bool TaskXMLParser::parseFeatureDisplacement(TiXmlElement const& feature_node, displacement_task_t& taskdesc){
    TiXmlElement const* segment_node = feature_node.FirstChildElement("segment");
    if(segment_node == NULL){
        std::cout << taskdesc.id << " : No segment node in feature node" << std::endl;
        return false;
    }
    if (segment_node->Attribute("name") != NULL){
        taskdesc.segment_name = segment_node->Attribute("name"); 
    }
    else{
        std::cout << taskdesc.id << " : No segment name" << std::endl;
        return false;
    }
    //Parse local offset
    TiXmlElement const* local_offset_node = feature_node.FirstChildElement("local_offset");
    parseLocalOffset(*local_offset_node, taskdesc);

    //Creation of features
    taskdesc.SF = new orc::SegmentFrame(taskdesc.id+".SFrame", ctrl->getModel(), taskdesc.segment_name, taskdesc.offset);
    taskdesc.TF = new orc::TargetFrame(taskdesc.id+".TFrame", ctrl->getModel());

    //Parse objectives
    parseObjectiveDisplacementFrame(feature_node, taskdesc);

    taskdesc.feat = new orc::DisplacementFeature(taskdesc.id+".DisplacementFeature", *(taskdesc.SF),  taskdesc.dofs);
    taskdesc.featdes = new orc::DisplacementFeature(taskdesc.id+".DisplacementFeature", *(taskdesc.TF),  taskdesc.dofs);

    return true;
}

bool TaskXMLParser::parseFeatureOrientation(TiXmlElement const& feature_node, orientation_task_t& taskdesc){
    TiXmlElement const* segment_node = feature_node.FirstChildElement("segment");
    if(segment_node == NULL){
        std::cout << taskdesc.id << " : No segment node in feature node" << std::endl;
        return false;
    }
    if (segment_node->Attribute("name") != NULL){
        taskdesc.segment_name = segment_node->Attribute("name"); 
    }
    else{
        std::cout << taskdesc.id << " : No segment name" << std::endl;
        return false;
    }
    //Parse local offset
    TiXmlElement const* local_offset_node = feature_node.FirstChildElement("local_offset");
    parseLocalOffset(*local_offset_node, taskdesc);

    //Creation of features
    taskdesc.SF = new orc::SegmentFrame(taskdesc.id+".SFrame", ctrl->getModel(), taskdesc.segment_name, taskdesc.offset);
    taskdesc.TF = new orc::TargetFrame(taskdesc.id+".TFrame", ctrl->getModel());

    //Parse objectives
    parseObjectiveOrientationFrame(feature_node, taskdesc);

    taskdesc.feat = new orc::OrientationFeature(taskdesc.id+".OrientationFeature", *(taskdesc.SF));
    taskdesc.featdes = new orc::OrientationFeature(taskdesc.id+".OrientationFeature", *(taskdesc.TF));

    return true;
}

bool TaskXMLParser::parseFeaturePosition(TiXmlElement const& feature_node, position_task_t& taskdesc){
    TiXmlElement const* segment_node = feature_node.FirstChildElement("segment");
    if(segment_node == NULL){
        std::cout << taskdesc.id << " : No segment node in feature node" << std::endl;
        return false;
    }
    if (segment_node->Attribute("name") != NULL){
        taskdesc.segment_name = segment_node->Attribute("name"); 
    }
    else{
        std::cout << taskdesc.id << " : No segment name" << std::endl;
        return false;
    }
    //Parse local offset
    TiXmlElement const* local_offset_node = feature_node.FirstChildElement("local_offset");
    parseLocalOffset(*local_offset_node, taskdesc);

    //Creation of features
    taskdesc.SF = new orc::SegmentFrame(taskdesc.id+".SFrame", ctrl->getModel(), taskdesc.segment_name, taskdesc.offset);
    taskdesc.TF = new orc::TargetFrame(taskdesc.id+".TFrame", ctrl->getModel());

    //Parse objectives
    parseObjectivePositionFrame(feature_node, taskdesc);

    taskdesc.feat = new orc::PositionFeature(taskdesc.id+".PositionFeature", *(taskdesc.SF),  taskdesc.dofs);
    taskdesc.featdes = new orc::PositionFeature(taskdesc.id+".PositionFeature", *(taskdesc.TF),  taskdesc.dofs);

    return true;
}

bool TaskXMLParser::parseFeatureCom(TiXmlElement const& feature_node, com_task_t& taskdesc){
    //Creation of CoM feature
    taskdesc.CoMF = new orc::CoMFrame(taskdesc.id+"CoMFrame", ctrl->getModel());
    taskdesc.TF = new orc::TargetFrame(taskdesc.id+".TFrame", ctrl->getModel());

    //Parse objectives
    parseObjectiveCoM(feature_node, taskdesc);

    taskdesc.feat = new orc::PositionFeature(taskdesc.id+".PositionFeature", *(taskdesc.CoMF),  taskdesc.dofs);
    taskdesc.featdes = new orc::PositionFeature(taskdesc.id+".PositionFeature", *(taskdesc.TF),  taskdesc.dofs);
    return true;
}

bool TaskXMLParser::parseFeatureContact(TiXmlElement const& feature_node, contact_task_t& taskdesc){
    TiXmlElement const* segment_node = feature_node.FirstChildElement("segment");
    if(segment_node == NULL){
        std::cout << taskdesc.id << " : No segment node in feature node" << std::endl;
        return false;
    }
    if (segment_node->Attribute("name") != NULL){
        taskdesc.segment_name = segment_node->Attribute("name"); 
    }
    else{
        std::cout << taskdesc.id << " : No segment name" << std::endl;
        return false;
    }
    //Parse local offset
    TiXmlElement const* local_offset_node = feature_node.FirstChildElement("local_offset");
    parseLocalOffset(*local_offset_node, taskdesc);
    //Parse Mu and margin
    TiXmlElement const* mu_node = feature_node.FirstChildElement("mu");
    if(mu_node != NULL && mu_node->Attribute("value") != NULL){
        std::istringstream mu_ss(mu_node->Attribute("value"));
        mu_ss >> taskdesc.mu;
    }
    else { 
        std::cout << taskdesc.id << " : Incorrect mu node, using default value" << "\n";
        taskdesc.mu = 0.7;
    }

    TiXmlElement const* margin_node = feature_node.FirstChildElement("margin");
    if(margin_node != NULL && margin_node->Attribute("value") != NULL){
        std::istringstream margin_ss(margin_node->Attribute("value"));
        margin_ss >> taskdesc.margin;
    }
    else { 
        std::cout << taskdesc.id << " : Incorrect margin node, using default value" << "\n";
        taskdesc.mu = 0.;
    }

    //Creation of features
    taskdesc.SF = new orc::SegmentFrame(taskdesc.id+".SFrame", ctrl->getModel(), taskdesc.segment_name, taskdesc.offset);
    taskdesc.feat = new orc::PointContactFeature(taskdesc.id+".ContactFeature", *(taskdesc.SF));

    return true;
}

bool TaskXMLParser::addTask(TiXmlElement const& task_node){
    TiXmlElement const* param_node = task_node.FirstChildElement("param");
    TiXmlElement const* feature_node = task_node.FirstChildElement("feature");
    if (feature_node == NULL){
        std::cout << "No feature!" << std::endl;
        return false;
    }

    std::string featType(feature_node->Attribute("type"));
    if ( featType == "fullstate" ){
        fullstate_task_t* taskdesc = new fullstate_task_t;
        taskdesc->feature_type = "fullstate";
        parseTaskInfo(task_node, *taskdesc);
        parseParam(*param_node, *taskdesc);
        parseFeatureFullState(*feature_node, *taskdesc);

        //std::cout << taskdesc->id << std::endl;
        taskdesc_map.insert(taskdesc->id, taskdesc);

        orcisir::ISIRTask* task;
        task = &(ctrl->createISIRTask(taskdesc->id, *taskdesc->feat, *taskdesc->featdes));
        initTask(*task, *taskdesc);

    }
    else if( featType == "partialstate" ){
        partialstate_task_t* taskdesc = new partialstate_task_t;
        taskdesc->feature_type = "partialstate";
        parseTaskInfo(task_node, *taskdesc);
        parseParam(*param_node, *taskdesc);
        parseFeaturePartialState(*feature_node, *taskdesc);

        taskdesc_map.insert(taskdesc->id, taskdesc);

        orcisir::ISIRTask* task;
        task = &(ctrl->createISIRTask(taskdesc->id, *taskdesc->feat, *taskdesc->featdes));
        initTask(*task, *taskdesc);

    }
    else if( featType == "displacement"){
        //Check which type of feature to create
        TiXmlElement const* dofs_node = feature_node->FirstChildElement("dofs");
        bool rotation_enabled;
        orc::ECartesianDof cartesian_dofs;
        parseFrameTaskDofs(*dofs_node, rotation_enabled, cartesian_dofs);
        displacement_task_t* taskdesc = new displacement_task_t;
        taskdesc->feature_type = "displacement";
        taskdesc->dofs = cartesian_dofs;
        parseTaskInfo(task_node, *taskdesc);
        parseParam(*param_node, *taskdesc);
        orcisir::ISIRTask* task;
        if (rotation_enabled == true){
            if (cartesian_dofs != orc::NONE){
                //Displacement feature
                parseFeatureDisplacement(*feature_node, *taskdesc);

                taskdesc_map.insert(taskdesc->id, taskdesc);
                task = &(ctrl->createISIRTask(taskdesc->id, *taskdesc->feat, *taskdesc->featdes));
                initTask(*task, *taskdesc);
                ////
            }
            else{
                std::cout << taskdesc->id << " : incorrect dof, displacement must have translation component" << std::endl;
            }
        }
        else{
            std::cout << taskdesc->id << " : incorrect dof, displacement must have rotation component" << std::endl;
        }
    }
    else if( featType == "orientation" ){
        //Check which type of feature to create
        TiXmlElement const* dofs_node = feature_node->FirstChildElement("dofs");
        bool rotation_enabled;
        orc::ECartesianDof cartesian_dofs;
        parseFrameTaskDofs(*dofs_node, rotation_enabled, cartesian_dofs);
        orientation_task_t* taskdesc = new orientation_task_t;
        taskdesc->feature_type = "orientation";
        parseTaskInfo(task_node, *taskdesc);
        parseParam(*param_node, *taskdesc);
        orcisir::ISIRTask* task;
        
        if (rotation_enabled == true && cartesian_dofs == orc::NONE){
                //Orientation feature
                parseFeatureOrientation(*feature_node, *taskdesc);

                taskdesc_map.insert(taskdesc->id, taskdesc);
                task = &(ctrl->createISIRTask(taskdesc->id, *taskdesc->feat, *taskdesc->featdes));
                initTask(*task, *taskdesc);
                ////
        }
        else{
            std::cout << taskdesc->id << " : Invalid dof" << std::endl;
        }
    }
    else if( featType == "position" ){
        //Check which type of feature to create
        TiXmlElement const* dofs_node = feature_node->FirstChildElement("dofs");
        bool rotation_enabled;
        orc::ECartesianDof cartesian_dofs;
        parseFrameTaskDofs(*dofs_node, rotation_enabled, cartesian_dofs);
        position_task_t* taskdesc = new position_task_t;
        taskdesc->feature_type = "position";
        taskdesc->dofs = cartesian_dofs;
        parseTaskInfo(task_node, *taskdesc);
        parseParam(*param_node, *taskdesc);
        orcisir::ISIRTask* task;
        
        if (rotation_enabled == false && cartesian_dofs != orc::NONE){
            //PositionFeature
            parseFeaturePosition(*feature_node, *taskdesc);

            taskdesc_map.insert(taskdesc->id, taskdesc);
            task = &(ctrl->createISIRTask(taskdesc->id, *taskdesc->feat, *taskdesc->featdes));
            initTask(*task, *taskdesc);
        }
        else{
            std::cout << taskdesc->id << " : Invalid dof" << std::endl;
        }
    }
    else if( featType == "com" ){
        TiXmlElement const* dofs_node = feature_node->FirstChildElement("dofs");
        bool rotation_enabled;
        orc::ECartesianDof cartesian_dofs;
        parseFrameTaskDofs(*dofs_node, rotation_enabled, cartesian_dofs);
        com_task_t* taskdesc = new com_task_t;
//
        taskdesc->feature_type = "com";
        taskdesc->dofs = cartesian_dofs;
        parseTaskInfo(task_node, *taskdesc);
        parseParam(*param_node, *taskdesc);
        orcisir::ISIRTask* task;
        
        if (rotation_enabled == false && cartesian_dofs != orc::NONE){
            //com
            parseFeatureCom(*feature_node, *taskdesc);

            taskdesc_map.insert(taskdesc->id, taskdesc);
            task = &(ctrl->createISIRTask(taskdesc->id, *taskdesc->feat, *taskdesc->featdes));
            if (taskdesc->type == "ACCELERATION"){
                initTask(*task, *taskdesc);
            }
        }
        else{
            std::cout << taskdesc->id << " : Invalid dof" << std::endl;
        }

    }
    else if( featType == "contact" ){
        contact_task_t* taskdesc = new contact_task_t;
        taskdesc->feature_type = "contact";
        parseTaskInfo(task_node, *taskdesc);
        parseParam(*param_node, *taskdesc);
        parseFeatureContact(*feature_node, *taskdesc);

        taskdesc_map.insert(taskdesc->id, taskdesc);

        orcisir::ISIRTask* task;
        task = &(ctrl->createISIRContactTask(taskdesc->id, *taskdesc->feat, taskdesc->mu, taskdesc->margin));

        task->initAsAccelerationTask();
        ctrl->addTask(*task);
        task->activateAsObjective();
        task->setStiffness(taskdesc->kp);
        task->setDamping(taskdesc->kd);
        task->setWeight(taskdesc->w);

    }
    else{
        std::cout << "Unsupported " << featType << std::endl;
    }

    return true;
}

bool TaskXMLParser::updateTask(TiXmlElement const& task_node, task_t& taskdesc){
    //fullstate_task_t* t = dynamic_cast<fullstate_task_t*>(&taskdesc_map["full"]);
    //std::cout << "task id " << t->q_des[1] << std::endl;

    //task = dynamic_cast<orcisir::ISIRTask*>(&(ctrl->getTask(taskdesc.id)));
    TiXmlElement const* param_node = task_node.FirstChildElement("param");
    TiXmlElement const* feature_node = task_node.FirstChildElement("feature");
    if (feature_node == NULL){
        std::cout << "No feature!" << std::endl;
        return false;
    }

    //Check if existing task is consistent.
    std::string featType(feature_node->Attribute("type"));
    if (taskdesc.feature_type != featType){
        std::cout << taskdesc.id << " : Inconsistent feature type" << std::endl;
        return false;
    }

    if ( featType == "fullstate" ){
        fullstate_task_t* fullstate_task_desc = dynamic_cast<fullstate_task_t*>(&taskdesc);
        parseParam(*param_node, *fullstate_task_desc);
        parseObjectiveFullState(*feature_node, *fullstate_task_desc);
    }
    else if( featType == "partialstate"){
        partialstate_task_t* partialstate_task_desc = dynamic_cast<partialstate_task_t*>(&taskdesc);
        parseParam(*param_node, *partialstate_task_desc);
        parseObjectivePartialState(*feature_node, *partialstate_task_desc);
    }
    else if( featType == "displacement" ){
        displacement_task_t* displacement_task_desc = dynamic_cast<displacement_task_t*>(&taskdesc);
        parseParam(*param_node, *displacement_task_desc);
        parseObjectiveDisplacementFrame(*feature_node, *displacement_task_desc);
        
    }
    else if( featType == "orientation"){
        orientation_task_t* orientation_task_desc = dynamic_cast<orientation_task_t*>(&taskdesc);
        parseParam(*param_node, *orientation_task_desc);
        parseObjectiveOrientationFrame(*feature_node, *orientation_task_desc);

    }
    else if( featType == "position"){
        position_task_t* position_task_desc = dynamic_cast<position_task_t*>(&taskdesc);
        parseParam(*param_node, *position_task_desc);
        parseObjectivePositionFrame(*feature_node, *position_task_desc);
    }
    else if( featType == "com"){
        com_task_t* com_task_desc = dynamic_cast<com_task_t*>(&taskdesc);
        parseParam(*param_node, *com_task_desc);
        parseObjectiveCoM(*feature_node, *com_task_desc);
    }
    else if( featType == "contact" ){
        contact_task_t* contact_task_desc = dynamic_cast<contact_task_t*>(&taskdesc);
        parseParam(*param_node, *contact_task_desc);
    }

    return true;
}

void TaskXMLParser::printFullstateDesc(fullstate_task_t &task){
    std::cout << "---------\n"
        << task.id << "\n"
        << task.feature_type << "\n"
        << "w = : " << task.w << "\n"
        << "kp = : " << task.kp << "\n"
        << "kd = : " << task.kd << "\n"
        << "q_des: " ;
    
    for(unsigned int i=0; i<task.q_des.size(); ++i){
        std::cout << task.q_des[i] << " ";
    }

    std::cout << "\n---------\n";
}

void TaskXMLParser::printPartialstateDesc(partialstate_task_t &task){
    std::cout << "---------\n"
        << task.id << "\n"
        << task.feature_type << "\n"
        << "w = : " << task.w << "\n"
        << "kp = : " << task.kp << "\n"
        << "kd = : " << task.kd << "\n"
        << "sdofs = : ";

    for(unsigned int i=0; i<task.sdofs.size(); ++i){
        std::cout << task.sdofs[i] << " ";
    }

    std::cout << "\n" << "q_des: " ;
    
    for(unsigned int i=0; i<task.q_des.size(); ++i){
        std::cout << task.q_des[i] << " ";
    }

    std::cout << "\n---------\n";
}

void TaskXMLParser::printDisplacementDesc(displacement_task_t &task){
    std::cout << "---------\n"
        << task.id << "\n"
        << task.feature_type << "\n"
        << "w = : " << task.w << "\n"
        << "kp = : " << task.kp << "\n"
        << "kd = : " << task.kd << "\n";


}
