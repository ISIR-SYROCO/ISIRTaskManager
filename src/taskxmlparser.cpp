// Filename:  taskxmlparser.cpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:

#include "ISIRTaskManager/taskxmlparser.hpp"
#include <boost/filesystem.hpp>
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
        addTask(*task);
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

    std::cout << taskdesc.id << ":" << taskdesc.type << "\n\t" 
                         << " w = " << taskdesc.w << "\n\t"
                         << " kp = " << taskdesc.kp << "\n\t"
                         << " kd = " << taskdesc.kd << "\n";
    return true;
}

bool TaskXMLParser::parseFeature(TiXmlElement const& feature_node, fullstate_task_t& taskdesc){
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

    TiXmlElement const* qdes_node = feature_node.FirstChildElement("objective")->FirstChildElement("q_des");
    Eigen::VectorXd q_des(ctrl->getModel().nbInternalDofs());
    if (qdes_node->Attribute("value") != NULL){
        std::istringstream q_des_ss(qdes_node->Attribute("value"));
        for (unsigned int i=0; i<q_des.size(); i++){
            q_des_ss >> q_des[i];
        }
    }
    taskdesc.FTS->set_q(q_des);

    taskdesc.feat = new orc::FullStateFeature (taskdesc.id+".feat", *(taskdesc.FMS));
    taskdesc.featdes = new orc::FullStateFeature (taskdesc.id+"featdes", *(taskdesc.FTS));

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
        fullstate_task_t taskdesc;
        parseTaskInfo(task_node, taskdesc);
        parseParam(*param_node, taskdesc);
        parseFeature(*feature_node, taskdesc);
        orcisir::ISIRTask& task = ctrl->createISIRTask(taskdesc.id, *taskdesc.feat, *taskdesc.featdes);

        if (taskdesc.type == "ACCELERATION"){
            task.initAsAccelerationTask();
            ctrl->addTask(task);
            task.activateAsObjective();
            task.setStiffness(taskdesc.kp);
            task.setDamping(taskdesc.kd);
            task.setWeight(taskdesc.w);
        }
        else{
            std::cout << taskdesc.id << " : Unsupported " << taskdesc.type << std::endl;
            return false;
        }
    }
    else{
        std::cout << "Unsupported " << featType << std::endl;
    }

    return true;
}
