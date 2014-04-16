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

bool TaskXMLParser::addTask(TiXmlElement const& tasknode){
    std::cout << tasknode.Attribute("id") << std::endl;
    return true;
}
