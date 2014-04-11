// Filename:  taskxmlparser.cpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:

#include "ISIRTaskManager/taskxmlparser.hpp"
#include <boost/filesystem.hpp>
#include <iostream>

TaskXMLParser::TaskXMLParser(){

}

TaskXMLParser::~TaskXMLParser(){

}

TaskXMLParser::TaskXMLParser(std::string filepath){
    std::cout << filepath << std::endl;
}

bool TaskXMLParser::loadFile(std::string filepath){
    return true;
}
