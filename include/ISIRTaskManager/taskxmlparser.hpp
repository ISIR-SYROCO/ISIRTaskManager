// Filename:  taskxmlparser.hpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  Parse XML file and create ISIRTask objects

#include <tinyxml.h>
#include <string>
#include <orcisir/Tasks/ISIRTask.h>

class TaskXMLParser{
    public:
        TaskXMLParser();
        ~TaskXMLParser();
        TaskXMLParser(std::string filepath);

        bool loadFile(std::string filepath);
        orcisir::ISIRTask getTask();

};


