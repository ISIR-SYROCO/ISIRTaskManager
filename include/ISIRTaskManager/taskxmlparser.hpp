// Filename:  taskxmlparser.hpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  Parse XML file and create ISIRTask objects

#include <tinyxml.h>
#include <string>

#include <orcisir/ISIRController.h>

class TaskXMLParser{
    public:
        TaskXMLParser();
        ~TaskXMLParser();
        TaskXMLParser(std::string filepath, orcisir::ISIRController& controller);

        bool loadFile(std::string filepath);
        orcisir::ISIRTask getTask(std::string taskName);

    private:
        bool parse();
        bool addTask(TiXmlElement const& tasknode);

    public:
        TiXmlDocument taskfile;
        orcisir::ISIRController* ctrl;

};


