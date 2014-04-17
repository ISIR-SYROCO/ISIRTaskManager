// Filename:  taskxmlparser.hpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  Parse XML file and create ISIRTask objects

#include <tinyxml.h>
#include <string>

#include <orcisir/ISIRController.h>
#include <orc/control/Feature.h>
#include <orc/control/FullState.h>

struct task_t{
    double w;
    double kp;
    double kd;
    std::string type;
    std::string id;
};

struct fullstate_task_t : task_t{
    std::string whatPart;
    orc::FullModelState* FMS;
    orc::FullTargetState* FTS;
    orc::FullStateFeature* feat;
    orc::FullStateFeature* featdes;
};

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

        bool parseTaskInfo(TiXmlElement const& task_node, task_t& taskdesc);
        bool parseFeature(TiXmlElement const& feature_node, fullstate_task_t& taskdesc);
        bool parseParam(TiXmlElement const& param_node, task_t& taskdesc);

    public:
        TiXmlDocument taskfile;
        orcisir::ISIRController* ctrl;

};


