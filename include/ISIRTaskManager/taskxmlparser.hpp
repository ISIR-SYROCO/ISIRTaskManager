// Filename:  taskxmlparser.hpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  Parse XML file and create ISIRTask objects

#include <tinyxml.h>
#include <string>
#include <map>

#include <orcisir/ISIRController.h>
#include <orc/control/Feature.h>
#include <orc/control/FullState.h>
#include <boost/ptr_container/ptr_map.hpp>

struct task_t{
    virtual ~task_t() {}
    double w;
    double kp;
    double kd;
    std::string type;
    std::string feature_type;
    std::string id;
};

struct fullstate_task_t : task_t{
    int active;
    std::string whatPart;
    orc::FullModelState* FMS;
    orc::FullTargetState* FTS;
    orc::FullStateFeature* feat;
    orc::FullStateFeature* featdes;
    Eigen::VectorXd q_des;
    Eigen::VectorXd qd_des;
    Eigen::VectorXd qdd_des;
    Eigen::VectorXd tau_des;
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
        bool updateTask(TiXmlElement const& tasknode);

        bool parseTaskInfo(TiXmlElement const& task_node, task_t& taskdesc);
        bool parseFeatureFullState(TiXmlElement const& feature_node, fullstate_task_t& taskdesc);
        bool parseParam(TiXmlElement const& param_node, task_t& taskdesc);

        boost::ptr_map< std::string, task_t > taskdesc_map;

    public:
        TiXmlDocument taskfile;
        orcisir::ISIRController* ctrl;

};


