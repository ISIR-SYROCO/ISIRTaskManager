// Filename:  taskxmlparser.hpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  Parse XML file and create ISIRTask objects

#include <tinyxml.h>
#include <string>
#include <map>

#include <orcisir/ISIRController.h>
#include <orcisir/Features/ISIRFeature.h>
#include <orc/control/Feature.h>
#include <orc/control/FullState.h>
#include <boost/ptr_container/ptr_map.hpp>

struct task_t{
    virtual ~task_t() {}
    double w;
    double kp;
    double kd;
    int active;
    std::string type;
    std::string feature_type;
    std::string id;
};

struct fullstate_task_t : task_t{
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

struct partialstate_task_t : task_t{
    std::string whatPart;
    orcisir::PartialModelState* PMS;
    orcisir::PartialTargetState* PTS;
    orcisir::PartialStateFeature* feat;
    orcisir::PartialStateFeature* featdes;
    Eigen::VectorXi sdofs;
    Eigen::VectorXd q_des;
    Eigen::VectorXd qd_des;
    Eigen::VectorXd qdd_des;
    Eigen::VectorXd tau_des;
};

struct frame_task_t : task_t{
    orc::SegmentFrame* SF;
    orc::TargetFrame* TF;
    std::string segment_name;
    Eigen::Displacementd offset;
    Eigen::Displacementd position_des;
    Eigen::Twistd velocity_des;
    Eigen::Twistd acceleration_des;
    Eigen::Wrenchd wrench_des;
};

struct displacement_task_t : frame_task_t{
    orc::ECartesianDof dofs;
    orc::DisplacementFeature* feat;
    orc::DisplacementFeature* featdes;
};

struct position_task_t : frame_task_t{
    orc::ECartesianDof dofs;
    orc::PositionFeature* feat;
    orc::PositionFeature* featdes;
};

struct orientation_task_t : frame_task_t{
    orc::OrientationFeature* feat;
    orc::OrientationFeature* featdes;
};

struct com_task_t : task_t{
    orc::CoMFrame* CoMF;
    orc::TargetFrame* TF;
    orc::ECartesianDof dofs;
    Eigen::Displacementd position_des;
    Eigen::Twistd velocity_des;
    Eigen::Twistd acceleration_des;
    orc::PositionFeature* feat;
    orc::PositionFeature* featdes;
};

struct contact_task_t : task_t{
    orc::SegmentFrame* SF;
    orc::PointContactFeature* feat;
    Eigen::Displacementd offset;
    std::string segment_name;
    double mu;
    double margin;

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
        bool updateTask(TiXmlElement const& tasknode, task_t& taskdesc);

        bool parseTaskInfo(TiXmlElement const& task_node, task_t& taskdesc);

        bool fillVector(TiXmlElement const* node, unsigned int vector_size, Eigen::VectorXd& vector_result);
        bool fillVector3(TiXmlElement const* node, const char * attribute, Eigen::Vector3d& vector_result);

        bool parseFeatureFullState(TiXmlElement const& feature_node, fullstate_task_t& taskdesc);
        bool parseObjectiveFullState(TiXmlElement const& feature_node, fullstate_task_t& taskdesc);

        bool parseFeaturePartialState(TiXmlElement const& feature_node, partialstate_task_t& taskdesc);
        bool parseObjectivePartialState(TiXmlElement const& feature_node, partialstate_task_t& taskdesc);

        bool parseObjectiveDisplacementFrame(TiXmlElement const& feature_node, frame_task_t& taskdesc);
        bool parseObjectiveOrientationFrame(TiXmlElement const& feature_node, frame_task_t& taskdesc);
        bool parseObjectivePositionFrame(TiXmlElement const& feature_node, frame_task_t& taskdesc);
        bool parseObjectiveCoM(TiXmlElement const& feature_node, com_task_t& taskdesc);

        bool parseFeatureDisplacement(TiXmlElement const& feature_node, displacement_task_t& taskdesc);
        
        bool parseFeatureOrientation(TiXmlElement const& feature_node, orientation_task_t& taskdesc);

        bool parseFeaturePosition(TiXmlElement const& feature_node, position_task_t& taskdesc);
        bool parseFeatureCom(TiXmlElement const& feature_node, com_task_t& taskdesc);
        bool parseFeatureContact(TiXmlElement const& feature_node, contact_task_t& taskdesc);

        bool parseParam(TiXmlElement const& param_node, task_t& taskdesc);

        void parseFrameTaskDofs(TiXmlElement const& dof_node, bool& rotation, orc::ECartesianDof& cartesian_dof);

        template<class T>
        void parseLocalOffset(TiXmlElement const& local_offset_node, T& taskdesc);

        void initTask(orcisir::ISIRTask& task, task_t& taskdesc);

        boost::ptr_map< std::string, task_t > taskdesc_map;

        std::map<std::string, orc::ECartesianDof> str_frame_dof_map;

        void printFullstateDesc(fullstate_task_t &task);
        void printPartialstateDesc(partialstate_task_t &task);
        void printDisplacementDesc(displacement_task_t &task);

    public:
        TiXmlDocument taskfile;
        orcisir::ISIRController* ctrl;

};


