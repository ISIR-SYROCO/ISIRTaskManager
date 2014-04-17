// Filename:  testxmlparser.cpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  


#include "ISIRTaskManager/taskxmlparser.hpp"
#include <iostream>
#include <orcisir/Solvers/OneLevelSolver.h>
#include <orcisir/ISIRController.h>
#include <Eigen/Lgsm>
#include "kukafixed.h"

int main(int argc, char** argv){
    if (argc < 2){
        std::cout << "Usage: testxmlparser path\n";
        return 1;
    }

    bool useReducedProblem = true;

    kukafixed model("kuka");

    orcisir::OneLevelSolverWithQuadProg solver;

    orcisir::ISIRController ctrl("myCtrl", model, solver, useReducedProblem);

    ctrl.takeIntoAccountGravity(false);

    std::string path(argv[1]);

    TaskXMLParser xmlparser(path, ctrl);

    VectorXd q      = Eigen::VectorXd::Constant(model.nbInternalDofs(), 0.1);
    VectorXd dq     = Eigen::VectorXd::Zero(model.nbInternalDofs());
    VectorXd tau    = Eigen::VectorXd::Zero(model.nbInternalDofs());
    double dt = 0.01;

    model.setJointPositions(q);
    model.setJointVelocities(dq);


    //SIMULATE
    std::cout<<"SIMULATE\n";
    for (int i=0; i<1000; i++)
    {
        std::cout<<"- -  --- - - - -- - - -- - "<<i<<"\n";

        model.setJointPositions(q);
        model.setJointVelocities(dq);

        ctrl.computeOutput(tau);    //compute tau
        //VectorXd ddq = model.getAccelerationVariable().getValue();
        //VectorXd ddq = model.getInertiaMatrixInverse() * ( tau - model.getNonLinearTerms() - model.getLinearTerms() - model.getGravityTerms() );
        VectorXd ddq = model.getInertiaMatrixInverse() * ( tau - model.getNonLinearTerms() - model.getLinearTerms());

        dq += ddq * dt;
        q  += dq  * dt;

        std::cout<<"tau: "<<tau.transpose()<<"\n";
        std::cout<<"ddq: "<<ddq.transpose()<<"\n";
        std::cout<<"q  : "<<model.getJointPositions().transpose()<<"\n";
        std::cout<<"pos EE: "<< model.getSegmentPosition(7).getTranslation().transpose()<<"\n";
        std::cout<<"ori EE: "<< model.getSegmentPosition(7).getRotation()<<"\n";

        //std::cout<<solver.toString()<<"\n";
    } 

    return 0;
}


