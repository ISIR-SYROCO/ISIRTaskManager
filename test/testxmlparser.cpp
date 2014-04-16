// Filename:  testxmlparser.cpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  


#include "ISIRTaskManager/taskxmlparser.hpp"
#include <iostream>
#include <orcisir/Solvers/OneLevelSolver.h>
#include <orcisir/ISIRController.h>
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
    return 0;
}


