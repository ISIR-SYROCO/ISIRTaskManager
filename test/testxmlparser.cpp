// Filename:  testxmlparser.cpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  


#include "ISIRTaskManager/taskxmlparser.hpp"
#include <iostream>

int main(int argc, char** argv){
    if (argc < 2){
        std::cout << "Usage: testxmlparser path\n";
        return 1;
    }

    std::string path(argv[1]);
    TaskXMLParser xmlparser(path);
    return 0;
}


