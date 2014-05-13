# Copyright 2014 ISIR-CNRS
# Authors: Sovannara HAK

ADD_LIBRARY(
    ${PROJECT_NAME}
    ${LIB_TYPE}
    taskxmlparser.cpp
    mathutils.cpp
)

TARGET_LINK_LIBRARIES(
    ${PROJECT_NAME}
    ${TINYXML_LIBRARIES}
    #${ORC_FRAMEWORK_LIBRARIES}
    ${ISIRController_LIBRARIES}
    ${Boost_LIBRARIES}
)

INSTALL(
    TARGETS ${PROJECT_NAME}
    ${install_type} DESTINATION lib
)
