# Copyright 2014 ISIR-CNRS
# Authors: Sovannara HAK

ADD_LIBRARY(
    ModelKukaFixed
    SHARED
    kukafixed.cpp
)

ADD_EXECUTABLE(testxmlparser
    testxmlparser.cpp
)

TARGET_LINK_LIBRARIES(
    testxmlparser
    ModelKukaFixed
    ${PROJECT_NAME}
)

ADD_EXECUTABLE(testquaternion
    testquaternion.cpp
)

TARGET_LINK_LIBRARIES(
    testquaternion
    ${PROJECT_NAME}
)
