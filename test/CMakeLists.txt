# Copyright 2014 ISIR-CNRS
# Authors: Sovannara HAK

ADD_EXECUTABLE(testxmlparser
    testxmlparser.cpp
)

TARGET_LINK_LIBRARIES(
    testxmlparser
    ${PROJECT_NAME}
)
