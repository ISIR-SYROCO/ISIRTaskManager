# Copyright 2014 ISIR-CNRS
# Authors: Sovannara HAK

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(ISIRTaskManager)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

SET(CMAKE_CXX_FLAGS "-fpermissive")

IF(MSVC)
    SET(LIB_TYPE "STATIC")
    SET(suffix_type "_static")
    SET(install_type "ARCHIVE")
ELSE()
    SET(LIB_TYPE "SHARED")
    SET(suffix_type "")
    IF(UNIX)
        SET(install_type "LIBRARY")
    ELSE()
        SET(install_type "RUNTIME")
    ENDIF()
ENDIF()

IF (UNIX)
    ADD_DEFINITIONS(-DUNIX)
ENDIF (UNIX)

FIND_PACKAGE(PkgConfig)

PKG_CHECK_MODULES(TINYXML tinyxml)
IF(NOT TINYXML_FOUND)
    MESSAGE(FATAL_ERROR "tinyxml not found")
ENDIF()

PKG_CHECK_MODULES(ISIRController orcisir_ISIRController)
IF(NOT ISIRController_FOUND)
    MESSAGE(FATAL_ERROR "ISIRController not found")
ENDIF()

#PKG_CHECK_MODULES(ORC_FRAMEWORK orc_framework)
#IF(NOT ORC_FRAMEWORK_FOUND)
#    MESSAGE(FATAL_ERROR "orc_framework not found")
#ENDIF()

FIND_PACKAGE(Boost 1.46.1 REQUIRED)

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(include)
ADD_SUBDIRECTORY(pkg-config)
