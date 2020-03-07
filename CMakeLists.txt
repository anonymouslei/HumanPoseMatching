# HRPGroup3

cmake_minimum_required(VERSION 3.10)

find_package("ArmarXCore" REQUIRED)

# Include provides all necessary ArmarX CMake macros
include(${ArmarXCore_USE_FILE})

set(ARMARX_ENABLE_DEPENDENCY_VERSION_CHECK_DEFAULT FALSE)
set(ARMARX_ENABLE_AUTO_CODE_FORMATTING TRUE)

# Name for the project
armarx_project("HRPGroup3")

# Specify each ArmarX Package dependency with the following macro
depends_on_armarx_package(ArmarXGui "OPTIONAL")
depends_on_armarx_package(RobotAPI)

set(PROJECT_SOURCE_DIR ${CMAKE_SOURCE_DIR})
add_definitions(-DPROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}" -D_SCL_SECURE_NO_WARNINGS)

add_subdirectory(etc)
add_subdirectory(source)

install_project()
