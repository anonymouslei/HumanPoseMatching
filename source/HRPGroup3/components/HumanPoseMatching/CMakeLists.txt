armarx_component_set_name("HumanPoseMatching")

find_package(MMMCore REQUIRED)
find_package(MMMTools REQUIRED)


set(COMPONENT_LIBS
    ArmarXCore
    ArmarXCoreInterfaces  # for DebugObserverInterface
    # RobotAPICore  # for DebugDrawerTopic
    RobotAPIComponentPlugins
    ArmarXGuiComponentPlugins
    MMMCore
    MMMSimoxTools
)

set(SOURCES
    ./HumanPoseMatching.cpp
#@TEMPLATE_LINE@@COMPONENT_PATH@/@COMPONENT_NAME@.cpp
)
set(HEADERS
    ./HumanPoseMatching.h
#@TEMPLATE_LINE@@COMPONENT_PATH@/@COMPONENT_NAME@.h
)


armarx_add_component("${SOURCES}" "${HEADERS}")

if(NOT TARGET OpenMP::OpenMP_CXX)
    find_package(Threads REQUIRED)
    add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
    set_property(TARGET OpenMP::OpenMP_CXX
                 PROPERTY INTERFACE_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
    # Only works if the same flag is passed to the linker; use CMake 3.9+ otherwise (Intel, AppleClang)
    set_property(TARGET OpenMP::OpenMP_CXX
                 PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)

endif()
target_link_libraries(HumanPoseMatching PUBLIC OpenMP::OpenMP_CXX)

#find_package(MyLib QUIET)
#armarx_build_if(MyLib_FOUND "MyLib not available")
# all target_include_directories must be guarded by if(Xyz_FOUND)
# for multiple libraries write: if(X_FOUND AND Y_FOUND)....
#if(MyLib_FOUND)
#    target_include_directories(HumanPoseMatching PUBLIC ${MyLib_INCLUDE_DIRS})
#endif()

target_include_directories(HumanPoseMatching PUBLIC ${MMMCore_INCLUDE_DIRS})
target_include_directories(HumanPoseMatching PUBLIC ${MMMTools_INCLUDE_DIRS})

# add unit tests
#add_subdirectory(test)
