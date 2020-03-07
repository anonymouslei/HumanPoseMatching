armarx_component_set_name("HumanPoseMatchingApp")
set(COMPONENT_LIBS HumanPoseMatching)
armarx_add_component_executable(main.cpp)

#find_package(MyLib QUIET)
#armarx_build_if(MyLib_FOUND "MyLib not available")
# all target_include_directories must be guarded by if(Xyz_FOUND)
# for multiple libraries write: if(X_FOUND AND Y_FOUND)....
#if(MyLib_FOUND)
#    target_include_directories(HumanPoseMatching PUBLIC ${MyLib_INCLUDE_DIRS})
#endif()
