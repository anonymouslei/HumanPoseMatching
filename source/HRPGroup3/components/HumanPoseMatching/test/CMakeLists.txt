
# Libs required for the tests
SET(LIBS ${LIBS} ArmarXCore HumanPoseMatching)
 
armarx_add_test(HumanPoseMatchingTest HumanPoseMatchingTest.cpp "${LIBS}")
