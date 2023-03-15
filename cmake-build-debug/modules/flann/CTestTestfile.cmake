# CMake generated Testfile for 
# Source directory: /home/jeffery/Desktop/Sustech/GroupProject/code/opencv/modules/flann
# Build directory: /home/jeffery/Desktop/Sustech/GroupProject/code/opencv/cmake-build-debug/modules/flann
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(opencv_test_flann "/home/jeffery/Desktop/Sustech/GroupProject/code/opencv/cmake-build-debug/bin/opencv_test_flann" "--gtest_output=xml:opencv_test_flann.xml")
set_tests_properties(opencv_test_flann PROPERTIES  LABELS "Main;opencv_flann;Accuracy" WORKING_DIRECTORY "/home/jeffery/Desktop/Sustech/GroupProject/code/opencv/cmake-build-debug/test-reports/accuracy" _BACKTRACE_TRIPLES "/home/jeffery/Desktop/Sustech/GroupProject/code/opencv/cmake/OpenCVUtils.cmake;1752;add_test;/home/jeffery/Desktop/Sustech/GroupProject/code/opencv/cmake/OpenCVModule.cmake;1352;ocv_add_test_from_target;/home/jeffery/Desktop/Sustech/GroupProject/code/opencv/cmake/OpenCVModule.cmake;1110;ocv_add_accuracy_tests;/home/jeffery/Desktop/Sustech/GroupProject/code/opencv/modules/flann/CMakeLists.txt;2;ocv_define_module;/home/jeffery/Desktop/Sustech/GroupProject/code/opencv/modules/flann/CMakeLists.txt;0;")
