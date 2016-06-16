rock_testsuite(test_suite
   tests.cpp
   HEADERS
   LIBS ${Boost_SYSTEM_LIBRARY}
   DEPS_CMAKE GTest
   DEPS_PKGCONFIG base-lib vizkit3d_normal_depth_map vizkit3d_world
   DEPS imaging_sonar_localization) 
