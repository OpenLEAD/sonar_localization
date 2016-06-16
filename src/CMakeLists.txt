rock_library(imaging_sonar_localization
    SOURCES PoseEstimator.cpp
    HEADERS PoseEstimator.hpp
            PoseParticle.hpp
            Configuration.hpp
    DEPS_PKGCONFIG base-types gpu_sonar_simulation vizkit3d_normal_depth_map) 


