rock_library(imaging_sonar_localization
    SOURCES PoseEstimator.cpp
    HEADERS PoseEstimator.hpp
            PoseParticle.hpp
            Configuration.hpp
    DEPS_PKGCONFIG base-types odometry eslam gpu_sonar_simulation) 


