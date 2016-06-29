rock_library(imaging_sonar_localization
    SOURCES PoseEstimator.cpp
            SonarSimulation.cpp
    HEADERS PoseEstimator.hpp
            PoseParticle.hpp
            Configuration.hpp
            ParticleFilter.hpp
            SonarSimulation.hpp
    DEPS_PKGCONFIG base-types gpu_sonar_simulation vizkit3d_normal_depth_map) 


