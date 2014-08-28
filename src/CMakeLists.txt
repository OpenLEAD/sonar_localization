set(FILTER_HDRS
    ParticleFilter.hpp
    PoseEstimator.hpp
    PoseParticle.hpp
    UnderwaterVehicleState.hpp
    UnderwaterVehicleOdometry.hpp    
    )
    
set(FILTER_SCRS
    PoseEstimator.cpp
    UnderwaterVehicleOdometry.cpp    
    )    


rock_library(sonar_localization
    SOURCES ${FILTER_SCRS}
    HEADERS ${FILTER_HDRS}
    DEPS_PKGCONFIG base-types odometry)

rock_executable(sonar_localization_bin Main.cpp
    DEPS sonar_localization)

