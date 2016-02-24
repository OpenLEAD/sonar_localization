rock_library(imaging_sonar_localization
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(imaging_sonar_localization_bin Main.cpp
    DEPS imaging_sonar_localization)

