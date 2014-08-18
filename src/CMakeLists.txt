rock_library(sonar_localisation
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(sonar_localisation_bin Main.cpp
    DEPS sonar_localisation)

