#ifndef __IMAGING_SONAR_SIMULATION_HPP__
#define __IMAGING_SONAR_SIMULATION_HPP__

#include <Eigen/Core>

#include <gpu_sonar_simulation/ScanSonar.hpp>
#include <gpu_sonar_simulation/MultibeamSonar.hpp>
#include <gpu_sonar_simulation/Utils.hpp>

#include <vizkit3d_normal_depth_map/NormalDepthMap.hpp>
#include <vizkit3d_normal_depth_map/ImageViewerCaptureTool.hpp>
#include <frame_helper/FrameHelper.h>

#include <base/samples/SonarScan.hpp>

namespace imaging_sonar_localization
{

class SonarSimulation
{
public:

    SonarSimulation(double max_sensor_range, double fovX, double fovY,
        uint value, bool isHeight, osg::ref_ptr<osg::Group> root);
    ~SonarSimulation();
    base::samples::Sonar simulateSonarData(const Eigen::Affine3d& sonar_pose);
    void updateSimulatedSonarPose(const Eigen::Affine3d pose); 
    

    gpu_sonar_simulation::MultibeamSonar sonar;
    vizkit3d_normal_depth_map::NormalDepthMap normal_depth_map;
    vizkit3d_normal_depth_map::ImageViewerCaptureTool capture_tool;


};

}

#endif
