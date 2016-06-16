#ifndef __IMAGING_SONAR_POSE_ESTIMATOR_HPP__
#define __IMAGING_SONAR_POSE_ESTIMATOR_HPP__

#include <boost/random/normal_distribution.hpp>
#include <boost/intrusive_ptr.hpp>
#include <vector>
#include "ParticleFilter.hpp"
#include <base/Pose.hpp>
#include <base/samples/SonarScan.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <Eigen/Core>
#include "PoseParticle.hpp"
#include "Configuration.hpp"
#include <gpu_sonar_simulation/ScanSonar.hpp>
#include <gpu_sonar_simulation/MultibeamSonar.hpp>
#include <gpu_sonar_simulation/Utils.hpp>
#include <vizkit3d_normal_depth_map/NormalDepthMap.hpp>
#include <vizkit3d_normal_depth_map/ImageViewerCaptureTool.hpp>
#include <frame_helper/FrameHelper.h>

namespace imaging_sonar_localization
{

class PoseEstimator:
   public ParticleFilter<PoseParticle> 
{
public:
    PoseEstimator(const Configuration& config,
            gpu_sonar_simulation::MultibeamSonar& msonar,
            vizkit3d_normal_depth_map::NormalDepthMap& n_depth_map,
            vizkit3d_normal_depth_map::ImageViewerCaptureTool& capture);
    ~PoseEstimator();
    
    void init(int num_particles, const base::Vector2d& mu_pose, const base::Vector2d& sigma_pose, 
                double  mu_orientation, double  sigma_orientation, double mu_wc, double sigma_wc);
    void project(double u, double v, double r, double dt);
    void update(const base::samples::Sonar& sonar_data, Eigen::Affine3d& sonar_tf);
 
private:
    void updateSimulatedSonarPose(base::samples::RigidBodyState pose); 
    double compareSonarData(const base::samples::Sonar& real, const base::samples::Sonar& sim);

    void updateWeights(const base::samples::Sonar& sonar_data, Eigen::Affine3d& sonar_tf, 
            base::samples:: RigidBodyState& inertial_data);
 
    boost::variate_generator<boost::minstd_rand&, boost::normal_distribution<>  > rand_norm;
    boost::variate_generator<boost::minstd_rand&, boost::uniform_real<> >       rand_uni;

    base::Vector2d samplePose(const base::Vector2d& mu, const base::Vector2d& sigma);    
    double sampleValue(double mu, double sigma);
    double max_weight;

    //gpu_sonar_simulation::ScanSonar _ssonar;
    gpu_sonar_simulation::MultibeamSonar _msonar;
    vizkit3d_normal_depth_map::NormalDepthMap _normal_depth_map;
    vizkit3d_normal_depth_map::ImageViewerCaptureTool _capture;
    Configuration config;
    bool msis = true; //switch beetween scanning and multibeam
};

}

#endif
