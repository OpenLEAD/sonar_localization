#ifndef __IMAGING_SONAR_POSE_ESTIMATOR_HPP__
#define __IMAGING_SONAR_POSE_ESTIMATOR_HPP__

#include <boost/random/normal_distribution.hpp>
#include <boost/intrusive_ptr.hpp>
#include <vector>
#include <eslam/ParticleFilter.hpp>
#include <base/Pose.hpp>
#include <base/samples/SonarScan.hpp>
#include <Eigen/Core>
#include "PoseParticle.hpp"



namespace imaging_sonar_localization
{

class PoseEstimator:
   public eslam::ParticleFilter<PoseParticle> 
{
public:
    PoseEstimator();
    ~PoseEstimator();

    void init(int num_particles, const base::Vector3d& mu_pose, const base::Vector3d& sigma_pose, 
                const base::Vector3d& mu_orientation, const base::Vector3d& sigma_orientationw);
    void project();
    void update(const base::samples::SonarBeam& beam, Eigen::Affine3d& sonar_tf);
    void update(const base::samples::SonarScan& scan, Eigen::Affine3d& sonar_tf);
 
private:
    void updateWeights(const base::samples::SonarBeam& beam, Eigen::Affine3d& sonar_tf);
    void updateWeights(const base::samples::SonarScan& scan, Eigen::Affine3d& sonar_tf);
 
    boost::variate_generator<boost::minstd_rand&, boost::normal_distribution<>  > rand_norm;
    boost::variate_generator<boost::minstd_rand&, boost::uniform_real<> >       rand_uni;

    base::Vector3d samplePose(const base::Vector3d& mu, const base::Vector3d& sigma);    
    Eigen::Matrix3f sampleOrientation(const base::Vector3d& mu, const base::Vector3d& sigma);
    double max_weight;
};

}

#endif
