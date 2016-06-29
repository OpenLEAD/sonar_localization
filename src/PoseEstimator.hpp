#ifndef __IMAGING_SONAR_POSE_ESTIMATOR_HPP__
#define __IMAGING_SONAR_POSE_ESTIMATOR_HPP__

#include <vector>
#include <boost/random/normal_distribution.hpp>
#include <boost/intrusive_ptr.hpp>
#include <Eigen/Core>

#include "PoseParticle.hpp"
#include "Configuration.hpp"
#include "ParticleFilter.hpp"
#include "SonarSimulation.hpp"

#include <base/Pose.hpp>
#include <base/samples/SonarScan.hpp>
#include <base/TwistWithCovariance.hpp>

namespace imaging_sonar_localization
{

struct Statistics
{
    double m_x;
    double m_y; 
    double m_wc;
    double m_yaw;
    double s_x; 
    double s_y; 
    double s_wc; 
    double s_yaw;
};


class PoseEstimator:
   public ParticleFilter<PoseParticle> 
{
public:
    PoseEstimator(const Configuration& config);
    ~PoseEstimator();
    
    void init(int num_particles, SonarSimulation* sonar_sim, const base::Vector2d& mu_pose, const base::Vector2d& sigma_pose, 
                double  mu_orientation, double  sigma_orientation, double mu_wc, double sigma_wc);
    void project(const base::TwistWithCovariance& input_velocities, double dt);
    void update(const base::samples::Sonar& sonar_data, const Eigen::Affine3d& sonar_tf,
            const Eigen::Affine3d& abs_data);

    base::Pose getCentroid();
    Statistics getStatistics();
 
private:
    double compareSonarData(const base::samples::Sonar& real, const base::samples::Sonar& sim);

    void updateWeights(const base::samples::Sonar& sonar_data, const Eigen::Affine3d& sonar_tf, 
            const Eigen::Affine3d& inertial_data);
 
    base::Vector2d samplePose(const base::Vector2d& mu, const base::Vector2d& sigma);    
    double sampleValue(double mu, double sigma);
    double max_weight;

    boost::variate_generator<boost::minstd_rand&, boost::normal_distribution<>  > rand_norm;
    boost::variate_generator<boost::minstd_rand&, boost::uniform_real<> >       rand_uni;
    Configuration config;

    SonarSimulation* sonar_sim;

};

}

#endif
