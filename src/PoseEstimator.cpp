#include "PoseEstimator.hpp"

using namespace imaging_sonar_localization;



PoseEstimator::PoseEstimator():
    ParticleFilter<PoseParticle>(10),
    rand_norm(rand_gen, boost::normal_distribution<>(0,1.0) ),
    rand_uni(rand_gen, boost::uniform_real<>(0,1.0) ),
    max_weight(0)
{
}

PoseEstimator::~PoseEstimator()
{
}


base::Vector3d PoseEstimator::samplePose(const base::Vector3d& mu, const base::Vector3d& sigma)
{

    double x = rand_norm(), y= rand_norm(), z = rand_norm();
    return base::Vector3d(x*sigma[0] + mu[0],
                          y*sigma[1] +mu[0],
                          z*sigma[2] +mu[2]);

}

Eigen::Matrix3f PoseEstimator::sampleOrientation(const base::Vector3d& mu, const base::Vector3d& sigma)
{  
    double roll = rand_norm(), pitch= rand_norm(),  yaw= rand_norm();
    Eigen::Matrix3f orientation;
    orientation = Eigen::AngleAxisf(roll*sigma[0] + mu[0], Eigen::Vector3f::UnitZ())
                                  * Eigen::AngleAxisf(pitch*sigma[1] + mu[1], Eigen::Vector3f::UnitY())
                                   * Eigen::AngleAxisf(yaw*sigma[2]+mu[2], Eigen::Vector3f::UnitZ());
 
    return orientation;
}



void PoseEstimator::init(int num_particles, const base::Vector3d& mu_pose, const base::Vector3d& sigma_pose, const base::Vector3d& mu_orientation, const base::Vector3d& sigma_orientation)
{
    for (int i=0; i<num_particles; i++)
    {
        base::Vector3d pose_sampled =  samplePose(mu_pose,sigma_pose);
        Eigen::Matrix3f orientation_sampled = sampleOrientation(mu_orientation, sigma_orientation);
        
        xi_k.push_back(Particle(pose_sampled, orientation_sampled));
    }
}

//TODO verify what is the spread function    
/**
 * this is a piecewise linear function, with 
 * @param x as the function input
 * @param alpha minimum threshold any value of x below this value will be 1.0
 * @param beta any x above beta will be set to gamma
 * @param gamma result if x is above beta
 *
 * the interval between alpha and beta is linear, so that the function is
 * continous.
 */
double weightingFunction( double x, double alpha = 0.1, double beta = 0.9, double gamma = 0.05 )
{
    if( x < alpha )
	return 1.0;
    if( x < beta )
    {
	double a = (1.0-gamma)/(alpha-beta);
	double b = 1.0-alpha*a;
	return a*x + b; 
    }
    if( x >= beta )
	return gamma;

    return 0.0;
}



void PoseEstimator::project()
{
    double spread = 0; //weightingFunction( max_weight, 0.0, config.spreadThreshold, 0.0 ); 
 
    //Eigen::Affine3d odometry_pose = odometry.getPoseSample.toTransform();
    
    for(size_t i=0; xi_k.size();i++)
    {
        Particle &p(xi_k[i]);
        
        // update the position of the particle from the reading from the
        // odometry
        //p.odometry_position += odometry_pose.translation();
        
        //update the orientation the same way
        //p.orientation = TODO rotate the orientation
    
        if(spread>0)
        {
        // TODO spread the particle to see if the PF recovers itself
        // spread position and orientation? 
        }
    
    }

}


void PoseEstimator::updateWeights(const base::samples::SonarBeam& beam, Eigen::Affine3d& sonar_tf)
{
    //TODO Implement the weight function for the MSIS
    return;

}

void PoseEstimator::updateWeights(const base::samples::SonarScan& scan, Eigen::Affine3d& sonar_tf)
{
    //TODO Implement the weight function for the Foward looking Imaging
    return;

}


void PoseEstimator::update(const base::samples::SonarBeam& beam, Eigen::Affine3d& sonar_tf)
{
    updateWeights(beam,sonar_tf);
    double eff = normalizeWeights();
    if( eff < 10) //TODO config.minEffective )
    {
	resample();
    }
}

void PoseEstimator::update(const base::samples::SonarScan& scan, Eigen::Affine3d& sonar_tf)
{
    updateWeights(scan,sonar_tf);
    double eff = normalizeWeights();
    if( eff < 10) //TODO config.minEffective )
    {
	resample();
    }
}
