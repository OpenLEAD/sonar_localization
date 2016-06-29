#include "PoseEstimator.hpp"
#include <math.h>
using namespace imaging_sonar_localization;



PoseEstimator::PoseEstimator(const Configuration& config):
    ParticleFilter<PoseParticle>(config.seed),
    rand_norm(rand_gen, boost::normal_distribution<>(0,1.0) ),
    rand_uni(rand_gen, boost::uniform_real<>(0,1.0) ),
    max_weight(0),
    config(config),
    sonar_sim(NULL)

{
}

PoseEstimator::~PoseEstimator()
{
    delete sonar_sim;
}


base::Vector2d PoseEstimator::samplePose(const base::Vector2d& mu, const base::Vector2d& sigma)
{

    double x = rand_norm(), y= rand_norm();
    base::Vector2d pose = base::Vector2d(x*sigma[0] + mu[0],
                          y*sigma[1] + mu[1]);

}

double PoseEstimator::sampleValue(double  mu, double sigma)
{  
    double rand = rand_norm();
    double value = rand*sigma + mu;
    return value;
}



void PoseEstimator::init(int num_particles, SonarSimulation* sonar_sim, const base::Vector2d& mu_pose,
        const base::Vector2d& sigma_pose, double  mu_orientation, 
        double sigma_orientation, double mu_wc, double sigma_wc)
{
 
    assert(sonar_sim);
    this->sonar_sim = sonar_sim;

    for (int i=0; i<num_particles; i++)
    {
        base::Vector2d pose_sampled =  samplePose(mu_pose,sigma_pose);
        double yaw_sampled = sampleValue(mu_orientation, sigma_orientation);
        double water_column_sampled = sampleValue(mu_wc,sigma_wc);
        xi_k.push_back(Particle(pose_sampled, yaw_sampled,water_column_sampled));
    }
}

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



void PoseEstimator::project(const base::TwistWithCovariance& input_velocities, double dt)
{
    double spread =0;// weightingFunction( max_weight, 0.0, config.spreadThreshold, 0.0 ); 

    for(size_t i=0; xi_k.size();i++)
    {   
        base::Vector3d linear_velocity = input_velocities.getLinearVelocity();
        base::Vector3d ang_velocity = input_velocities.getAngularVelocity();
        base::Matrix3d linear_cov =  input_velocities.getLinearVelocityCov();
        base::Matrix3d ang_cov = input_velocities.getAngularVelocityCov();

        double n_u = sampleValue(0,linear_cov(0,0));
        double n_v = sampleValue(0,linear_cov(1,1));
        double n_r = sampleValue(0,ang_cov(2,2));

        base::Vector3d velocity_with_error(linear_velocity[0]+n_u,
                                           linear_velocity[1]+n_v,
                                           ang_velocity[2]+n_r);

        Particle &p(xi_k[i]);
        
        //correcting the heading of the vehicle
        Eigen::AngleAxisd correct_yaw = Eigen::AngleAxisd(p.yaw, Eigen::Vector3d::UnitZ());
        velocity_with_error = correct_yaw*velocity_with_error*dt;
              
        p.xy_position += velocity_with_error.head(2);
        p.yaw += velocity_with_error[2];
        
        //If the max_weight is below a threshold (i.e the filter is lost) 
        //add random noise (ideally proportional to "how lost you are) to the
        //particle, to try to recover yourself.
        if(spread>0)
        {
            const double position_factor = config.spreadTranslationFactor * spread;
            const double orientation_factor = config.spreadRotationFactor * spread;
            const double water_column_factor = config.spreadWaterColumnFactor * spread;
            p.xy_position += samplePose(base::Vector2d(),
                    base::Vector2d(position_factor,position_factor));
            p.yaw += sampleValue(0,orientation_factor);
            p.water_column += sampleValue(0,water_column_factor);  
        }
    
    }

}


double PoseEstimator::compareSonarData(const base::samples::Sonar& real, const base::samples::Sonar& sim)
{
  
    double weight = 1;
    if(real.bins.size()!=sim.bins.size())
    {
        //TODO error
    }

    for(size_t i = 0; i<real.bins.size();i++)
    {
        double diff = abs(real.bins[i]-sim.bins[i]);
        weight *= diff; 
    }
  
    return weight;
  
  
  
  
  
  
    //TODO validate that the two beams are equivalent
    
    //Last compute the distance r-s and calculate the prob
 //   int bin_count = real.bin_count;
 //   int accumulated_dist = 0;

 //   for(int beam=0; beam < real.beam_count ; beam++)
 //   {   
 //       int beam_idx = beam*bin_count; 
 //       std::vector<float>::const_iterator it_real,it_sim;
 //       std::vector<float>::const_iterator real_beam_begin = real.bins.begin()+beam_idx;
 //       std::vector<float>::const_iterator real_beam_end = real.bins.begin()+beam_idx+bin_count;
 //       std::vector<float>::const_iterator sim_beam_begin = sim.bins.begin()+beam_idx;
 //       std::vector<float>::const_iterator sim_beam_end = sim.bins.begin()+beam_idx+bin_count;


 //       it_real = std::max_element(real_beam_begin,real_beam_end);
 //       it_sim =  std::max_element(sim_beam_begin,sim_beam_end);
 //       
 //       if(real.bins[real_idx] < config.minSonarThreshold)
 //           it_real = real_beam_end;
 //       if(sim.bins[sim_idx]< config.minSonarThreshold)
 //           it_sim = sim_beam_end;
 //
 //       int real_idx =  std::distance(real_beam_begin,it_real);
 //       int sim_idx =  std::distance(sim_beam_begin,it_sim);
 //       
 //              
 //       accumulated_dist += (real_idx-sim_idx);       
 //   }
 //   

 //   return accumulated_dist;
}


void PoseEstimator::updateWeights(const base::samples::Sonar& sonar_data, const Eigen::Affine3d& sonar_tf,const Eigen::Affine3d& abs_data)
{
    for(size_t i=0; xi_k.size();i++)
    {
        //For each particle we need to introduce the information from the
        //absolute sensors (depth and pitch and roll), as well the
        //transformation of the PTU when avaiable
        Particle &p(xi_k[i]);

        Eigen::Vector3d particle_position(p.xy_position[0],p.xy_position[1],0);
        particle_position += abs_data.translation();

        Eigen::Quaterniond particle_orientation(Eigen::AngleAxisd(p.yaw,
                Eigen::MatrixBase<base:: Vector3d>::UnitZ()));
        particle_orientation=particle_orientation*abs_data.rotation();



        Eigen::Affine3d simulated_sonar_pose;
        simulated_sonar_pose.translation() = particle_position;
        
        double avg_bearing = (sonar_data.bearings.begin()->rad + sonar_data.bearings.end()->rad)/2;
        

        simulated_sonar_pose.rotate(particle_orientation 
            * Eigen::AngleAxisd(avg_bearing, Eigen::Vector3d::UnitZ()));
  
        //TODO incorporate the PTU tf
        
        // simulate sonar data
        base::samples::Sonar sim_sonar_data = sonar_sim->simulateSonarData(simulated_sonar_pose);

        p.weight =  compareSonarData(sonar_data,sim_sonar_data);
    

    }    

}

base::Pose PoseEstimator::getCentroid()
{
    normalizeWeights();

    // calculate the weighted mean for now
    base::Pose2D mean;
    double water_column_mean = 0.0;
    double sumWeights = 0.0;
    for(size_t i=0;i<xi_k.size();i++)
    {
	const Particle &particle(xi_k[i]);
	//if( !particle.x.floating )
	{
	    mean.position += particle.xy_position * particle.weight;
	    mean.orientation += particle.yaw * particle.weight;
	    water_column_mean += particle.water_column * particle.weight;
	    sumWeights += particle.weight;
	}
    }
    mean.position /= sumWeights;
    mean.orientation /= sumWeights;
    water_column_mean /= sumWeights;


    base::Orientation mean_orientantion = base::Orientation(Eigen::AngleAxisd( mean.orientation, Eigen::Vector3d::UnitZ()));

    // and convert into a 3d position
    base::Pose result( 
	    base::Vector3d( mean.position.x(), mean.position.y(), water_column_mean ), 
	    mean_orientantion);

    return result;
}

Statistics PoseEstimator::getStatistics()
{
    Statistics stats;

    for(size_t i=0; i<xi_k.size(); i++)
    {
        const Particle &particle(xi_k[i]);
        stats.m_x += particle.xy_position[0];
        stats.m_y += particle.xy_position[1];
        stats.m_yaw += particle.yaw;
        stats.m_wc += particle.water_column;
    }
    
    stats.m_x /= xi_k.size();
    stats.m_y /= xi_k.size();
    stats.m_yaw /= xi_k.size();
    stats.m_wc /= xi_k.size();

    for(size_t i=0; i<xi_k.size(); i++)
    {
        const Particle &particle(xi_k[i]);
        stats.s_x += (stats.m_x - particle.xy_position[0])*(stats.m_x - particle.xy_position[0]);
        stats.s_y  += (stats.m_y - particle.xy_position[1])*(stats.m_y - particle.xy_position[1]);
        stats.s_yaw += (stats.m_yaw - particle.yaw)*(stats.m_yaw - particle.yaw);
        stats.s_wc +=  (stats.m_wc - particle.water_column)*(stats.m_wc - particle.water_column);
    }

    stats.s_x /= xi_k.size();
    stats.s_y /= xi_k.size();
    stats.s_yaw /= xi_k.size();
    stats.s_wc /= xi_k.size();


    return stats;


}



void PoseEstimator::update(const base::samples::Sonar& sonar_data, const Eigen::Affine3d& sonar_tf, const Eigen::Affine3d& abs_data)
{
    updateWeights(sonar_data, sonar_tf, abs_data);
    double eff = normalizeWeights();
    if( eff < config.minEffective )
    {
	resample();
    }
}
