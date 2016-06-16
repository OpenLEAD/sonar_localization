#include "PoseEstimator.hpp"
#include <math.h>
using namespace imaging_sonar_localization;



PoseEstimator::PoseEstimator(const Configuration& config,
        gpu_sonar_simulation::MultibeamSonar& msonar,
        vizkit3d_normal_depth_map::NormalDepthMap& n_depth_map,
        vizkit3d_normal_depth_map::ImageViewerCaptureTool& capture): 
    ParticleFilter<PoseParticle>(config.seed),
    rand_norm(rand_gen, boost::normal_distribution<>(0,1.0) ),
    rand_uni(rand_gen, boost::uniform_real<>(0,1.0) ),
    max_weight(0),
    _msonar(msonar),
    _normal_depth_map(n_depth_map),
    _capture(capture),
    config(config)

{
}

PoseEstimator::~PoseEstimator()
{
}


base::Vector2d PoseEstimator::samplePose(const base::Vector2d& mu, const base::Vector2d& sigma)
{

    double x = rand_norm(), y= rand_norm();
    return base::Vector2d(x*sigma[0] + mu[0],
                          y*sigma[1] +mu[0]);

}

double PoseEstimator::sampleValue(double  mu, double sigma)
{  
    double rand = rand_norm();
    double value = rand*sigma + mu;
    return value;
}



void PoseEstimator::init(int num_particles, const base::Vector2d& mu_pose,
        const base::Vector2d& sigma_pose, double  mu_orientation, 
        double sigma_orientation, double mu_wc, double sigma_wc)
{
    for (int i=0; i<num_particles; i++)
    {
        base::Vector2d pose_sampled =  samplePose(mu_pose,sigma_pose);
        double yaw_sampled = sampleValue(mu_orientation, sigma_orientation);
        double water_column_sampled = sampleValue(mu_wc,sigma_wc);
        xi_k.push_back(Particle(pose_sampled, yaw_sampled,water_column_sampled));
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



void PoseEstimator::project(double u, double v, double r, double dt)
{
    double spread = 0; //weightingFunction( max_weight, 0.0, config.spreadThreshold, 0.0 ); 

    for(size_t i=0; xi_k.size();i++)
    {   
        double n_u = sampleValue(0,config.n_u_sigma);
        double n_v = sampleValue(0,config.n_v_sigma);
        double n_r = sampleValue(0,config.n_r_sigma);
        
        Particle &p(xi_k[i]);
       
        double dx = (u*dt+n_u*dt*dt/2)*cos(p.yaw)+(v*dt+n_v*dt*dt/2)*sin(p.yaw);
        double dy = (u*dt+n_u*dt*dt/2)*sin(p.yaw)+(v*dt+n_v*dt*dt/2)*cos(p.yaw);
        double dyaw = r*dt+n_r*dt*dt/2;

        p.xy_position[0] += dx;
        p.xy_position[1] += dy;
        p.yaw += dyaw;
        
    
        if(spread>0)
        {
        // TODO spread the particle to see if the PF recovers itself
        // spread position and orientation? 
        }
    
    }

}


void PoseEstimator::updateSimulatedSonarPose(base::samples::RigidBodyState pose) {

	// convert OSG (Z-forward) to RoCK coordinate system (X-forward)
	osg::Matrixd rock_coordinate_matrix = osg::Matrixd::rotate( M_PI_2, osg::Vec3(0, 0, 1)) * osg::Matrixd::rotate(-M_PI_2, osg::Vec3(1, 0, 0));

	// transformation matrixes multiplication
	osg::Matrixd matrix;
	matrix.setTrans(osg::Vec3(pose.position.x(), pose.position.y(), pose.position.z()));
	matrix.setRotate(osg::Quat(pose.orientation.x(), pose.orientation.y(), pose.orientation.z(), pose.orientation.w()));
	matrix.invert(matrix);

	// correct coordinate system and apply geometric transformations
	osg::Matrixd m = matrix * rock_coordinate_matrix;

	osg::Vec3 eye, center, up;
	m.getLookAt(eye, center, up);
	_capture.setCameraPosition(eye, center, up);
}

double PoseEstimator::compareSonarData(const base::samples::Sonar& real, const base::samples::Sonar& sim)
{
    //TODO validate that the two beams are equivalent
    
    //First threshold the data above config.sonarThreshold
    //Second extract the peak
    //Last compute the distance r-s and calculate the prob
    //std::vector<float> 
    //v.erase(std::remove_if(
    //            v.begin(), v.end(),
    //                [](const int& x) { 
    //                        return x > 10; // put your condition here
    //                            }), v.end());
    int bin_count = real.bin_count;
    int accumulated_dist = 0;

    for(int current_beam=0; current_beam < real.beam_count ; current_beam++)
    {   
        int current_beam_idx = current_beam*bin_count; 
        std::vector<float>::const_iterator it_real,it_sim;
        std::vector<float>::const_iterator real_current_beam_begin = real.bins.begin()+current_beam_idx;
        std::vector<float>::const_iterator real_current_beam_end = real.bins.begin()+current_beam_idx+bin_count;
        std::vector<float>::const_iterator sim_current_beam_begin = sim.bins.begin()+current_beam_idx;
        std::vector<float>::const_iterator sim_current_beam_end = sim.bins.begin()+current_beam_idx+bin_count;


        it_real = std::max_element(real_current_beam_begin,real_current_beam_end);
        it_sim =  std::max_element(sim_current_beam_begin,sim_current_beam_end);
        
        int real_idx =  std::distance(real_current_beam_begin,it_real);
        int sim_idx =  std::distance(sim_current_beam_begin,it_sim);

        std::cout << "Real Ping:" << real_idx << std::endl;
        std::cout << "Sim Ping"   << sim_idx << std::endl;
        
        accumulated_dist += (real_idx-sim_idx);       
    }
    

    return accumulated_dist;
}


void PoseEstimator::updateWeights(const base::samples::Sonar& sonar_data, Eigen::Affine3d& sonar_tf, base::samples::RigidBodyState& abs_data)
{
    for(size_t i=0; xi_k.size();i++)
    {
        //For each particle we need to introduce the information from the
        //absolute sensors (depth and pitch and roll), as well the
        //transformation of the PTU when avaiable
        Particle &p(xi_k[i]);

        Eigen::Vector3d particle_position(p.xy_position[0],p.xy_position[1],0);
        particle_position += abs_data.position;

        Eigen::Quaterniond particle_orientation(Eigen::AngleAxisd(p.yaw,
                Eigen::MatrixBase<base:: Vector3d>::UnitZ()));
        particle_orientation=particle_orientation*abs_data.orientation;



        base::samples::RigidBodyState simulated_sonar_pose;
        simulated_sonar_pose.position = particle_position;
        
        double avg_bearing = (sonar_data.bearings.begin()->rad + sonar_data.bearings.end()->rad)/2;
        

        simulated_sonar_pose.orientation = particle_orientation 
            * Eigen::AngleAxisd(avg_bearing, Eigen::Vector3d::UnitZ());
  
        //TODO incorporate the PTU tf
        
        // update the sonar pose in the depth map
        updateSimulatedSonarPose(simulated_sonar_pose);

	// receive shader image
	osg::ref_ptr<osg::Image> osg_image = 
            _capture.grabImage(_normal_depth_map.getNormalDepthMapNode());
	
        cv::Mat3f cv_image = gpu_sonar_simulation::convertShaderOSG2CV(osg_image);


        std::vector<float> sonar_data = _msonar.codeSonarData(cv_image);

        // apply the "gain" (in this case, it is a light intensity change)
        float gain_factor = _msonar.getGain() / 0.5;
        std::transform(sonar_data.begin(), sonar_data.end(), sonar_data.begin(), 
                    std::bind1st(std::multiplies<float>(), gain_factor));
        std::replace_if(sonar_data.begin(), sonar_data.end(), bind2nd(greater<float>(), 1.0), 1.0);

        // simulate sonar data
        base::samples::Sonar sonar = _msonar.simulateMultiBeam(sonar_data);

        //TODO compare simulated with real
        //TODO assign particle weight 
    

    }    
    return;

}



void PoseEstimator::update(const base::samples::Sonar& sonar_data, Eigen::Affine3d& sonar_tf)
{
    
    
    //updateWeights(beam,sonar_tf);
    double eff = normalizeWeights();
    if( eff < config.minEffective )
    {
	resample();
    }
}
