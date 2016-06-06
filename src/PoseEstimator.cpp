#include "PoseEstimator.hpp"
#include <math.h>

using namespace imaging_sonar_localization;



PoseEstimator::PoseEstimator(const Configuration& config):
    ParticleFilter<PoseParticle>(10),
    rand_norm(rand_gen, boost::normal_distribution<>(0,1.0) ),
    rand_uni(rand_gen, boost::uniform_real<>(0,1.0) ),
    max_weight(0),
    projection(config)
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
        double wacter_column_sampled = sampleValue(mu_wc,sigma_wc);
        
        xi_k.push_back(Particle(pose_sampled, yaw_sampled,wacter_column_sampled));
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
        //TODO implement correctly the parameters
        double n_u_sigma, n_v_sigma, n_r_sigma = 0.01;

        double n_u = sampleValue(0,n_u_sigma);
        double n_v = sampleValue(0,n_v_sigma);
        double n_r = sampleValue(0,n_r_sigma);
        
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


void PoseEstimator::updateWeights(const base::samples::SonarBeam& beam, Eigen::Affine3d& sonar_tf, base::samples::RigidBodyState& abs_data)
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
        simulated_sonar_pose.orientation = particle_orientation 
            * Eigen::AngleAxisd(beam.bearing.rad, Eigen::Vector3d::UnitZ());
  
        //TODO incorporate the PTU tf
        
        // update the sonar pose in the depth map
        updateSimulatedSonarPose(simulated_sonar_pose);

	// receive shader image
	osg::ref_ptr<osg::Image> osg_image = _capture.grabImage(_normal_depth_map.getNormalDepthMapNode());
	cv::Mat3f cv_image = gpu_sonar_simulation::convertShaderOSG2CV(osg_image);

	// decode shader image
	std::vector<double> raw_intensity = _ssonar.decodeShaderImage(cv_image);

	// get ping data
	std::vector<uint8_t> sonar_data = _ssonar.getPingData(raw_intensity);

	// apply the "gain" (in this case, it is a light intensity change)
	double gain_factor = _ssonar.getGain() / 0.5;
	std::transform(sonar_data.begin(), sonar_data.end(), sonar_data.begin(), std::bind1st(std::multiplies<double>(), gain_factor));

	// simulate sonar data
	base::samples::SonarBeam sonar_beam = _ssonar.simulateSonarBeam(sonar_data);

       //TODO compare simulated with real
       //TODO assign particle weight 
    

    }    
    return;

}

void PoseEstimator::updateWeights(const base::samples::SonarScan& scan, Eigen::Affine3d& sonar_tf)
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
        simulated_sonar_pose.orientation = particle_orientation 
            * Eigen::AngleAxisd(beam.bearing.rad, Eigen::Vector3d::UnitZ());
    
        Task::updateSonarPose(pose);

	// receives shader image
	osg::ref_ptr<osg::Image> osg_image = _capture.grabImage(_normal_depth_map.getNormalDepthMapNode());
	cv::Mat3f cv_image = gpu_sonar_simulation::convertShaderOSG2CV(osg_image);

	// simulate sonar data
	std::vector<uint8_t> sonar_data = _msonar.codeSonarData(cv_image);

	// apply the "gain" (in this case, it is a light intensity change)
	double gain_factor = _msonar.getGain() / 0.5;
	std::transform(sonar_data.begin(), sonar_data.end(), sonar_data.begin(), std::bind1st(std::multiplies<double>(), gain_factor));

	// simulate sonar data
	base::samples::SonarScan sonar_scan = _msonar.simulateSonarScan(sonar_data);

        
        return;

}


void PoseEstimator::update(const base::samples::SonarBeam& beam, Eigen::Affine3d& sonar_tf)
{
    
    
    //updateWeights(beam,sonar_tf);
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
