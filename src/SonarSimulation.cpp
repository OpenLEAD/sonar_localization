#include "SonarSimulation.hpp"

using namespace imaging_sonar_localization;

SonarSimulation::SonarSimulation(double max_sensor_range, double fovX, double fovY,
        uint value, bool isHeight, osg::ref_ptr<osg::Group> root):
    normal_depth_map(max_sensor_range, fovX * 0.5, fovY * 0.5),
    capture_tool(fovY, fovY, value, isHeight)
{
    capture_tool.setBackgroundColor(osg::Vec4d(0.0, 0.0, 0.0, 1.0));
    normal_depth_map.addNodeChild(root);
}

SonarSimulation::~SonarSimulation()
{}


base::samples::Sonar SonarSimulation::simulateSonarData(const Eigen::Affine3d& sonar_pose)
{
    // update the sonar pose in the depth map
    updateSimulatedSonarPose(sonar_pose);

    // receive shader image
    osg::ref_ptr<osg::Image> osg_image = 
        capture_tool.grabImage(normal_depth_map.getNormalDepthMapNode());
	
    cv::Mat3f cv_image = gpu_sonar_simulation::convertShaderOSG2CV(osg_image);


    std::vector<float> sim_raw_sonar_data = sonar.codeSonarData(cv_image);

    // apply the "gain" (in this case, it is a light intensity change)
    float gain_factor = sonar.getGain() / 0.5;
    std::transform(sim_raw_sonar_data.begin(), sim_raw_sonar_data.end(), sim_raw_sonar_data.begin(), 
                    std::bind1st(std::multiplies<float>(), gain_factor));
    std::replace_if(sim_raw_sonar_data.begin(), sim_raw_sonar_data.end(), bind2nd(greater<float>(), 1.0), 1.0);

    // simulate sonar data
    
    base::samples::Sonar sim_sonar_data = sonar.simulateMultiBeam(sim_raw_sonar_data);

    return sim_sonar_data;
}

void SonarSimulation::updateSimulatedSonarPose(const Eigen::Affine3d pose)
{
    // convert OSG (Z-forward) to RoCK coordinate system (X-forward)
    osg::Matrixd rock_coordinate_matrix = osg::Matrixd::rotate( M_PI_2, osg::Vec3(0, 0, 1)) 
        * osg::Matrixd::rotate(-M_PI_2, osg::Vec3(1, 0, 0));

    // transformation matrixes multiplication
    osg::Matrixd matrix(pose.data());
    //matrix.setTrans(osg::Vec3(pose.translation()));
    //matrix.setRotate(osg::Quat(pose.orientation.x(), pose.orientation.y(), 
    //pose.orientation.z(), pose.orientation.w()));
    matrix.invert(matrix);

    // correct coordinate system and apply geometric transformations
    osg::Matrixd m = matrix * rock_coordinate_matrix;

    osg::Vec3 eye, center, up;
    m.getLookAt(eye, center, up);
    capture_tool.setCameraPosition(eye, center, up);
}

