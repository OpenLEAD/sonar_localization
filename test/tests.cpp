#include <gtest/gtest.h>

#include <imaging_sonar_localization/PoseEstimator.cpp>
#include <imaging_sonar_localization/Configuration.hpp>
#include <imaging_sonar_localization/SonarSimulation.hpp>

#include <vizkit3d_normal_depth_map/ImageViewerCaptureTool.hpp>
#include <vizkit3d_normal_depth_map/NormalDepthMap.hpp>
#include <gpu_sonar_simulation/MultibeamSonar.hpp>
#include <vizkit3d_world/Vizkit3dWorld.hpp>
#include <gpu_sonar_simulation/MultibeamSonar.hpp>
#include <string>
#include <iostream>

#include <osg/Geode>
#include <osg/Group>
#include <osg/ShapeDrawable>
#include <osg/Transform>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>



using namespace imaging_sonar_localization;

// add an oil rig manifold to the scene
void addOilRig(osg::ref_ptr<osg::Group> root){

    std::string current_path(__FILE__);
    current_path = current_path.substr(0, current_path.find_last_of("/"));
    osg::Node* oilring = osgDB::readNodeFile(current_path + "/oil_rig_manifold/visual.dae.osgb");

	osg::Matrix mtransf;
	mtransf.preMult(osg::Matrix::translate(0, 13, 0));
	mtransf.preMult(osg::Matrix::scale(0.1f,0.1f,0.1f));
	osg::MatrixTransform *ptransform = new osg::MatrixTransform();
	ptransform->setMatrix(mtransf);
	ptransform->addChild(oilring);
	root->addChild(ptransform);
}



class PoseEstimatorTest : public::testing::Test
{
    protected:
        virtual void SetUp()
        {

            osg::ref_ptr<osg::Image> osg_image;

            // init scene
            float viewX = 3.0, viewY = 35.0;
            double range = 60.0;

            osg::ref_ptr<osg::Group> root = new osg::Group();
            addOilRig(root);
           
            sonar_sim = new SonarSimulation(range, viewX, viewY, 100, true, root);
            config = new Configuration();
            a=0;
        }

        SonarSimulation* sonar_sim;
        Configuration* config;
        int a;

};

TEST_F(PoseEstimatorTest, DISABLED_InitTest){
    PoseEstimator filter(*config);

     //mean of the initial xy pose dist
    base::Vector2d mu_pose(1.0,1.0);
    //variance of the initial xy pose dist
    base::Vector2d sigma_pose(.1,.1);

    double mu_yaw = 1.0;
    double sigma_yaw = 0.1;

    double mu_wc = 1.0;
    double sigma_wc = 0.1;

    filter.init(1000000,sonar_sim, mu_pose, sigma_pose, 
            mu_yaw,sigma_yaw,mu_wc,sigma_wc);


    Statistics init_stats = filter.getStatistics();

    EXPECT_NEAR(1.0,init_stats.m_x,0.001);
    EXPECT_NEAR(1.0,init_stats.m_y,0.001);
    EXPECT_NEAR(1.0,init_stats.m_yaw,0.001);
    EXPECT_NEAR(1.0,init_stats.m_wc,0.001);

    EXPECT_NEAR(0.01,init_stats.s_x,0.0001);
    EXPECT_NEAR(0.01,init_stats.s_y,0.0001);
    EXPECT_NEAR(0.01,init_stats.s_yaw,0.0001);
    EXPECT_NEAR(0.01,init_stats.s_wc,0.0001);
    
}

TEST_F(PoseEstimatorTest,InitTestwithZeroError){

    PoseEstimator filter(*config);

     //mean of the initial xy pose dist
    base::Vector2d mu_pose(0.0,0.0);
    //variance of the initial xy pose dist
    base::Vector2d sigma_pose(0.0,0.0);

    double mu_yaw = 0.0;
    double sigma_yaw = 0.0;

    double mu_wc = 0.0;
    double sigma_wc = 0.0;

    filter.init(10,sonar_sim, mu_pose, sigma_pose, 
            mu_yaw,sigma_yaw,mu_wc,sigma_wc);

    Statistics init_stats = filter.getStatistics();
    EXPECT_NEAR(.0,init_stats.m_x,0.00001);
    EXPECT_NEAR(.0,init_stats.m_y,0.00001);
    EXPECT_NEAR(.0,init_stats.m_yaw,0.00001);
    EXPECT_NEAR(.0,init_stats.m_wc,0.00001);

    EXPECT_NEAR(0.0,init_stats.s_x,0.00000001);
    EXPECT_NEAR(0.0,init_stats.s_y,0.00000001);
    EXPECT_NEAR(0.0,init_stats.s_yaw,0.00000001);
    EXPECT_NEAR(0.0,init_stats.s_wc,0.00000001);
}

TEST_F(PoseEstimatorTest,ProjectLinearVelTest){

    PoseEstimator filter(*config);

     //mean of the initial xy pose dist
    base::Vector2d mu_pose(0.0,0.0);
    //variance of the initial xy pose dist
    base::Vector2d sigma_pose(0.0,0.0);

    double mu_yaw = 0.0;
    double sigma_yaw = 0.0;

    double mu_wc = 0.0;
    double sigma_wc = 0.0;

    filter.init(1000000, sonar_sim, mu_pose, sigma_pose, 
            mu_yaw,sigma_yaw,mu_wc,sigma_wc);

    Statistics init_stats = filter.getStatistics();
    
    base::TwistWithCovariance input;
    input.vel = base::Vector3d(1,1,0);
    input.rot = base::Vector3d(0,0,0);
    input.cov << 0.1, 0,  0, 0, 0, 0,
                 0,   0.1, 0, 0, 0, 0,
                 0,   0,  0, 0, 0, 0, 
                 0,   0,  0, 0, 0, 0,
                 0,   0,  0, 0, 0, 0,
                 0,   0,  0, 0, 0, 0;
    
    filter.project(input,1);             
    
    EXPECT_NEAR(1.0,init_stats.m_x,0.00001);
    EXPECT_NEAR(1.0,init_stats.m_y,0.00001);
    EXPECT_NEAR(.0,init_stats.m_yaw,0.00001);
    EXPECT_NEAR(.0,init_stats.m_wc,0.00001);

    EXPECT_NEAR(0.1,init_stats.s_x,0.00000001);
    EXPECT_NEAR(0.1,init_stats.s_y,0.00000001);
    EXPECT_NEAR(0.0,init_stats.s_yaw,0.00000001);
    EXPECT_NEAR(0.0,init_stats.s_wc,0.00000001);
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
