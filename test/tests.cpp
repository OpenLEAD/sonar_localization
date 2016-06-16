#include <gtest/gtest.h>

#include <imaging_sonar_localization/PoseEstimator.cpp>
#include <imaging_sonar_localization/Configuration.hpp>

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

            normal_depth_map = new vizkit3d_normal_depth_map::NormalDepthMap(range, viewX, viewY);
            capture = new vizkit3d_normal_depth_map::ImageViewerCaptureTool(640,480);
            capture->setBackgroundColor(osg::Vec4d(0, 0, 0, 0));

            osg::ref_ptr<osg::Group> root = new osg::Group();
            addOilRig(root);
            normal_depth_map->addNodeChild(root);
            

            config = new Configuration();
            a=0;
        }

        vizkit3d_normal_depth_map::NormalDepthMap* normal_depth_map;
        vizkit3d_normal_depth_map::ImageViewerCaptureTool* capture;
        gpu_sonar_simulation::MultibeamSonar sonar;
        Configuration* config;
        int a;

};

TEST_F(PoseEstimatorTest, InitTest){
    PoseEstimator filter(*config,sonar,*normal_depth_map,*capture);
    base::Vector2d mu_pose(1.0,1.0);
    base::Vector2d sigma_pose(1.0,1.0);
    filter.init(10, mu_pose, sigma_pose, 1.0,1.0,10.0,1.0);

    
    EXPECT_EQ(0,a);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
