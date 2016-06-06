#include "Projection.hpp"

const double BAR2METER = 10.06;

using namespace imaging_sonar_localization;

Projection::Projection(const Configuration& config)
: config(config)
  //sampling(config)
{
}

Projection::~Projection()
{
}

void Projection::update(const base::samples::RigidBodyState& depth, const Eigen::Quaterniond& orientation)
{
    this->orientation = orientation;   


    prevOrientation = orientation;
}
