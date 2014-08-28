#ifndef __ODOMETRY_UNDERWATERVEHICLE_STATE_HPP__
#define __ODOMETRY_UNDERWATERVEHICLE_STATE_HPP__

#include <base/time.h>
namespace odometry{
struct UnderwaterVehicleState
{
    /** @brief timestamp */
    base::Time time;
    
    /** @brief pressure aquired from the depth sensor, unit bar */
    double pressure;
    
    //TODO aquire data from thursters and IMU
    
    
};
}
#endif
