#ifndef COSTFUNCTIONFACTORY2_H
#define COSTFUNCTIONFACTORY2_H

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>

#include "camodocal/camera_models/Camera.h"
#include "camodocal/camera_systems/CameraSystem.h"
// #include "../camera_models/CostFunctionFactory.h"

//fwd
namespace ceres
{
    class CostFunction;
}

namespace camodocal
{

class CostFunctionFactory2
{
public:
    CostFunctionFactory2();

    static boost::shared_ptr<CostFunctionFactory2> instance(void);

    //新增
    ceres::CostFunction* generateCostFunction(const CameraSystemConstPtr& cameraSystem, int idx,
                                              const Eigen::Vector2d& observed_p) const;
private:
    static boost::shared_ptr<CostFunctionFactory2> m_instance;
};

}

#endif
