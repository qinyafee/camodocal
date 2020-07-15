#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <stdint.h>

namespace camodocal
{

class Transform
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Transform();
    Transform(const Eigen::Matrix4d& H);

    Eigen::Quaterniond& rotation(void);
    const Eigen::Quaterniond& rotation(void) const;
    double* rotationData(void); //返回四元数的系数
    const double* const rotationData(void) const;

    Eigen::Vector3d& translation(void);
    const Eigen::Vector3d& translation(void) const;
    double* translationData(void);
    const double* const translationData(void) const;

    Eigen::Matrix4d toMatrix(void) const;

private:
    Eigen::Quaterniond m_q;
    Eigen::Vector3d m_t;
};

}

#endif
