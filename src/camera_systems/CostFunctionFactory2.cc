#include "CostFunctionFactory2.h"

#include "ceres/ceres.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/EquidistantCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "camodocal/camera_models/ScaramuzzaCamera.h"

namespace camodocal
{

//新增

template<typename T>
void  //input camera extrinsic, reference pose, output camera pose
worldToCameraTransform2(const T* const q_cam_odo, const T* const t_cam_odo,
                       const T* const q_ref, const T* const t_ref,
                       T* q, T* t)
{
    // Eigen::Quaternion<T> q_z_inv(cos(att_odo[0] / T(2)), T(0), T(0), -sin(att_odo[0] / T(2)));
    // Eigen::Quaternion<T> q_y_inv(cos(att_odo[1] / T(2)), T(0), -sin(att_odo[1] / T(2)), T(0));
    // Eigen::Quaternion<T> q_x_inv(cos(att_odo[2] / T(2)), -sin(att_odo[2] / T(2)), T(0), T(0));
    // Eigen::Quaternion<T> q_zyx_inv = q_x_inv * q_y_inv * q_z_inv;
    // T q_odo[4] = {q_zyx_inv.w(), q_zyx_inv.x(), q_zyx_inv.y(), q_zyx_inv.z()};  

    //取逆
    T q_odo[4] = {q_ref[3], -q_ref[0], -q_ref[1], -q_ref[2]};

    T q_odo_cam[4] = {q_cam_odo[3], -q_cam_odo[0], -q_cam_odo[1], -q_cam_odo[2]};

    T q0[4];
    ceres::QuaternionProduct(q_odo_cam, q_odo, q0);

    T t0[3];
    // T t_odo[3] = {p_odo[0], p_odo[1], p_odo[2]};
    T t_odo[3] = {t_ref[0], t_ref[1], t_ref[2]};

    ceres::QuaternionRotatePoint(q_odo, t_odo, t0);

    t0[0] += t_cam_odo[0];
    t0[1] += t_cam_odo[1];

    if (0)//(optimize_cam_odo_z)
    {
        t0[2] += t_cam_odo[2];
    }

    ceres::QuaternionRotatePoint(q_odo_cam, t0, t);
    t[0] = -t[0]; t[1] = -t[1]; t[2] = -t[2];

    // Convert quaternion from Ceres convention (w, x, y, z)
    // to Eigen convention (x, y, z, w)
    q[0] = q0[1]; q[1] = q0[2]; q[2] = q0[3]; q[3] = q0[0];
}

// variables: cameraRig reference pose, 3D point
template<class CameraT>
class ReprojectionError4
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ReprojectionError4(const std::vector<double>& intrinsic_params,
                       const Eigen::Quaterniond& cam_odo_q,
                       const Eigen::Vector3d& cam_odo_t,
                       const Eigen::Vector2d& observed_p)
     : m_intrinsic_params(intrinsic_params)
     , m_cam_odo_q(cam_odo_q), m_cam_odo_t(cam_odo_t)
     , m_observed_p(observed_p) {}


    // variables: 优化 cameraRig reference pose, 3D point
    template <typename T>
    bool operator()(const T* const ref_q, const T* const ref_t,
                    const T* const point, T* residuals) const
    {
        T cam_odo_q[4] = {T(m_cam_odo_q.w()), T(m_cam_odo_q.x()), T(m_cam_odo_q.y()), T(m_cam_odo_q.z())};
        T cam_odo_t[3] = {T(m_cam_odo_t(0)), T(m_cam_odo_t(1)), T(m_cam_odo_t(2))};

        T q[4], t[3];

        worldToCameraTransform2(cam_odo_q, cam_odo_t, ref_q, ref_t, q, t);

        std::vector<T> intrinsic_params(m_intrinsic_params.begin(), m_intrinsic_params.end());
        
        Eigen::Matrix<T,3,1> P;
        P(0) = T(point[0]);
        P(1) = T(point[1]);
        P(2) = T(point[2]);

        // project 3D object point to the image plane
        Eigen::Matrix<T,2,1> predicted_p;
        CameraT::spaceToPlane(intrinsic_params.data(), q, t, P, predicted_p);

        residuals[0] = predicted_p(0) - T(m_observed_p(0));
        residuals[1] = predicted_p(1) - T(m_observed_p(1));

        return true;
    }

private:
    // camera intrinsics
    std::vector<double> m_intrinsic_params;

    // observed camera-odometry transform, 外参
    Eigen::Quaterniond m_cam_odo_q;
    Eigen::Vector3d m_cam_odo_t;

    // observed 2D point
    Eigen::Vector2d m_observed_p;

    Eigen::Matrix2d m_sqrtPrecisionMat;

    // bool m_optimize_cam_odo_z;
};

boost::shared_ptr<CostFunctionFactory2> CostFunctionFactory2::m_instance;

CostFunctionFactory2::CostFunctionFactory2()
{

}

boost::shared_ptr<CostFunctionFactory2>
CostFunctionFactory2::instance(void)
{
    if (m_instance.get() == 0)
    {
        m_instance.reset(new CostFunctionFactory2);
    }

    return m_instance;
}


ceres::CostFunction*
CostFunctionFactory2::generateCostFunction(const CameraSystemConstPtr& cameraSystem, int idx,
                                          const Eigen::Vector2d& observed_p) const

{
    ceres::CostFunction* costFunction = 0;

    std::vector<double> intrinsic_params;
    cameraSystem->getCamera(idx)->writeParameters(intrinsic_params);

    // Eigen::Matrix4d H_odo_cam = m_T_cam_odo.toMatrix().inverse(); //外参
    Eigen::Matrix4d H_odo_cam = cameraSystem->getGlobalCameraPose(idx);
    Eigen::Matrix4d rota = H_odo_cam.block(0,0,3,3);
    Eigen::Quaterniond cam_odo_q = Eigen::Quaterniond(1,0,0,0);
    // Eigen::Quaterniond cam_odo_q = Eigen::Quaterniond(rota);    
    Eigen::Vector3d cam_odo_t = H_odo_cam.block(0,3,3,1);


    // switch (flags)
    // {
    // case CAMERARIG_POSE | POINT_3D: //2, 4, 3, 3:  <q, t, point, residuals>
        switch (cameraSystem->getCamera(idx)->modelType())
        {
        case Camera::KANNALA_BRANDT:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError4<EquidistantCamera>, 2, 4, 3, 3>(
                    new ReprojectionError4<EquidistantCamera>(intrinsic_params, cam_odo_q, cam_odo_t, observed_p));
            break;
        case Camera::PINHOLE:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError4<PinholeCamera>, 2, 4, 3, 3>(
                    new ReprojectionError4<PinholeCamera>(intrinsic_params, cam_odo_q, cam_odo_t, observed_p));
            break;
        case Camera::MEI:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError4<CataCamera>, 2, 4, 3, 3>(
                    new ReprojectionError4<CataCamera>(intrinsic_params, cam_odo_q, cam_odo_t, observed_p));
            break;
        case Camera::SCARAMUZZA:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError4<OCAMCamera>, 2, 4, 3, 3>(
                    new ReprojectionError4<OCAMCamera>(intrinsic_params, cam_odo_q, cam_odo_t, observed_p));
            break;
        }
    //     break;
    // }

    return costFunction;
}        

}

