#include <camodocal/sparse_graph/SparseGraphUtils.h>

namespace camodocal
{

void
rectifyImagePoint(const CameraConstPtr& camera,
                  const cv::Point2f& src, cv::Point2f& dst)
{
    Eigen::Vector3d P;

    camera->liftProjective(Eigen::Vector2d(src.x, src.y), P);

    P /= P(2);

    dst.x = P(0);
    dst.y = P(1);
}

void
rectifyImagePoint(const CameraConstPtr& camera,
                  const Eigen::Vector2d& src, Eigen::Vector2d& dst)
{
    Eigen::Vector3d P;

    camera->liftProjective(src, P);

    P /= P(2);

    dst = P.block<2,1>(0,0);
}

void
rectifyImagePoints(const CameraConstPtr& camera,
                   const std::vector<cv::Point2f>& src,
                   std::vector<cv::Point2f>& dst)
{
    dst.resize(src.size());

    for (size_t i = 0; i < src.size(); ++i)
    {
        const cv::Point2f& p = src.at(i);

        Eigen::Vector3d P;
        camera->liftProjective(Eigen::Vector2d(p.x, p.y), P);

        P /= P(2);

        dst.at(i) = cv::Point2f(P(0), P(1));
    }
}

void pointsToPluckerLines(const CameraSystemConstPtr& cameraSystem, int camera_id,
                        const std::vector<cv::Point2f>& src,
                        std::vector<Eigen::Matrix<double, 6, 1> >& lines)
{
    //CameraPtr
    CameraConstPtr camera = cameraSystem->getCamera(camera_id); //是否要转化
    Eigen::Matrix4d extrinsic = cameraSystem->getGlobalCameraPose(camera_id);
    Eigen::Matrix3d rota = extrinsic.block(0,0,3,3);
    Eigen::Vector3d tran = extrinsic.block(0,3,3,1);

    std::vector<cv::Point2f> dst;
    dst.resize(src.size());
    rectifyImagePoints(camera, src, dst);

    Eigen::Vector3d pt2d;
    lines.resize(src.size());
    Eigen::Matrix<double, 6, 1> line;
    Eigen::Vector3d head;
    Eigen::Vector3d tail;

    for (size_t i = 0; i < src.size(); ++i)
    {   
        pt2d << dst[i].x, dst[i].y, 1.0;
        head = rota * pt2d;
        tail = tran.cross(head);
        line.head(3) = head;
        line.tail(3) = head;
        lines.push_back(line);
    }
}                        

}
