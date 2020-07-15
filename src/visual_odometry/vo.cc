#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "FeatureTracker.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include<time.h> 
#include "DLib/FileFunctions.h"

#include <boost/make_shared.hpp>
#include <boost/unordered_set.hpp>
#include <cstdio>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <camodocal/sparse_graph/SparseGraphUtils.h>
#include "ceres/ceres.h"
#include "../camera_models/CostFunctionFactory.h"
#include "camodocal/EigenUtils.h"
#include "../npoint/five-point/five-point.hpp"
#include "../pose_estimation/P3P.h"

using namespace camodocal;
using namespace std;

int main(int argc, char* argv[]){
  if(argc!=3)  {
    std::cerr << "Usage: ./pipeline <image directory> <image extension .ext>" << endl; // <frequency>
    return 0;
  }

  // CataCamera::Parameters(const std::string& cameraName,
  //           int w, int h,
  //           double xi,
  //           double k1, double k2, double p1, double p2,
  //           double gamma1, double gamma2, double u0, double v0);
  // CataCamera::Parameters cameraParameters("cam0", imageSize.width, imageSize.height,
  //                                         0.9, 0.0, 0.0, 0.0, 0.0, f, f,
  //                                         imageSize.width / 2.0, imageSize.height / 2.0);
  // CataCameraPtr camera(new CataCamera(cameraParameters));

  // PinholeCamera::Parameters(const std::string& cameraName,
  //           int w, int h,
  //           double k1, double k2, double p1, double p2,
  //           double fx, double fy, double cx, double cy);

  // cv::Size imageSize(1.392000e+03, 5.120000e+02);
  // double k1=-3.728755e-01;
  // double k2=2.037299e-01;
  // double p1=2.219027e-03;
  // double p2=1.383707e-03;
  // double fx=9.842439e+02;
  // double fy=9.808141e+02;
  // double cx=6.900000e+02;
  // double cy=2.331966e+02;

  cv::Size imageSize(1280, 720);
  double k1 = 0;
  double k2 = 0;
  double p1 = 0;
  double p2 = 0;
  double fx = 1090.09;
  double fy = 818.8;
  double cx = 640;
  double cy = 360;


  PinholeCamera::Parameters cameraParameters("cam0", imageSize.width, imageSize.height,
                                              k1, k2, p1, p2,
                                              fx, fy, cx, cy);
  PinholeCameraPtr camera(new PinholeCamera(cameraParameters));

  // TemporalFeatureTracker track0(camera);
  TemporalFeatureTracker track0(camera, SURF_DETECTOR, SURF_DESCRIPTOR);
  // track0.setVerbose(true);
  // Vector of paths to image
  vector<string> imgfiles = DUtils::FileFunctions::Dir(argv[1], argv[2], true);

  // vector<string> imgfiles = DUtils::FileFunctions::Dir("/home/yafei/Documents/03data/KITTI/2011_09_26/2011_09_26_drive_0079_sync/image_00/data", ".png", true);

  // vector<string> imgfiles = DUtils::FileFunctions::Dir("/home/yafei/Documents/03data/Camera_01", ".png", true);

  cout << "Images: " << imgfiles.size() << endl;

  clock_t start, end;
  start = clock();
  cv::Mat m_mask = cv::Mat();
  for(size_t i=0; i<imgfiles.size(); ++i){
    //cout << imgfiles[i] << '\n';
    //cout << pcfiles[i] << '\n';
    cv::Mat src(cv::imread(imgfiles[i]));
    
    //boost::make_shared<Pose>();
    FramePtr frame = boost::make_shared<Frame>();
    frame->image() = src;
    frame->cameraPose() = boost::make_shared<Pose>();
    track0.addFrame(frame, m_mask);

  }

  end = clock();   //结束时间
  std::cout << "time = "<<double(end-start)/CLOCKS_PER_SEC<<"s"<<endl;

  return 0;
}
