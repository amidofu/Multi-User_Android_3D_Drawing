#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <android/log.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/core/internal.hpp>
#include "mCV.h"
#include <time.h>
using namespace cv;
//convert rotation axis and angle to rotation matrix
void AxisAngleToMatrix(CvMat* & Mat,CvMat* & Axis,float & angle);
//convert rotation matrix to rotation axis and angle
void MatrixToAxisAngle(CvMat* & Mat,CvMat* & Axis,float & angle);
//convert a list of point2f to CvMat
void point2fToCvMat(CvMat* & mat,std::vector<Point2f> & points,bool TwoByN);
//convert CvMat to a list of point2f
void CvMatToPoint2f(CvMat* & mat, std::vector<Point2f> &  points,bool TwoByN);
//undistortion of image point
void MundistortPoints( InputArray src, OutputArray dst,
        InputArray cameraMatrix, InputArray distCoeffs,
        InputArray R=noArray(), InputArray P=noArray());
//compute translation
void computeTrans(CvMat* & trans,CvMat* & rot,CvMat* & Rot, CvMat* & Rot44,CvMat* & Result, float & x, float &y, float & z);
//rotation axis and angle to rotation vector
void AxisAngleToRotVec(CvMat* & rot,CvMat* & Axis,float & angle);
//rotation vector to rotation axis and angle
void RotVecToAxisAngle(CvMat* & rot,CvMat* & Axis,float & angle);
//project 3D points to 2D image
void projectPoints(std::vector<Point3f> & objPts,CvMat* & rot,CvMat* & trans,CvMat* & cam_mat,CvMat* & distort,std::vector<Point2f> & result);
