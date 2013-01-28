#ifndef MCV_H
#define MCV_H
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
using namespace cv;
class CVStuff{
public:
	CVStuff();
	CvMat* objectPts;
	CvMat* imgPts;
	CvMat* pointCounts;
	int numCaliPt;
	CvMat* camera_mat;
	CvMat* distort_mat;
	CvMat* Rot;
	CvMat* Rot44;
	CvMat* Trans;
	CvMat* Result;
	CvMat* RotAxis;
	float RotAngle;
	CvMat* MarkerRot;
	CvMat* MarkerTrans;
	CvMat* tempRot;
	CvMat* tempTrans;
};
#endif
