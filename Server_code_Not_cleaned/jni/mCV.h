#ifndef MCV_H
#define MCV_H
//#include <android/log.h>
//#include <stdio.h>
//#include <math.h>

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
	//CvMat* rotVec;
	//CvMat* transVec;
	CvMat* Rot;
	CvMat* Rot44;
	CvMat* Trans;
	CvMat* Result;
	CvMat* RotAxis;
	float RotAngle;


	//tracking part

	CvMat* preProj;
	CvMat* FirstProj;
	CvMat* SecProj;
	CvMat* FirstPose;
	CvMat* SecPose;
	CvMat* FirstKFPose;
	CvMat* SecKFPose;
	CvMat* KFRot;
	CvMat* KFTrans;
	CvMat* MarkerRot;
	CvMat* MarkerTrans;
	int LKWindSize;
	int LKWindSizeRatio;
	int LKMaxLevel;
	int LKWindSizeMin;
	std::vector<CvMat*> rots;
	std::vector<CvMat*> transs;
	int numGoodPtsThreshold;
	int KeyPointsFoundThreshold;
	int reFindKFThreshold;
	Mat previous;
		Mat KFprevious;
		bool first;
		bool firstKFPose;
		int FASTthreshold;
		std::vector<cv::Point2i> result;
		std::vector<cv::Point2i> previousPts;
		std::vector<cv::Point2i> previousMatchPts;

		std::vector<cv::Point2f> LKPts1;
		std::vector<cv::Point2f> LKPts2;
		std::vector<Point2f> KFLKPts1;
		std::vector<Point2f> KFLKPts2;
		std::vector<float> err;
		std::vector<float> KFerr;
		vector<uchar> status;
		std::vector<uchar> KFstatus;
		bool nonMax;
		bool working;
		bool KFWorking;
		bool MarkerWorking;
		CvPoint3D32f KFRotVector;
		CvPoint3D32f KFTransVector;
		bool PoseByKF;
		Mat camMat;
		Mat distortion;
		int trackingFeatureNumThreshold;
		CvMat* tempRot;
		CvMat* tempTrans;
};
#endif
