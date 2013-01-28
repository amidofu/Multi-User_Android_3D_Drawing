#include "nativeFunctions_NativeLib.h"
#include "MarkerFinder.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <android/log.h>
#include "mCV.h"
#include <fstream>
using namespace cv;

CVStuff mCV;
#define numViews 40
Mat camMat(3,3,CV_32FC1);
Mat distortion(1,4,CV_32FC1);
bool Calibrated;
int ScreenWidth;
int ScreenHeight;
bool CalibrateGo=false;
bool Calibrating=false;
void Calibration();
float px,py,pz;
bool compareP;

void showcamMat()
{
	for(int i=0;i<3;i++)
	{
		__android_log_print(ANDROID_LOG_ERROR,"camMat","%f, %f, %f",CV_MAT_ELEM(*mCV.camera_mat,float,i,0),CV_MAT_ELEM(*mCV.camera_mat,float,i,1),CV_MAT_ELEM(*mCV.camera_mat,float,i,2));
	}
}

void showDistortionMat()
{
	__android_log_print(ANDROID_LOG_ERROR,"distortion","%f, %f, %f, %f",CV_MAT_ELEM(*mCV.distort_mat,float,0,0),CV_MAT_ELEM(*mCV.distort_mat,float,0,1),CV_MAT_ELEM(*mCV.distort_mat,float,0,2),CV_MAT_ELEM(*mCV.distort_mat,float,0,3));
}


JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_saveCameraParam
  (JNIEnv * env, jclass, jstring jfilePath)
{
	std::string s(env->GetStringUTFChars(jfilePath,0));
	__android_log_print(ANDROID_LOG_ERROR,"storage path","%s",s.c_str());
	int pathLength=env->GetStringLength(jfilePath);
	std::fstream fp;
	fp.open(s.c_str(),std::ios::out);
	if(!fp)
		__android_log_print(ANDROID_LOG_ERROR,"storage path","open fail");
	fp<<ScreenWidth<<" "<<ScreenHeight<<std::endl;
	for(int i=0;i<3;i++)
		fp<<CV_MAT_ELEM(*mCV.camera_mat,float,i,0)<<" "<<CV_MAT_ELEM(*mCV.camera_mat,float,i,1)<<" "<<CV_MAT_ELEM(*mCV.camera_mat,float,i,2)<<std::endl;

	fp<<CV_MAT_ELEM(*mCV.distort_mat,float,0,0)<<" "<<CV_MAT_ELEM(*mCV.distort_mat,float,0,1)<<" "<<CV_MAT_ELEM(*mCV.distort_mat,float,0,2)<<" "<<CV_MAT_ELEM(*mCV.distort_mat,float,0,3);
	fp.close();
}

JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_startCalibrate
  (JNIEnv *, jclass)
{
	CalibrateGo=true;
}

JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_FindRectangle     (JNIEnv *, jclass, jlong addrRgba, jint height, jint width, jint RectLimit)
{
	if(Calibrating)
		return;

	Mat* pMatRgb=(Mat*)addrRgba;
	IplImage ipl=*pMatRgb;

	MarkerRectangle* MR=new MarkerRectangle[RectLimit];
	const int RL=RectLimit;
	int NumRect=FindRectangle(&ipl,MR,RL);

	__android_log_print(ANDROID_LOG_INFO,"Num Rect Found:","%d",NumRect);
	if(NumRect>0)
	{
		DecodeMarker2DCode(&ipl, MR,NumRect);
		if(!CalibrateGo)
			return;

		if(!Calibrated)
		{
			if(MR[0].MarkerID==0)
			{
				for(int i=0;i<4;i++)
				{
					CV_MAT_ELEM(*mCV.imgPts,float,mCV.numCaliPt,0)=MR[0].outer_corners[i].x;
					CV_MAT_ELEM(*mCV.imgPts,float,mCV.numCaliPt,1)=MR[0].outer_corners[i].y;
					mCV.numCaliPt++;
				}
				__android_log_print(ANDROID_LOG_ERROR,"Num views:","%d",mCV.numCaliPt/4);
				if(mCV.numCaliPt==4*numViews)
				{
					Calibrating=true;
					Calibration();
					Calibrated=true;
					Calibrating=false;
				}
			}
			return;
		}
	}
	else
	{
		return;
	}

	EstimateMarkerRotation_TranlationVector(mCV.camera_mat,mCV.distort_mat,MR,NumRect);


	int selectR=0;


	if(MR[selectR].MarkerID==-1||MR[selectR].Direction==DIRECTION_UNKNOWN)
	{
		return;
	}
	drawAxis(&ipl,mCV.camera_mat,mCV.distort_mat,MR,NumRect);
	drawCornersOneRect(&ipl,mCV.camera_mat,mCV.distort_mat,MR[selectR]);
	delete [] MR;

}
JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_InitializedOpenCVGlobalVar
  (JNIEnv *, jclass, jint width, jint height)
{
	ScreenWidth=width;
	ScreenHeight=height;

	Calibrated=false;
	compareP=true;
	setMarkerDetectScreenResolution(width,height);
	initGlobalVars();
	mCV.objectPts=cvCreateMat (4*numViews, 3, CV_32FC1);
	int idx=0;
	for(int i=0;i<numViews*4;i++)
	{
		int c=i%4;

		switch(c)
		{
		case 0:
			CV_MAT_ELEM(*mCV.objectPts,float,i,0)=-45.0;
			CV_MAT_ELEM(*mCV.objectPts,float,i,1)=45.0;
			CV_MAT_ELEM(*mCV.objectPts,float,i,2)=0.0;
			continue;
		case 1:
			CV_MAT_ELEM(*mCV.objectPts,float,i,0)=-45.0;
			CV_MAT_ELEM(*mCV.objectPts,float,i,1)=-45.0;
			CV_MAT_ELEM(*mCV.objectPts,float,i,2)=0.0;
			continue;
		case 2:
			CV_MAT_ELEM(*mCV.objectPts,float,i,0)=45.0;
			CV_MAT_ELEM(*mCV.objectPts,float,i,1)=-45.0;
			CV_MAT_ELEM(*mCV.objectPts,float,i,2)=0.0;
			continue;
		case 3:
			CV_MAT_ELEM(*mCV.objectPts,float,i,0)=45.0;
			CV_MAT_ELEM(*mCV.objectPts,float,i,1)=45.0;
			CV_MAT_ELEM(*mCV.objectPts,float,i,2)=0.0;
			continue;
		}
	}
	mCV.imgPts=cvCreateMat(4*numViews, 2, CV_32FC1);
	mCV.pointCounts=cvCreateMat(numViews, 1, CV_32SC1);
	for(int i=0;i<numViews;i++)
		CV_MAT_ELEM(*mCV.pointCounts,int,i,0)=4;
	mCV.numCaliPt=0;
	mCV.camera_mat=cvCreateMat(3,3,CV_32FC1);
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
            CV_MAT_ELEM(*mCV.camera_mat,float,i,j)=0.0;
    }
	CV_MAT_ELEM(*mCV.camera_mat,float,0,0)=1.0;
	CV_MAT_ELEM(*mCV.camera_mat,float,1,1)=1.0;
	CV_MAT_ELEM(*mCV.camera_mat,float,2,2)=1.0;
	CV_MAT_ELEM(*mCV.camera_mat,float,0,2)=(float)(width/2);
	CV_MAT_ELEM(*mCV.camera_mat,float,1,2)=(float)(height/2);

	mCV.distort_mat=cvCreateMat(1,4,CV_32FC1);
    for(int i=0;i<4;i++)
        CV_MAT_ELEM(*mCV.distort_mat,float,0,i)=0.0;

    mCV.Rot=cvCreateMat(3,3,CV_32FC1);


	mCV.Rot44=cvCreateMat(4,4,CV_32FC1);
	mCV.Trans=cvCreateMat(4,1,CV_32FC1);
	mCV.Result=cvCreateMat(4,1,CV_32FC1);
	mCV.RotAxis=cvCreateMat(1,3,CV_32FC1);

}

void Calibration()
{
	CvSize size;
	size.width=ScreenWidth;
	size.height=ScreenHeight;
	__android_log_print(ANDROID_LOG_ERROR,"Calibration","start");
	//cvCalibrateCamera2(mCV.objectPts,mCV.imgPts,mCV.pointCounts,size,mCV.camera_mat,mCV.distort_mat,NULL,NULL,0);
	//cvCalibrateCamera2(mCV.objectPts,mCV.imgPts,mCV.pointCounts,size,mCV.camera_mat,mCV.distort_mat,NULL,NULL,CV_CALIB_USE_INTRINSIC_GUESS);
	cvCalibrateCamera2(mCV.objectPts,mCV.imgPts,mCV.pointCounts,size,mCV.camera_mat,mCV.distort_mat,NULL,NULL,0);
	__android_log_print(ANDROID_LOG_ERROR,"Calibration","finish");
	Calibrating=false;
}
