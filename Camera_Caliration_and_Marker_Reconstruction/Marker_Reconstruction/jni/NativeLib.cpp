#include <fstream>
#include "nativeFunctions_NativeLib.h"
#include "MarkerFinder.h"
#include <map>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <android/log.h>
#include <stdio.h>
#include <string.h>
#include "functions.h"
using namespace cv;
CVStuff mCV;
#define numViews 40
bool Calibrated;
int ScreenWidth;
int ScreenHeight;
bool Calibrating=false;
void Calibration();
int numImgHaveToTriangulation=8;
int finalRansacTimes=20;
int numImgsToTriangulation=5;

std::string storagePath;
MSMap mMSM;
#ifdef COMPARERECON
MSMap mMSM2;
#endif
#define USE_REPROJ
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

JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_FindRectangle
  (JNIEnv *, jclass, jlong addrRgba, jint height, jint width, jint RectLimit)
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
    }
    	EstimateMarkerRotation_TranlationVector(mCV.camera_mat,mCV.distort_mat,MR,NumRect);
    	for(int i=0;i<NumRect;i++)
    	{
    		if(MR[i].Direction!= DIRECTION_UNKNOWN)
    	drawCornersOneRect(&ipl,
    		mCV.camera_mat,
    		mCV.distort_mat,
    		MR[i]);
    	}
    delete [] MR;
}

//#define SAMSUNG_NORMAL

//save image info (camera pose and image coordinates for marker corners) for unknown markers
JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_SaveNewMarkerFrame
  (JNIEnv *env, jclass, jlong addrRgba, jint height, jint width, jint RectLimit, jobject MarkerAssist)
{
	jclass MarkerAssistCls=env->GetObjectClass(MarkerAssist);


	jfieldID saveThisFrameID=env->GetFieldID(MarkerAssistCls,"saveFrameOK","Z");
	jfieldID NumMarkerFoundID=env->GetFieldID(MarkerAssistCls,"NumMarkerFound","I");
	jfieldID numImgsGotID=env->GetFieldID(MarkerAssistCls,"numImgsGot","[I");
	jfieldID MarkerIDID=env->GetFieldID(MarkerAssistCls,"MarkerID","[I");

    jintArray JNIMarkerIDArray=(jintArray)env->GetObjectField(MarkerAssist,MarkerIDID);
    jint* JNIMarkerID=env->GetIntArrayElements(JNIMarkerIDArray,0);
    jintArray JNINumImgsGotArray=(jintArray)env->GetObjectField(MarkerAssist,numImgsGotID);
    jint* JNINumImgsGot=env->GetIntArrayElements(JNINumImgsGotArray,0);

	Mat* pMatRgb=(Mat*)addrRgba;
    IplImage ipl=*pMatRgb;
	MarkerRectangle* MR=new MarkerRectangle[RectLimit];
	__android_log_print(ANDROID_LOG_INFO,"Num Rect Found:","Save Marker OK3");
	const int RL=RectLimit;
	int NumRect=FindRectangle(&ipl,MR,RL);

	__android_log_print(ANDROID_LOG_INFO,"Num Rect Found:","%d",NumRect);
	if(NumRect>1)
	{
	   DecodeMarker2DCode(&ipl, MR,NumRect);
	}
	else
	{
		jboolean b=false;
		env->SetBooleanField(MarkerAssist,saveThisFrameID,b);
		return;
	}

    int oldestAge=9999;
    int oldestMarkerIndex=-1;
    int oldestID=-1;
#ifdef COMPARERECON
    int oldestAge2=9999;
    int oldestMarkerIndex2=-1;
    int oldestID2=-1;
#endif
    for(int j=0;j<NumRect;j++)//find the oldest built marker
    {
      int tempID=MR[j].MarkerID;
      if(tempID==-1)//invalid marker
      	  continue;
      MSMap::iterator it;
      it=mMSM.find(tempID);//check if the marker is found before
      if(it!=mMSM.end())
      {
       	  std::cout<<"found marker in mMSM. ID:"<<tempID<<std::endl;
          MS tempMS=it->second;
          //find out the earlist reconstructed marker
          if(tempMS.buildTime<oldestAge)
          {
                oldestAge=tempMS.buildTime;
                oldestMarkerIndex=j;
                oldestID=tempMS.ID;
          }
       }

#ifdef COMPARERECON
      MSMap::iterator it2;
      it2=mMSM2.find(tempID);
      if(it!=mMSM2.end())
      {
       	  std::cout<<"found marker in mMSM. ID:"<<tempID<<std::endl;
          MS tempMS2=it2->second;
          if(tempMS2.buildTime<oldestAge2)
          {
                oldestAge2=tempMS.buildTime2;
                oldestMarkerIndex2=j;
                oldestID2=tempMS.ID;
          }
       }
#endif
     }

     if(oldestMarkerIndex!=-1)//use the oldest found marker to estimate camera pose
     {
          int tempID=MR[oldestMarkerIndex].MarkerID;
          std::cout<<"found oldest ID:"<<tempID<<std::endl;
          if(mMSM[tempID].valid)//check if marker is valid
          {
               mMSM[tempID].EstimateTR(MR[oldestMarkerIndex].outer_corners,mCV.camera_mat,mCV.distort_mat,mCV.tempRot,mCV.tempTrans);
           }
           else
           {
        	   //tell Java layer that the preview frame is not saved
  			  jboolean b=false;
  			  env->SetBooleanField(MarkerAssist,saveThisFrameID,b);
              return;
           }
      }
      else
      {
    	  //tell Java layer that the preview frame is not saved
			  jboolean b=false;
			  env->SetBooleanField(MarkerAssist,saveThisFrameID,b);
            return;
       }

     int JNIInd=0;

        for(int j=0;j<NumRect;j++)
        {
            int tempID=MR[j].MarkerID;
            if(tempID==-1||tempID==0)
                continue;
            if(j==oldestMarkerIndex)
                continue;
            MSMap::iterator it;
            it=mMSM.find(tempID);
            if(it!=mMSM.end())
            {
                std::cout<<"found young marker, add Image, ID:"<<tempID<<std::endl;
                mMSM[tempID].addImageInfo(MR[j].outer_corners,mCV.tempRot,mCV.tempTrans,mMSM[oldestID].buildTime);
                std::cout<<"Num Img:"<<mMSM[tempID].numImgs<<std::endl;
                JNIMarkerID[JNIInd]=tempID;
                JNINumImgsGot[JNIInd]=mMSM[tempID].numImgs;
            }
            else//got a marker which has never been seen
            {
                std::cout<<"found new MS, add image, ID:"<<tempID<<std::endl;
                MS newMS(tempID);
                mMSM.insert(MSPair(tempID,newMS));
                mMSM[tempID].addImageInfo(MR[j].outer_corners,mCV.tempRot,mCV.tempTrans,mMSM[oldestID].buildTime);
                std::cout<<"Num Img:"<<mMSM[tempID].numImgs<<std::endl;
                JNIMarkerID[JNIInd]=tempID;
                JNINumImgsGot[JNIInd]=mMSM[tempID].numImgs;
            }
            JNIInd++;
        }
		  jboolean b=true;
		  env->SetBooleanField(MarkerAssist,saveThisFrameID,b);
		  env->ReleaseIntArrayElements(JNIMarkerIDArray,JNIMarkerID,0);
		  env->ReleaseIntArrayElements(JNINumImgsGotArray,JNINumImgsGot,0);
		  env->SetIntField(MarkerAssist,NumMarkerFoundID,JNIInd);

}
//compute 3D position of unknown markers
JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_ComputeMarkerPosition
  (JNIEnv *env, jclass, jobject MarkerAssist)
{
	jclass MarkerAssistCls=env->GetObjectClass(MarkerAssist);
	jfieldID BuiltMarkerSetID=env->GetFieldID(MarkerAssistCls,"BuiltMarkerSet","Ljava/util/HashSet;");
	jclass SetCls=env->FindClass("java/util/HashSet");
	jobject HashSet=env->GetObjectField(MarkerAssist,BuiltMarkerSetID);
	jmethodID SetAddID=env->GetMethodID(SetCls,"add","(Ljava/lang/Object;)Z");//return: Object, arguments: Object, Object. Only argument is class need ";"
	jclass MyIntCls=env->FindClass("nativeFunctions/MyInt");
	jmethodID MyIntConstruct=env->GetMethodID(MyIntCls,"<init>","()V");//()V: return: Void, argument: no
	jfieldID MyIntValueID=env->GetFieldID(MyIntCls,"value","I");

    MSMap::iterator it;
    for(it=mMSM.begin();it!=mMSM.end();it++)
    {
        if(it->second.ID!=0)
        {
            if(it->second.numImgs>numImgHaveToTriangulation&&!it->second.valid)
            {
            	bool tValid=mMSM[it->second.ID].valid;
            	//bool good=RANSACTriangulation(mMSM[it->second.ID],mCV.camera_mat,finalRansacTimes);
            	//do the reconstruction
#ifdef USE_REPROJ
            	bool good=RANSACTriangulationByReproj(numImgsToTriangulation,mMSM[it->second.ID],mCV.camera_mat,mCV.distort_mat);//check by reprojection
#else
            	bool good=RANSACTriangulationByRect(numImgsToTriangulation,mMSM[it->second.ID],mCV.camera_mat,mCV.distort_mat);
#endif
            	if(good)
            		__android_log_print(ANDROID_LOG_INFO,"build Marker","Marker built ID: %d",it->second.ID);
            	else
            		__android_log_print(ANDROID_LOG_INFO,"it->second.ID","No Marker built");
            	jboolean b=true;
            	if(!tValid&&mMSM[it->second.ID].valid)
            	{
            		jobject MyInt;
            		MyInt=env->NewObject(MyIntCls,MyIntConstruct);
            		jint tID=it->second.ID;
            		env->SetIntField(MyInt,MyIntValueID,tID);
            		jboolean tempB=env->CallBooleanMethod(HashSet,SetAddID,MyInt);
            	}
            }
        }
    }
}

JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_InitialozeOpenCVGlobalVarByLoad
  (JNIEnv * env, jclass, jstring jfolderPath)
{
	storagePath.clear();
	storagePath=(env->GetStringUTFChars(jfolderPath,0));
	std::fstream fp;
	std::string s=storagePath;
	s.append("/CamParam.txt");
	fp.open(s.c_str());
	if(!fp)
	{
		__android_log_print(ANDROID_LOG_ERROR,"init OpenCV global","load fail");
	}


	//MR=new MarkerRectangle[1];
	int width,height;
	fp>>width;fp>>height;
	setMarkerDetectScreenResolution(width,height);

	mCV.camera_mat=cvCreateMat(3,3,CV_32FC1);
	mCV.distort_mat=cvCreateMat(1,4,CV_32FC1);

	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			float t;
			fp>>t;
			CV_MAT_ELEM(*mCV.camera_mat,float,i,j)=t;
		}
	}

	for(int i=0;i<4;i++)
	{
		float t;
		fp>>t;
		CV_MAT_ELEM(*mCV.distort_mat,float,0,i)=t;
	}

	mCV.MarkerRot=cvCreateMat(1,3,CV_32FC1);
	mCV.MarkerTrans=cvCreateMat(1,3,CV_32FC1);
	mCV.Rot=cvCreateMat(3,3,CV_32FC1);
	mCV.Rot44=cvCreateMat(4,4,CV_32FC1);
	mCV.Trans=cvCreateMat(4,1,CV_32FC1);
	mCV.Result=cvCreateMat(4,1,CV_32FC1);
	mCV.RotAxis=cvCreateMat(1,3,CV_32FC1);

		initGlobalVars();

		//////////////////////////////////
		   ScreenWidth=width;
		    ScreenHeight=height;

		    Calibrated=false;


		    std::vector<Point3f> OriginPts;
		     Point3f pt1,pt2,pt3,pt4;
		     pt1.x=-45.0f;pt1.y=45.0;pt1.z=0.0;
		     pt2.x=-45.0f;pt2.y=-45.0;pt2.z=0.0;
		     pt3.x=45.0f;pt3.y=-45.0;pt3.z=0.0;
		     pt4.x=45.0f;pt4.y=45.0;pt4.z=0.0;
		     OriginPts.push_back(pt1);OriginPts.push_back(pt2);OriginPts.push_back(pt3);OriginPts.push_back(pt4);
		     MS origin( OriginPts,0);
		     origin.buildTime=0;
		     origin.camera_mat=cvCloneMat(mCV.camera_mat);
		     origin.distort_mat=cvCloneMat(mCV.distort_mat);
		     mMSM.insert(MSPair(origin.ID,origin));

		     mCV.tempRot=cvCreateMat(1,3,CV_32FC1);
		     mCV.tempTrans=cvCreateMat(1,3,CV_32FC1);
}
JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_saveAllMarkers
  (JNIEnv *, jclass)
{
	std::fstream fp;
	fp.open("/sdcard/marker_data/sMarker.txt",std::ios::out);
	if(!fp)
	{
		__android_log_write(ANDROID_LOG_ERROR,"failed","write markers");
		fp.close();
		return;
	}
    MSMap::iterator it;
    int numValid=0;
    for(it=mMSM.begin();it!=mMSM.end();it++)
    {
    	MS temp=it->second;
    	if(temp.valid)
    	{
    		numValid++;
    	}
    }
    fp<<numValid<<std::endl;
    for(it=mMSM.begin();it!=mMSM.end();it++)
    {
    	MS temp=it->second;
    	if(temp.valid)
    	{
    		fp<<temp.ID<<std::endl;
    		fp<<temp.buildTime<<std::endl;
#ifdef USE_REPROJ
    		projMScorners(temp);
    		rectifyRectangle2(temp);
#endif
    		for(int i=0;i<4;i++)
    		{
    			fp<<temp.WPts.at(i).x<<" "<<temp.WPts.at(i).y<<" "<<temp.WPts.at(i).z<<std::endl;
    		}
    		fp<<std::endl;
    	}
    }

	fp.close();
}
//clear unknown markers that can't be well reconstructed
JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_clearNonValidMarkerImage
  (JNIEnv *, jclass)
{
    MSMap::iterator it;
    std::vector<int> index;
    for(it=mMSM.begin();it!=mMSM.end();it++)
    {
    	MS temp=it->second;
    	if(!temp.valid)
    	{
    		int ID=it->second.ID;
    		index.push_back(ID);
    	}
    }
    for(int i=0;i<index.size();i++)
    {
    	int ID=index.at(i);
    	mMSM[ID].clearData();
    	mMSM.erase(ID);
    }
}
//try to find better reconstruction for markers
JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_reComputeMarkerPosition
  (JNIEnv *, jclass)
{
    MSMap::iterator it;
    for(it=mMSM.begin();it!=mMSM.end();it++)
    {
        if(it->second.ID!=0)
        {
            if(it->second.numImgs>numImgHaveToTriangulation)
            {
#ifdef USE_REPROJ
            	bool good=RANSACTriangulationByReproj(numImgsToTriangulation,mMSM[it->second.ID],mCV.camera_mat,mCV.distort_mat);//check by reprojection
#else
            	bool good=RANSACTriangulationByRect(numImgsToTriangulation,mMSM[it->second.ID],mCV.camera_mat,mCV.distort_mat);
#endif
            }
        }
    }
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
