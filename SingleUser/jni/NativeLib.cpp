#include "nativeFunctions_NativeLib.h"
#include "serverMain.h"
#include "osgMain.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include "MarkerFinder.h"
#include "mCV.h"
#include "MarkerStorage.h"
using namespace std;
using namespace cv;
osgMain osgmain;
Server server;
CVStuff mCV;
osg::Vec3 pose;
osg::Matrixd Correction;
osg::Matrixd MarkerRot;
bool FirstPose;
MarkerRectangle* MR;
MSMap mMSM;
std::string storagePath;
std::string OSGStoragePath;

bool MarkerInsight=false;
bool useSensor=false;
bool fixVA=false;
#define QUATLERP
#ifdef QUATLERP
std::vector<osg::Quat> quats;
std::vector<osg::Vec3> Vtrans;
float lerpRatio=0.5;
#endif
#define CHANGE_COLOR
#define CHECK_SELECT
void writeNode(osg::Node * node,std::ofstream & fp)
{
    std::stringstream stst;
    osg::ref_ptr<osgDB::ReaderWriter> rw=osgDB::Registry::instance()->getReaderWriterForExtension("osg");
    rw->writeNode(*node,stst);
    fp<<stst.str();
}
osg::ref_ptr<osg::Geode> readNode(std::fstream & fp)
{
    std::string str("");
    std::string temp;
    while(fp>>temp)
    {
    	if(0!=strcmp("finish",temp.c_str()))
    		str+=(temp+" ");
    	else
    		break;
    }
    std::cout<<str<<std::endl;
    std::stringstream stst(str);
    osg::ref_ptr<osgDB::ReaderWriter> rw;

    rw=osgDB::Registry::instance()->getReaderWriterForExtension("osg");
    osg::ref_ptr<osg::Geode> Rgeode=dynamic_cast<osg::Geode*>(rw->readNode(stst).getNode());
    if(!Rgeode)
        std::cout<<"shit Rgeode"<<std::endl;
    return Rgeode;
}

void loadMarkers()
{
	std::fstream fp;
	fp.open("/sdcard/marker_data/sMarker.txt",std::ios::in);
	if(!fp)
	{
		__android_log_write(ANDROID_LOG_ERROR,"failed","load markers");
		fp.close();
		return;
	}
	int numMarkers;
	fp>>numMarkers;
	__android_log_print(ANDROID_LOG_ERROR,"load markers","load markers , num markers: %d",numMarkers);
	int ID;
	int buildTime;
	float x,y,z;
	for(int i=0;i<numMarkers;i++)
	{
		fp>>ID;
		fp>>buildTime;
		std::vector<Point3f> WPts;
		for(int j=0;j<4;j++)
		{
			fp>>x; fp>>y; fp>>z;
			Point3f p;
			p.x=x; p.y=y; p.z=z;
			WPts.push_back(p);
		}
		MS tMS(WPts,ID);
		tMS.buildTime=buildTime;
		mMSM.insert(MSPair(ID,tMS));
		__android_log_print(ANDROID_LOG_ERROR,"load markers","load markers , ID: %d",mMSM[ID].ID);
		for(int j=0;j<4;j++)
		{
			Point3f p=mMSM[ID].WPts.at(j);
			__android_log_print(ANDROID_LOG_ERROR,"load markers","load markers , point: x: %f, y: %f, z:%f",p.x,p.y,p.z);
		}


	}
	fp.close();
	__android_log_write(ANDROID_LOG_INFO,"OK","markers loaded");
}

void loadMarkers(std::string folderPath)
{
	std::fstream fp;
	std::string s("/sMarker.txt");
	std::string temp=folderPath;
	temp=temp+s;
	fp.open(temp.c_str(),std::ios::in);
	if(!fp)
	{
		__android_log_write(ANDROID_LOG_ERROR,"failed","load markers");
		fp.close();
		return;
	}
	int numMarkers;
	fp>>numMarkers;
	__android_log_print(ANDROID_LOG_ERROR,"load markers","load markers , num markers: %d",numMarkers);
	int ID;
	int buildTime;
	float x,y,z;
	for(int i=0;i<numMarkers;i++)
	{
		fp>>ID;
		fp>>buildTime;
		std::vector<Point3f> WPts;
		for(int j=0;j<4;j++)
		{
			fp>>x; fp>>y; fp>>z;
			Point3f p;
			p.x=x; p.y=y; p.z=z;
			WPts.push_back(p);
		}
		MS tMS(WPts,ID);
		tMS.buildTime=buildTime;
		mMSM.insert(MSPair(ID,tMS));
		__android_log_print(ANDROID_LOG_ERROR,"load markers","load markers , ID: %d",mMSM[ID].ID);
		for(int j=0;j<4;j++)
		{
			Point3f p=mMSM[ID].WPts.at(j);
			__android_log_print(ANDROID_LOG_ERROR,"load markers","load markers , point: x: %f, y: %f, z:%f",p.x,p.y,p.z);
		}


	}
	fp.close();
	__android_log_write(ANDROID_LOG_INFO,"OK","markers loaded");
}

JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_OSGRun
  (JNIEnv *, jclass)
{
	osgmain.draw();
}


JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_initOSG
  (JNIEnv *, jclass, jint width, jint height)
{
	osgmain.initOSG(width, height);
}


JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_mouseButtonPressEvent
  (JNIEnv *, jclass, jfloat x, jfloat y, jint button)
{
	osgmain.mouseEventPress(x,y,button);
}


JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_mouseButtonReleaseEvent
  (JNIEnv *, jclass, jfloat x, jfloat y, jint button)
{
	osgmain.mouseEventRelease(x,y,button);
}


JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_mouseMoveEvent
  (JNIEnv *, jclass, jfloat x, jfloat y)
{
	osgmain.mouseEventMotion(x,y);
}


JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_sendRotation
(JNIEnv *, jclass, jfloat axisX, jfloat axisY, jfloat neg_axisZ, jfloat angle)
{
	if(fixVA)
		return;
	if(useSensor)
	{
		osgmain.rotateCameraByRotation(axisX,neg_axisZ, axisY, angle);
		//__android_log_print(ANDROID_LOG_ERROR,"Sensor rotate"," ");
	}
	osgmain.SelfCamInfo.setCamRot(osgmain.MFPCM->getRotation());
}


JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_sendKeyDown
  (JNIEnv *, jclass, jint key)
{
	osgmain.keyboardEventKeyDown(key);
}

JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_sendMainScene
  (JNIEnv *, jclass)
{
	osgmain.startSendMainSceneGeodes=true;
}

JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_toggleDrawLine
  (JNIEnv *, jclass)
{
	osgmain.toggleDrawLine(osgmain.findNewID(),osgmain.currentColor);
}


JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_getNewPt
  (JNIEnv *, jclass)
{
	osg::Vec3 eye=osgmain.MFPCM->getEye();
	osgmain.drawLine(eye);
}
JNIEXPORT jboolean JNICALL Java_nativeFunctions_NativeLib_toggleEdit
  (JNIEnv *, jclass)
{
	jboolean b=false;
#ifdef CHECK_SELECT
	if(!osgmain.NodeSelected())
		return b;
#endif
	osgmain.ToggleEditing();
	b=true;
	return b;
}


JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_EditR
  (JNIEnv *, jclass, jint axis, jfloat amount)
{
	osgmain.edit(Rotation,axis,amount);
}

JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_Edit
  (JNIEnv *, jclass, jint type, jint axis, jfloat amount)
{
	osgmain.edit(type,axis,amount);
}

JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_deleteGeometry
  (JNIEnv *, jclass)
{
#ifdef CHECK_SELECT
	if(!osgmain.NodeSelected())
		return;
#endif
	if(osgmain.getCurrentSelectGeodeID(osgmain.selfC.deleteID))
	{
		osgmain.deleteGeode=true;
		osgmain.deleteCurrentSelectedGeometry();
	}

}

JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_addBox
  (JNIEnv *, jclass)
{
	int ID;
	osg::Vec3 pos;
    osgmain.addBox(ID,pos,osgmain.currentColor);
	//osgmain.selfC.addBox=true;
	//osgmain.selfC.BoxX=pos.x();
	//osgmain.selfC.BoxY=pos.y();
	//osgmain.selfC.BoxZ=pos.z();
	//osgmain.selfC.BoxID=ID;

}

JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_setColor
  (JNIEnv *, jclass, jfloat r, jfloat g, jfloat b)
{
	osgmain.currentColor.set(r,g,b,1.0);
	//osgmain.selfC.selfColorR=r;
	//osgmain.selfC.selfColorG=g;
	//osgmain.selfC.selfColorB=b;
}
JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_changeColor
  (JNIEnv *, jclass)
{
#ifdef CHECK_SELECT
	if(!osgmain.NodeSelected())
		return;
#endif
	if(osgmain.getCurrentSelectGeodeID(osgmain.selfC.changeColorID))
	{
		osgmain.changeColor=true;
		osgmain.changeColorForCurrentSelectGeode();
	}
}

JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_MarkerDetection
(JNIEnv *, jclass, jlong addrRgba, jint height, jint width, jint RectLimit)
{

	if(fixVA)
		return;
	if(osgmain.sending)
		return;
	Mat* pMatRgb=(Mat*)addrRgba;
	IplImage ipl=*pMatRgb;
	const int RL=RectLimit;
	int NumRect=FindRectangle(&ipl,MR,RL);
#ifdef MAIN_DEBUG
	__android_log_print(ANDROID_LOG_INFO,"Num Rect Found:","%d",NumRect);
#endif
#ifdef MAIN_DEBUG
	__android_log_print(ANDROID_LOG_ERROR,"camera mat",":");
	showcamMat();
	__android_log_print(ANDROID_LOG_ERROR,"distort mat",":");
	showDistortionMat();
#endif

		if(NumRect>0)
		{
			DecodeMarker2DCode(&ipl, MR,NumRect);
			MarkerInsight=true;
		}
		else
		{
			MarkerInsight=false;
		}

	if(MarkerInsight)
	{
		useSensor=false;
		if(NumRect==2)
		{
			if(MR[0].MarkerID==MR[1].MarkerID)
				return;
			EstimateMarkerRotation_TranlationVector(mCV.camera_mat,mCV.distort_mat,MR,NumRect);
			for(int i=0;i<NumRect;i++)
			{
				int ID=MR[i].MarkerID;
				if(mMSM.find(ID)==mMSM.end())
					return;
				mMSM[ID].EstimateTR(MR[i].outer_corners,mCV.camera_mat,mCV.distort_mat,mCV.MarkerRot,mCV.MarkerTrans);

				cvRodrigues2(mCV.MarkerRot,mCV.Rot,NULL);

				MatrixToAxisAngle(mCV.Rot,mCV.RotAxis,mCV.RotAngle);
				CV_MAT_ELEM(*mCV.RotAxis,float,0,0)=-CV_MAT_ELEM(*mCV.RotAxis,float,0,0);
				CV_MAT_ELEM(*mCV.RotAxis,float,0,1)=-CV_MAT_ELEM(*mCV.RotAxis,float,0,1);
				CV_MAT_ELEM(*mCV.RotAxis,float,0,2)=-CV_MAT_ELEM(*mCV.RotAxis,float,0,2);
				AxisAngleToMatrix(mCV.Rot,mCV.RotAxis,mCV.RotAngle);
				CV_MAT_ELEM(*mCV.Rot44,float,0,0)=CV_MAT_ELEM(*mCV.Rot,float,0,0); CV_MAT_ELEM(*mCV.Rot44,float,0,1)=CV_MAT_ELEM(*mCV.Rot,float,0,1); CV_MAT_ELEM(*mCV.Rot44,float,0,2)=CV_MAT_ELEM(*mCV.Rot,float,0,2); CV_MAT_ELEM(*mCV.Rot44,float,0,3)=0.0;
				CV_MAT_ELEM(*mCV.Rot44,float,1,0)=CV_MAT_ELEM(*mCV.Rot,float,1,0); CV_MAT_ELEM(*mCV.Rot44,float,1,1)=CV_MAT_ELEM(*mCV.Rot,float,1,1); CV_MAT_ELEM(*mCV.Rot44,float,1,2)=CV_MAT_ELEM(*mCV.Rot,float,1,2); CV_MAT_ELEM(*mCV.Rot44,float,1,3)=0.0;
				CV_MAT_ELEM(*mCV.Rot44,float,2,0)=CV_MAT_ELEM(*mCV.Rot,float,2,0); CV_MAT_ELEM(*mCV.Rot44,float,2,1)=CV_MAT_ELEM(*mCV.Rot,float,2,1); CV_MAT_ELEM(*mCV.Rot44,float,2,2)=CV_MAT_ELEM(*mCV.Rot,float,2,2); CV_MAT_ELEM(*mCV.Rot44,float,2,3)=0.0;
				CV_MAT_ELEM(*mCV.Rot44,float,3,0)=0.0;                             CV_MAT_ELEM(*mCV.Rot44,float,3,1)=0.0;                             CV_MAT_ELEM(*mCV.Rot44,float,3,2)=0.0;                             CV_MAT_ELEM(*mCV.Rot44,float,3,3)=1.0;
				Vtrans.at(i).set(CV_MAT_ELEM(*mCV.MarkerTrans,float,0,0),CV_MAT_ELEM(*mCV.MarkerTrans,float,0,1),CV_MAT_ELEM(*mCV.MarkerTrans,float,0,2));

				//OSG is column major
				MarkerRot.set(CV_MAT_ELEM(*mCV.Rot44,float,0,0),CV_MAT_ELEM(*mCV.Rot44,float,1,0),CV_MAT_ELEM(*mCV.Rot44,float,2,0),0.0,
							  CV_MAT_ELEM(*mCV.Rot44,float,0,1),CV_MAT_ELEM(*mCV.Rot44,float,1,1),CV_MAT_ELEM(*mCV.Rot44,float,2,1),0.0,
							  CV_MAT_ELEM(*mCV.Rot44,float,0,2),CV_MAT_ELEM(*mCV.Rot44,float,1,2),CV_MAT_ELEM(*mCV.Rot44,float,2,2),0.0,
							  0.0,							  0.0, 							  0.0							 ,1.0);
				quats.at(i).set(MarkerRot);
			}
			float dist1=sqrt(MR[0].translation_vector.x*MR[0].translation_vector.x+MR[0].translation_vector.y*MR[0].translation_vector.y+MR[0].translation_vector.z*MR[0].translation_vector.z);
			float dist2=sqrt(MR[1].translation_vector.x*MR[1].translation_vector.x+MR[1].translation_vector.y*MR[1].translation_vector.y+MR[1].translation_vector.z*MR[1].translation_vector.z);
			lerpRatio=dist2/(dist1+dist2);
			osg::Vec3 v;
			v=Vtrans.at(0)*lerpRatio+Vtrans.at(1)*(1.0-lerpRatio);
			osg::Quat q;
			q.slerp(lerpRatio,quats.at(0),quats.at(1));
			MarkerRot.set(q);
			for(int i=0;i<4;i++)
			{
				for(int j=0;j<4;j++)
					CV_MAT_ELEM(*mCV.Rot44,float,j,i)=MarkerRot(i,j);
			}

			CV_MAT_ELEM(*mCV.Trans,float,0,0)=v.x(); CV_MAT_ELEM(*mCV.Trans,float,1,0)=v.y(); CV_MAT_ELEM(*mCV.Trans,float,2,0)=v.z();CV_MAT_ELEM(*mCV.Trans,float,3,0)=1.0;
			cvGEMM(mCV.Rot44,mCV.Trans,1.0,NULL,0.0,mCV.Result,0);
			MarkerRot.preMult(Correction);
			osgmain.setCameraOrientation(MarkerRot);
			float res[3];
			res[0]=CV_MAT_ELEM(*mCV.Result,float,0,0)/CV_MAT_ELEM(*mCV.Result,float,3,0);
			res[1]=CV_MAT_ELEM(*mCV.Result,float,1,0)/CV_MAT_ELEM(*mCV.Result,float,3,0);
			res[2]=CV_MAT_ELEM(*mCV.Result,float,2,0)/CV_MAT_ELEM(*mCV.Result,float,3,0);
			__android_log_print(ANDROID_LOG_INFO,"Marker translation Final","x: %f, y: %f, z: %f",-res[0],-res[1],-res[2]);
			osgmain.setCameraPosition(-res[0],-res[1],-res[2]);
			__android_log_print(ANDROID_LOG_INFO,"Marker ID",": %d and %d",MR[0].MarkerID,MR[1].MarkerID);
		}
		else if(NumRect==1)
		{
			int ID=MR[0].MarkerID;
			MSMap::iterator it;
			it=mMSM.find(ID);
			if(it==mMSM.end())
				return;
			mMSM[ID].EstimateTR(MR[0].outer_corners,mCV.camera_mat,mCV.distort_mat,mCV.MarkerRot,mCV.MarkerTrans);
			cvRodrigues2(mCV.MarkerRot,mCV.Rot,NULL);

			MatrixToAxisAngle(mCV.Rot,mCV.RotAxis,mCV.RotAngle);
			CV_MAT_ELEM(*mCV.RotAxis,float,0,0)=-CV_MAT_ELEM(*mCV.RotAxis,float,0,0);
			CV_MAT_ELEM(*mCV.RotAxis,float,0,1)=-CV_MAT_ELEM(*mCV.RotAxis,float,0,1);
			CV_MAT_ELEM(*mCV.RotAxis,float,0,2)=-CV_MAT_ELEM(*mCV.RotAxis,float,0,2);
			AxisAngleToMatrix(mCV.Rot,mCV.RotAxis,mCV.RotAngle);
			CV_MAT_ELEM(*mCV.Rot44,float,0,0)=CV_MAT_ELEM(*mCV.Rot,float,0,0); CV_MAT_ELEM(*mCV.Rot44,float,0,1)=CV_MAT_ELEM(*mCV.Rot,float,0,1); CV_MAT_ELEM(*mCV.Rot44,float,0,2)=CV_MAT_ELEM(*mCV.Rot,float,0,2); CV_MAT_ELEM(*mCV.Rot44,float,0,3)=0.0;
			CV_MAT_ELEM(*mCV.Rot44,float,1,0)=CV_MAT_ELEM(*mCV.Rot,float,1,0); CV_MAT_ELEM(*mCV.Rot44,float,1,1)=CV_MAT_ELEM(*mCV.Rot,float,1,1); CV_MAT_ELEM(*mCV.Rot44,float,1,2)=CV_MAT_ELEM(*mCV.Rot,float,1,2); CV_MAT_ELEM(*mCV.Rot44,float,1,3)=0.0;
			CV_MAT_ELEM(*mCV.Rot44,float,2,0)=CV_MAT_ELEM(*mCV.Rot,float,2,0); CV_MAT_ELEM(*mCV.Rot44,float,2,1)=CV_MAT_ELEM(*mCV.Rot,float,2,1); CV_MAT_ELEM(*mCV.Rot44,float,2,2)=CV_MAT_ELEM(*mCV.Rot,float,2,2); CV_MAT_ELEM(*mCV.Rot44,float,2,3)=0.0;
			CV_MAT_ELEM(*mCV.Rot44,float,3,0)=0.0;                             CV_MAT_ELEM(*mCV.Rot44,float,3,1)=0.0;                             CV_MAT_ELEM(*mCV.Rot44,float,3,2)=0.0;                             CV_MAT_ELEM(*mCV.Rot44,float,3,3)=1.0;

			CV_MAT_ELEM(*mCV.Trans,float,0,0)=CV_MAT_ELEM(*mCV.MarkerTrans,float,0,0); CV_MAT_ELEM(*mCV.Trans,float,1,0)=CV_MAT_ELEM(*mCV.MarkerTrans,float,0,1); CV_MAT_ELEM(*mCV.Trans,float,2,0)=CV_MAT_ELEM(*mCV.MarkerTrans,float,0,2);CV_MAT_ELEM(*mCV.Trans,float,3,0)=1.0;
			cvGEMM(mCV.Rot44,mCV.Trans,1.0,NULL,0.0,mCV.Result,0);
			MarkerRot.set(CV_MAT_ELEM(*mCV.Rot44,float,0,0),CV_MAT_ELEM(*mCV.Rot44,float,1,0),CV_MAT_ELEM(*mCV.Rot44,float,2,0),0.0,
						  CV_MAT_ELEM(*mCV.Rot44,float,0,1),CV_MAT_ELEM(*mCV.Rot44,float,1,1),CV_MAT_ELEM(*mCV.Rot44,float,2,1),0.0,
						  CV_MAT_ELEM(*mCV.Rot44,float,0,2),CV_MAT_ELEM(*mCV.Rot44,float,1,2),CV_MAT_ELEM(*mCV.Rot44,float,2,2),0.0,
						  0.0,							  0.0, 							  0.0							 ,1.0);
			MarkerRot.preMult(Correction);
			osgmain.setCameraOrientation(MarkerRot);
			float res[3];
			res[0]=CV_MAT_ELEM(*mCV.Result,float,0,0)/CV_MAT_ELEM(*mCV.Result,float,3,0);
			res[1]=CV_MAT_ELEM(*mCV.Result,float,1,0)/CV_MAT_ELEM(*mCV.Result,float,3,0);
			res[2]=CV_MAT_ELEM(*mCV.Result,float,2,0)/CV_MAT_ELEM(*mCV.Result,float,3,0);
			__android_log_print(ANDROID_LOG_INFO,"Marker translation Final","x: %f, y: %f, z: %f",-res[0],-res[1],-res[2]);
			osgmain.setCameraPosition(-res[0],-res[1],-res[2]);
			__android_log_print(ANDROID_LOG_INFO,"Marker ID",": %d",MR[0].MarkerID);
		}
	}
	else
		useSensor=true;

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


#ifndef QUATLERP
	MR=new MarkerRectangle[1];
#else
	MR=new MarkerRectangle[2];
#endif
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
	Correction.set( 1.0,  0.0,  0.0,  0.0,
					0.0, -1.0,  0.0,  0.0,
					0.0,  0.0, -1.0,  0.0,
					0.0,  0.0,  0.0,  1.0);


		loadMarkers(storagePath);
		initGlobalVars();
		FirstPose=true;

	quats.resize(2);
    Vtrans.resize(2);
}

JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_sendSMarker
  (JNIEnv *, jclass)
{
	if(mMSM.size()>0)
		osgmain.selfC.sendMarkers=true;
}

JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_saveSMarker
  (JNIEnv * env, jclass, jstring jfilePath)
{
	std::string path(env->GetStringUTFChars(jfilePath,0));
	std::fstream fp;
	fp.open(path.c_str(),std::ios::out);
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
    		for(int i=0;i<4;i++)
    		{
    			fp<<temp.WPts.at(i).x<<" "<<temp.WPts.at(i).y<<" "<<temp.WPts.at(i).z<<std::endl;
    		}
    		fp<<std::endl;
    	}
    }
	fp.close();
}
JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_saveScene
  (JNIEnv *env, jclass, jstring jfolderPath)
{
	OSGStoragePath.clear();
	OSGStoragePath=(env->GetStringUTFChars(jfolderPath,0));
	std::ofstream fp;
	std::string s=OSGStoragePath;
	s.append("/scene.txt");
	fp.open(s.c_str());
	if(!fp)
	{
		__android_log_print(ANDROID_LOG_ERROR,"save scene","save fail");
	}

	IDGeodeMap::iterator it;
	int NumGeodes=osgmain.mNumGeodeMap.size();
	fp<<NumGeodes<<std::endl;
	for(it=osgmain.mNumGeodeMap.begin();it!=osgmain.mNumGeodeMap.end();it++)
	{
		int ID=it->first;
		fp<<ID<<std::endl;

		writeNode(it->second,fp);
		fp<<std::endl<<"finish"<<std::endl;

	}
	fp.close();
}


JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_loadScene
  (JNIEnv *env, jclass, jstring jfolderPath)
{
	OSGStoragePath.clear();
	OSGStoragePath=(env->GetStringUTFChars(jfolderPath,0));
	std::fstream fp;
	std::string s=OSGStoragePath;
	s.append("/scene.txt");
	fp.open(s.c_str());
	if(!fp)
	{
		__android_log_print(ANDROID_LOG_ERROR,"load scene","load fail");
		return;
	}
	int NumGeodes;
	fp>>NumGeodes;
	for(int i=0;i<NumGeodes;i++)
	{
		int ID;
		fp>>ID;
		ID=osgmain.findNewID();
		osg::ref_ptr<osg::Geode> geode=readNode(fp);
		osgmain.addGeode(geode.get(),ID);
	}
	fp.close();
}

JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_fixViewAngle
  (JNIEnv *, jclass)
{
	fixVA=!fixVA;
}
JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_DrawChara
  (JNIEnv *, jclass)
{
	osgmain.DrawChara=!osgmain.DrawChara;
}
