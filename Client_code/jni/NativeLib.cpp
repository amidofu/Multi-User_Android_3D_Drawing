#include "nativeFunctions_NativeLib.h"
#include "MarkerFinder.h"
#include "mCV.h"
//#include "MarkerlessFunc.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>

using namespace std;
using namespace cv;
#include "osgMain.h"
#include "clientMain.h"
#include "MarkerStorage.h"
osgMain osgmain;
Client client;
CVStuff mCV;
osg::Vec3 pose;
osg::Matrixd Correction;//rotate the coordinate system to make OpenCV coordinate system be consistent with OSG
osg::Matrixd MarkerRot;//pass marker's rotation to OSG
//bool FirstPose;
MarkerRectangle* MR;
MSMap mMSM;//store marker info
std::string storagePath;
std::string OSGStoragePath;
//markerless part
//#define MARKERLESS_PART
//KeyFrame testKF;
/*
double sigma1=1.0;
double sigma2=0.0;
int fastTh=100;
double binTh=100.0;
TermCriteria criteria(TermCriteria::COUNT+TermCriteria::EPS,30,10E-4);
Size GaussSize(5,5);
Size winSize(11,11);
double derivLambda = 0.5;
double minEigTh=10E-4;
int LKFlag=0;
double FundConfi=0.99;
double FundDist=3.0;
int maxLevel=3;


//end markerless
 */
bool MarkerInsight=false;
bool useSensor=false;
#define QUATLERP
#ifdef QUATLERP
std::vector<osg::Quat> quats;
std::vector<osg::Vec3> Vtrans;
float lerpRatio=0.5;
#endif
//#define CHANGE_COLOR
//#define CHECK_SELECT

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

JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_getServerIPByDomainNameAndStartClient
  (JNIEnv *env, jclass, jstring jserverIP)
{
	std::string temp;
	temp=(env->GetStringUTFChars(jserverIP,0));
	struct hostent *tmp = 0;
	tmp=gethostbyname(temp.c_str());
	if (!tmp) {
	      	__android_log_print(ANDROID_LOG_ERROR,"look up failed","%s",hstrerror(h_errno));
	        return;
	}
	 __android_log_print(ANDROID_LOG_INFO,"h_name","%s", tmp->h_name);
     int i = 0;
	 while(tmp->h_aliases[i] != NULL) {
	      	__android_log_print(ANDROID_LOG_INFO,"h_aliases[i]:", "%s", tmp->h_aliases[i]);
	       i++;
	 }

	 std::string IPtemp(inet_ntoa( (struct in_addr) *((struct in_addr *) tmp->h_addr_list[i])));
	 client.serverIPlength=IPtemp.size();
	 free(client.serverIP);
	 client.serverIP=new char[client.serverIPlength];
	 strcpy(client.serverIP,IPtemp.c_str());
	__android_log_print(ANDROID_LOG_INFO,"jni client","server IP:%s",client.serverIP );
	client.startClient();

}

JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_getServerIPandStartClient
  (JNIEnv *env, jclass, jstring jserverIP)
{
	const char * IPtemp=env->GetStringUTFChars(jserverIP,0);
	client.serverIPlength=env->GetStringLength(jserverIP);
	free(client.serverIP);
	client.serverIP=new char[client.serverIPlength];
	strcpy(client.serverIP,IPtemp);

	__android_log_print(ANDROID_LOG_INFO,"jni client","server IP:%s",client.serverIP );
	client.startClient();
}


JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_connectToServer
  (JNIEnv *, jclass)
{
	client.connectServer();
}

JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_sendAndRecvAcceInfoToOther
  (JNIEnv *, jclass)
{
	if(!osgmain.initialized)
			return ;
	//prepare to send local camera info (pose) and other flags
	osgmain.prepareSendCamInfo();
	if(osgmain.sendingMainSceneGeodes||osgmain.recvingMainSceneGeodes)//check if other works are done
	{
		 __android_log_print(ANDROID_LOG_ERROR,"jni client","sending or recving scene, return");
		return;
	}
	int numBytes;
	char* buf;
	size_t infoSize=sizeof(ScamInfo);
	//serialize camera info and flags
	client.storeSimpleStructIntoBuf(buf,osgmain.selfC);
	client.sendMSG(buf,infoSize);
	osgmain.sending=true;
	/*
	//probably not used
	 if(osgmain.startSendNode)
	 {
		 osgmain.SendingGeometry=true;
		 client.sendLongBuffer(osgmain.sendNodeBuffer, osgmain.selfC.numBytesofNode);
		 osgmain.startSendNode=false;
		 osgmain.SendingGeometry=false;
	 }
	 */

	 //send entire scene
	 if(osgmain.startSendMainSceneGeodes)
	 {
		 osgmain.sendingMainSceneGeodes=true;
		 int numBytes;
		 int numChildren=osgmain.mainSceneGeodes->getNumChildren();
		 //send the number of geometries
		 if(!client.sendINT(numChildren))
		 {
			 return;
		 }

		 //send geometries one by one
		 for(int i=0;i<numChildren;i++)
		 {
			 //send geometry
			 osg::Geode* child=osgmain.mainSceneGeodes->getChild(i)->asGeode();
			 osgmain.prepareSendNode(child,numBytes);
			 if(!client.sendINT(numBytes))
			 {
				 return;
			 }
			 client.sendLongBuffer(osgmain.sendNodeBuffer, numBytes);
			 //send the ID of the geometry (Geode)
			 int ID=osgmain.mRNumGeodeMap.find(child)->second;
			 if(!client.sendINT(ID))
			 {
				 return;
			 }
		 }
		 osgmain.sendingMainSceneGeodes=false;
		 osgmain.startSendMainSceneGeodes=false;
	 }

	 //start draw line function in local, send the draw line command and the ID of the line Geode
	 if(osgmain.toggleStartDrawLine&&osgmain.DrawLine)
	 {
		 __android_log_print(ANDROID_LOG_ERROR,"jni server","start draw line");
		 osgmain.toggleStartDrawLine=false;
		 int ID=osgmain.mRNumGeodeMap.find(osgmain.currentLineGeode)->second;
		 client.sendINT(ID);
	 }

	 //turn off draw line mode
	 if(osgmain.toggleStartDrawLine&&!osgmain.DrawLine)
	 {
		 osgmain.toggleStartDrawLine=false;
	 }

	 //wait for next new point for the line
	 if(osgmain.getNewPt)
	 {
		 osgmain.getNewPt=false;
	 }

	 //in geometry transformation mode, send the ID of geometry that is be modified
	 if(osgmain.toggleEditing&&osgmain.editMode)
	 {
		 __android_log_print(ANDROID_LOG_ERROR,"jni clinet","start edit");
		 client.sendINT(osgmain.editingID);
		 osgmain.toggleEditing=false;
	 }

	 //turn off geometry transformation mode
	 if(osgmain.toggleEditing&&!osgmain.editMode)
	 {
		 __android_log_print(ANDROID_LOG_ERROR,"jni clinet","end edit");
		 osgmain.toggleEditing=false;
	 }

	 //send the transformation matrix that is used to transform the geometry
	 if(osgmain.editMode&&osgmain.editHost)
	 {
		 char* buft=new char[4*4*4];
		 osg::Matrixd mat=osgmain.editTransform->getMatrix();
		 osgmain.storeMatrixIntoBuf(buft,mat);
		 client.sendMSG(buft,4*4*4);
	 }

	 //send the geometry ID that is to be deleted
	 if(osgmain.deleteGeode)
	 {
		 client.sendINT(osgmain.selfC.deleteID);
		 osgmain.deleteGeode=false;
	 }

	 //send the marker info
	 if(osgmain.selfC.sendMarkers)
	 {
		 int numMarkers=mMSM.size();
		 client.sendINT(numMarkers);
		 char* b=new char[4+4+4*3*4];//ID, age, 4 corners
		 MSMap::iterator it;
		 int de1;
		 float de2;
		 for(it=mMSM.begin();it!=mMSM.end();it++)
		 {
			 int ID=it->second.ID;
			 int age=mMSM[ID].buildTime;
			 memcpy(b,&ID,4);
			 memcpy(&de1,b,4);
			 __android_log_print(ANDROID_LOG_ERROR,"jni client"," de1: %d",de1);
			 memcpy(b+4,&age,4);
			 memcpy(&de1,b+4,4);
			 __android_log_print(ANDROID_LOG_ERROR,"jni client"," de1: %d",de1);
			 int offset=8;
			 for(int i=0;i<4;i++)
			 {
				 float x=mMSM[ID].WPts.at(i).x;
				 float y=mMSM[ID].WPts.at(i).y;
				 float z=mMSM[ID].WPts.at(i).z;
				 memcpy(b+offset,&x,4);
				 memcpy(&de2,b+offset,4);
				 __android_log_print(ANDROID_LOG_ERROR,"jni client"," de2: %f",de2);
				 offset+=4;
				 memcpy(b+offset,&y,4);
				 memcpy(&de2,b+offset,4);
				 __android_log_print(ANDROID_LOG_ERROR,"jni client"," de2: %f",de2);
				 offset+=4;
				 memcpy(b+offset,&z,4);
				 memcpy(&de2,b+offset,4);
				 __android_log_print(ANDROID_LOG_ERROR,"jni client"," de2: %f",de2);
				 offset+=4;
			 }//finish store one marker
			 client.sendLongBuffer(b, 4+4+4*3*4);
		 }
		 free(b);
		 osgmain.selfC.sendMarkers=false;
	 }
	 //wait for new box
	 if(osgmain.selfC.addBox)
	 {
		 osgmain.selfC.addBox=false;
	 }
	 //wait for new change color command
	 if(osgmain.selfC.changeColor)
	 {
		 osgmain.selfC.changeColor=false;
	 }

	 osgmain.sending=false;
	 ///////////////receiving part start/////////////////

	 if(!osgmain.RecvingGeometry)
	 {
		 client.recvMSG((int)infoSize);

		 //deserialize camera info and flag from other user
		 client.loadSimpleStructFromBuf(&client.buf[0],osgmain.otherC);
		 //decode flags
		 osgmain.readCamInfo();
		 free(buf);

		 //recv entire scene
		 if(osgmain.otherC.startSendMainScene)
		 {
			 __android_log_print(ANDROID_LOG_INFO,"jni client","start recv scene");
			 osgmain.recvingMainSceneGeodes=true;
			 int numChildren;
			 //get the number of geometries in scene
			 client.recvINT(numChildren);
			 __android_log_print(ANDROID_LOG_INFO,"jni client","recv scene num children %d", numChildren);
			 int numBytes;
			 //recv geometry one by one
			 for(int i=0;i<numChildren;i++)
			 {
				 //get the geometry size
				 client.recvINT(numBytes);
				 __android_log_print(ANDROID_LOG_INFO,"jni client","recv scene num bytes %d", numBytes);
				 char* read;
				 //recv one geometry
				 client.recvLongBuffer(read,numBytes);
				 std::stringstream stst;
				 osg::ref_ptr<osgDB::ReaderWriter> rw;
				 stst<<read;
				 rw=osgDB::Registry::instance()->getReaderWriterForExtension("osg");
				 osg::ref_ptr<osg::Geode> Rgeode=dynamic_cast<osg::Geode*>(rw->readNode(stst).getNode());
				 if(Rgeode==0)//recv geometry failed
				 {
					 osgmain.recvingMainSceneGeodes=false;
				 }
				 else
				 {
					 //add geometry to the scene
					 osgmain.mainSceneGeodes->addChild(Rgeode);
				 }
				 //recv the ID of the geometry
				 int ID;
				 client.recvINT(ID);
				 mapIDAndGeode(ID,Rgeode,osgmain.mNumGeodeMap,osgmain.mRNumGeodeMap);
				 free(read);

			 }
			 osgmain.recvingMainSceneGeodes=false;

		 }

		 //recv the draw line mode toggle notice
		 if(osgmain.otherC.drawLine&&osgmain.otherC.toggleStartDrawLine)
		 {
			 //get the ID for the new line we draw
				 int ID;
				 client.recvINT(ID);
				 osg::Vec4 color(osgmain.otherC.selfColorR,osgmain.otherC.selfColorG,osgmain.otherC.selfColorB,1.0);
				 osgmain.toggleDrawLine(ID,color);
				 osgmain.toggleStartDrawLine=false;

		 }
		 else if(!osgmain.otherC.drawLine&&osgmain.otherC.toggleStartDrawLine)//other user finish drawing line
		 {
			 osgmain.toggleDrawLine(0);//finish draw line
			 osgmain.toggleStartDrawLine=false;
		 }
		 else if(osgmain.otherC.drawLine)
		 {
			 if(osgmain.otherC.newPt)//other user add new point for the line he is drawing
			 {
				 osg::Vec3 NewPt(osgmain.otherC.newPtx,osgmain.otherC.newPty,osgmain.otherC.newPtz);
				 osgmain.drawLine(NewPt);
				 osgmain.getNewPt=false;
			 }
		 }

		 //recv the ID of the geometry that is to be modified
		 if(osgmain.otherC.toggleEditing&&osgmain.otherC.editing)
		 {
			 int ID;
			 client.recvINT(ID);
			 osgmain.ToggleEditing(ID);
			 __android_log_print(ANDROID_LOG_ERROR,"jni client","toggle edit ID: %d",ID);
		 }
		 //recv the flag that the other user finished the modification
		 if(osgmain.otherC.toggleEditing&&!osgmain.otherC.editing)
		 {
			 __android_log_print(ANDROID_LOG_ERROR,"jni client","other C finish");
			 osgmain.ToggleEditing();
			 osgmain.toggleEditing=false;
		 }
		 //recv the transformation matrix for the modifying geometry
		 if(osgmain.otherC.editing&&!osgmain.editHost)
		 {
			 __android_log_print(ANDROID_LOG_ERROR,"jni client","in regular sending, read matrix");
			 client.recvMSG(4*4*4);
			 __android_log_print(ANDROID_LOG_ERROR,"jni client","in regular sending, after read matrix");
			 osg::Matrixd mat2;
			 osgmain.loadMatrixFromBuf(client.buf,mat2);
			 osgmain.editTransform->setMatrix(mat2);
		 }
		 //finish geometry transformation
		 if(osgmain.justFinishEdit)
		 {
			 osgmain.justFinishEdit=false;
			 osgmain.editHost=false;
		 }

		 //get notice for deleting geometry
		 if(osgmain.otherC.deleteGeode)
		 {
			 //get ID of deleted geometry and delete it
			 int ID;
			 client.recvINT(ID);
			 osgmain.deleteMainSceneGeode(ID);
		 }

		 //recv marker info
		 if(osgmain.otherC.sendMarkers)
		 {
			 int numMarkers;
			 client.recvINT(numMarkers);
			 mMSM.clear();
			 for(int i=0;i<numMarkers;i++)
			 {
				 char* b;
				 client.recvLongBuffer(b, 4+4+4*3*4);
				 int ID,age;
				 memcpy(&ID,b,4);
				 memcpy(&age,b+4,4);
				 int offset=8;
				 std::vector<Point3f> pts;
				 for(int j=0;j<4;j++)
				 {
					 float x,y,z;
					 memcpy(&x,b+offset,4);
					 offset+=4;
					 memcpy(&y,b+offset,4);
					 offset+=4;
					 memcpy(&z,b+offset,4);
					 offset+=4;
					 Point3f p(x,y,z);
					 pts.push_back(p);
				 }
				 MS M(pts,ID);
				 M.buildTime=age;
				 mMSM.insert(MSPair(ID,M));
				 free(b);
			 }

		 }
		 //recv new box ID and create it
		 if(osgmain.otherC.addBox)
		 {
			 int ID=osgmain.otherC.BoxID;
			 osg::Vec3 pos(osgmain.otherC.BoxX,osgmain.otherC.BoxY,osgmain.otherC.BoxZ);
		     osg::Vec4 color(osgmain.otherC.selfColorR,osgmain.otherC.selfColorG,osgmain.otherC.selfColorB,1.0);
		     osgmain.addBox(pos,ID,color);
		 }
		 //get the ID of geometry that is going to be change color and change the color
		 if(osgmain.otherC.changeColor)
		 {
			 int ID=osgmain.otherC.changeColorID;
			 osg::Vec4 color(osgmain.otherC.selfColorR,osgmain.otherC.selfColorG,osgmain.otherC.selfColorB,1.0);
			 osgmain.changeColorForOneGeode(ID,color);
		 }
	 }
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
	if(useSensor)
		osgmain.rotateCameraByRotation(axisX,neg_axisZ, axisY, angle);
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
	if(!osgmain.NodeSelected())
		return b;
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
	if(!osgmain.NodeSelected())
		return;
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
	osgmain.selfC.addBox=true;
	osgmain.selfC.BoxX=pos.x();
	osgmain.selfC.BoxY=pos.y();
	osgmain.selfC.BoxZ=pos.z();
	osgmain.selfC.BoxID=ID;

}

JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_setColor
  (JNIEnv *, jclass, jfloat r, jfloat g, jfloat b)
{
	osgmain.currentColor.set(r,g,b,1.0);
	osgmain.selfC.selfColorR=r;
	osgmain.selfC.selfColorG=g;
	osgmain.selfC.selfColorB=b;
}
JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_changeColor
  (JNIEnv *, jclass)
{
	if(!osgmain.NodeSelected())
		return;
	if(osgmain.getCurrentSelectGeodeID(osgmain.selfC.changeColorID))
	{
		osgmain.changeColor=true;
		osgmain.changeColorForCurrentSelectGeode();
	}
}
/*
JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_FindFeatures
  (JNIEnv *, jclass, jlong addrGray, jlong addrRgba)
{

    Mat* pMatGr=(Mat*)addrGray;
    Mat* pMatRgb=(Mat*)addrRgba;
    vector<KeyPoint> v;

    FastFeatureDetector detector(50);
    detector.detect(*pMatGr, v);
    for( size_t i = 0; i < v.size(); i++ )
        circle(*pMatRgb, Point(v[i].pt.x, v[i].pt.y), 10, Scalar(255,0,0,255));

}
*/
JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_MarkerDetection
(JNIEnv *, jclass, jlong addrRgba, jint height, jint width, jint RectLimit)
{

	if(osgmain.sending)
		return;
	Mat* pMatRgb=(Mat*)addrRgba;
	IplImage ipl=*pMatRgb;
	const int RL=RectLimit;
	int NumRect=FindRectangle(&ipl,MR,RL);//find possible marker
#ifdef MAIN_DEBUG
	__android_log_print(ANDROID_LOG_INFO,"Num Rect Found:","%d",NumRect);
#endif
#ifdef MAIN_DEBUG
	__android_log_print(ANDROID_LOG_ERROR,"camera mat",":");
	showcamMat();
	__android_log_print(ANDROID_LOG_ERROR,"distort mat",":");
	showDistortionMat();
#endif

//begin
	if(NumRect>0)
	{
		DecodeMarker2DCode(&ipl, MR,NumRect);//check the possible markers and find out marker's info
		MarkerInsight=true;
		useSensor=false;
	}
	else//got 0 marker, use sensor
	{
		MarkerInsight=false;
		useSensor=true;
	}

	if(MarkerInsight)
	{
		if(NumRect==2)//got 2 markers
		{
			//compute camera pose from the 2 markers respectively
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

			//camera pose interpolation
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
			osgmain.setCameraPosition(-res[0],-res[1],-res[2]);
		}
		else if(NumRect==1)//only one marker
		{
			int ID=MR[0].MarkerID;
			MSMap::iterator it;
			it=mMSM.find(ID);
			if(it==mMSM.end())
				return;
			//pose estimation
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
			osgmain.setCameraPosition(-res[0],-res[1],-res[2]);
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
		//FirstPose=true;

#ifdef QUATLERP
	quats.resize(2);
    Vtrans.resize(2);
#endif
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
	//write geomtry and its ID one by one
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
