#include "nativeFunctions_NativeLib.h"
#include "MarkerFinder.h"
#include "mCV.h"
#include "MarkerlessFunc.h"
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
osg::Matrixd Correction;
osg::Matrixd MarkerRot;
bool FirstPose;
MarkerRectangle* MR;
MSMap mMSM;
std::string storagePath;
std::string OSGStoragePath;
//markerless part
//#define MARKERLESS_PART
KeyFrame testKF;
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

bool MarkerInsight=false;
bool useSensor=false;
//end markerless
#define QUATLERP
#ifdef QUATLERP
std::vector<osg::Quat> quats;
std::vector<osg::Vec3> Vtrans;
float lerpRatio=0.5;
#endif
#define CHANGE_COLOR
#define CHECK_SELECT
//#define USE_DOMAIN
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

void MarkerOffset(int MarkerID, float & x, float & y)
{
	x=0.0;
	y=0.0;
	float offset=100.0;
	switch(MarkerID)
	{
	case 1://1<<0:
		x=-offset;
		y=-offset;
		return;

	case 2://1<<1:
		y=-offset;
		return;

	case 4://1<<2:
		x=offset;
		y=-offset;
		return;

	case 8://1<<3:
		x=-offset;
		return;

	case 16://1<<4:
		x=offset;
		return;

	case 32://1<<5:
		x=-offset;
		y=offset;
		return;

	case 64://1<<6:
		y=offset;
		return;

	case 224://1<<7+1<<6+1<<5:
		y=offset;
		x=offset;
		return;
	case 6:
		x=-2.0*offset;
		return;
	case 5:
		x=-2.0*offset;
		y=offset;
		return;
	}
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

	 //__android_log_print(ANDROID_LOG_INFO,"h_addrtype:"," %d - %s", tmp->h_addrtype, addrtype(tmp->h_addrtype));
	 //__android_log_print(ANDROID_LOG_INFO,"h_length:"," %d", tmp->h_length);

	 //i = 0;
	 //while(tmp->h_addr_list[i] != NULL) {
	 //       	__android_log_print(ANDROID_LOG_INFO,"h_addr_list[i]:"," %s", inet_ntoa( (struct in_addr) *((struct in_addr *) tmp->h_addr_list[i])));
	 //i++;
	 //}
	 std::string IPtemp(inet_ntoa( (struct in_addr) *((struct in_addr *) tmp->h_addr_list[i])));
	 client.serverIPlength=IPtemp.size();
	 free(client.serverIP);
	 client.serverIP=new char[client.serverIPlength];
	 strcpy(client.serverIP,IPtemp.c_str());
	__android_log_print(ANDROID_LOG_INFO,"jni client","server IP:%s",client.serverIP );
	//env->ReleaseStringUTFChars(jserverIP,IPtemp);
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
	//env->ReleaseStringUTFChars(jserverIP,IPtemp);
	client.startClient();
}


JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_sendMsg
  (JNIEnv *, jclass)
{
	client.sendMSG();
}


JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_recvMsg
  (JNIEnv *, jclass)
{
	client.recvMSG();
}


JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_connectToServer
  (JNIEnv *, jclass)
{
	client.connectServer();
}


JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_sendAcceleration
 (JNIEnv *, jclass, jfloat x, jfloat y, jfloat z)
{
	//client.m_acceInfo.x=x;
	//client.m_acceInfo.y=y;
	//client.m_acceInfo.z=z;
	//__android_log_print(ANDROID_LOG_INFO,"jni client","loaded acceInfo x:%f, y:%f, z:%f",client.m_acceInfo.x,client.m_acceInfo.y,client.m_acceInfo.z);
}
JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_sendAndRecvAcceInfoToOther
  (JNIEnv *, jclass)
{
	if(!osgmain.initialized)
			return ;
	osgmain.prepareSendCamInfo();
	if(osgmain.startSendNode)
		osgmain.prepareSendNode();
	if(osgmain.sendingMainSceneGeodes||osgmain.recvingMainSceneGeodes)
	{
		 __android_log_print(ANDROID_LOG_ERROR,"jni client","sending or recving scene, return");
		return;
	}
	int numBytes;
	char* buf;
	size_t infoSize=sizeof(ScamInfo);
	client.storeSimpleStructIntoBuf(buf,osgmain.selfC);
	client.sendMSG(buf,infoSize);
	osgmain.sending=true;
	 if(osgmain.startSendNode)
	 {
		 osgmain.SendingGeometry=true;
		 client.sendLongBuffer(osgmain.sendNodeBuffer, osgmain.selfC.numBytesofNode);
		 osgmain.startSendNode=false;
		 osgmain.SendingGeometry=false;
	 }

	 if(osgmain.startSendMainSceneGeodes)
	 {
		 osgmain.sendingMainSceneGeodes=true;
		 int numBytes;
		 int numChildren=osgmain.mainSceneGeodes->getNumChildren();
		 if(!client.sendINT(numChildren))
		 {
			 return;
		 }

		 for(int i=0;i<numChildren;i++)
		 {
			 osg::Geode* child=osgmain.mainSceneGeodes->getChild(i)->asGeode();
			 osgmain.prepareSendNode(child,numBytes);
			 if(!client.sendINT(numBytes))
			 {
				 return;
			 }
			 client.sendLongBuffer(osgmain.sendNodeBuffer, numBytes);
			 int ID=osgmain.mRNumGeodeMap.find(child)->second;
			 if(!client.sendINT(ID))
			 {
				 return;
			 }
		 }
		 osgmain.sendingMainSceneGeodes=false;
		 osgmain.startSendMainSceneGeodes=false;
	 }

	 if(osgmain.toggleStartDrawLine&&osgmain.DrawLine)
	 {
		 __android_log_print(ANDROID_LOG_ERROR,"jni server","start draw line");
		 osgmain.toggleStartDrawLine=false;
		 int ID=osgmain.mRNumGeodeMap.find(osgmain.currentLineGeode)->second;
		 client.sendINT(ID);
	 }

	 if(osgmain.toggleStartDrawLine&&!osgmain.DrawLine)
	 {
		 osgmain.toggleStartDrawLine=false;
	 }

	 if(osgmain.getNewPt)
	 {
		 osgmain.getNewPt=false;
	 }

	 if(osgmain.toggleEditing&&osgmain.editMode)
	 {
		 __android_log_print(ANDROID_LOG_ERROR,"jni clinet","start edit");
		 client.sendINT(osgmain.editingID);
		 osgmain.toggleEditing=false;
	 }

	 if(osgmain.toggleEditing&&!osgmain.editMode)
	 {
		 __android_log_print(ANDROID_LOG_ERROR,"jni clinet","end edit");
		 osgmain.toggleEditing=false;
	 }

	 if(osgmain.editMode&&osgmain.editHost)
	 {
		 char* buft=new char[4*4*4];
		 osg::Matrixd mat=osgmain.editTransform->getMatrix();
		 osgmain.storeMatrixIntoBuf(buft,mat);
		 client.sendMSG(buft,4*4*4);
	 }

	 if(osgmain.deleteGeode)
	 {
		 client.sendINT(osgmain.selfC.deleteID);
		 osgmain.deleteGeode=false;
	 }

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
	 if(osgmain.selfC.addBox)
	 {
		 osgmain.selfC.addBox=false;
	 }
	 if(osgmain.selfC.changeColor)
	 {
		 osgmain.selfC.changeColor=false;
	 }

	 osgmain.sending=false;
	 //////////////////////////////////////////////

	 if(!osgmain.RecvingGeometry)
	 {
		 client.recvMSG((int)infoSize);

		 client.loadSimpleStructFromBuf(&client.buf[0],osgmain.otherC);
		 osgmain.readCamInfo();
		 free(buf);

		 if(osgmain.otherC.startSendNode)
		 {
			 __android_log_print(ANDROID_LOG_ERROR,"jni client"," in start send node");
			 osgmain.RecvingGeometry=true;
			 char *read;
			 int arraylength=osgmain.otherC.numBytesofNode;
			 client.recvLongBuffer(read, arraylength);//not implemented yet
			 osgmain.readRecvNode(read);
			 free(read);
			 osgmain.RecvingGeometry=false;
		 }

		 if(osgmain.otherC.startSendMainScene)
		 {
			 __android_log_print(ANDROID_LOG_INFO,"jni client","start recv scene");
			 osgmain.recvingMainSceneGeodes=true;
			 int numChildren;
			 client.recvINT(numChildren);
			 __android_log_print(ANDROID_LOG_INFO,"jni client","recv scene num children %d", numChildren);
			 int numBytes;
			 for(int i=0;i<numChildren;i++)
			 {
				 client.recvINT(numBytes);
				 __android_log_print(ANDROID_LOG_INFO,"jni client","recv scene num bytes %d", numBytes);
				 char* read;
				 client.recvLongBuffer(read,numBytes);
				 std::stringstream stst;
				 osg::ref_ptr<osgDB::ReaderWriter> rw;
				 stst<<read;
				 rw=osgDB::Registry::instance()->getReaderWriterForExtension("osg");
				 osg::ref_ptr<osg::Geode> Rgeode=dynamic_cast<osg::Geode*>(rw->readNode(stst).getNode());
				 if(Rgeode==0)
				 {
					 osgmain.recvingMainSceneGeodes=false;
				 }
				 else
				 {
					 osgmain.mainSceneGeodes->addChild(Rgeode);
				 }
				 int ID;
				 client.recvINT(ID);
				 mapIDAndGeode(ID,Rgeode,osgmain.mNumGeodeMap,osgmain.mRNumGeodeMap);
				 free(read);

			 }
			 osgmain.recvingMainSceneGeodes=false;

		 }

		 if(osgmain.otherC.drawLine&&osgmain.otherC.toggleStartDrawLine)
		 {
				 int ID;
				 client.recvINT(ID);
#ifndef CHANGE_COLOR
				 osgmain.toggleDrawLine(ID);
#else
				 osg::Vec4 color(osgmain.otherC.selfColorR,osgmain.otherC.selfColorG,osgmain.otherC.selfColorB,1.0);
				 osgmain.toggleDrawLine(ID,color);
#endif
				 osgmain.toggleStartDrawLine=false;

		 }
		 else if(!osgmain.otherC.drawLine&&osgmain.otherC.toggleStartDrawLine)
		 {
			 osgmain.toggleDrawLine(0);//finish draw line
			 osgmain.toggleStartDrawLine=false;
		 }
		 else if(osgmain.otherC.drawLine)
		 {
			 if(osgmain.otherC.newPt)
			 {
				 osg::Vec3 NewPt(osgmain.otherC.newPtx,osgmain.otherC.newPty,osgmain.otherC.newPtz);
				 osgmain.drawLine(NewPt);
				 osgmain.getNewPt=false;
			 }
		 }

		 if(osgmain.otherC.toggleEditing&&osgmain.otherC.editing)
		 {
			 int ID;
			 client.recvINT(ID);
			 osgmain.ToggleEditing(ID);
			 __android_log_print(ANDROID_LOG_ERROR,"jni client","toggle edit ID: %d",ID);
		 }
		 if(osgmain.otherC.toggleEditing&&!osgmain.otherC.editing)
		 {
			 __android_log_print(ANDROID_LOG_ERROR,"jni client","other C finish");
			 //osgmain.finishEditing();
			 osgmain.ToggleEditing();
			 osgmain.toggleEditing=false;
		 }
		 if(osgmain.otherC.editing&&!osgmain.editHost)
		 {
			 __android_log_print(ANDROID_LOG_ERROR,"jni client","in regular sending, read matrix");
			 client.recvMSG(4*4*4);
			 __android_log_print(ANDROID_LOG_ERROR,"jni client","in regular sending, after read matrix");
			 osg::Matrixd mat2;
			 osgmain.loadMatrixFromBuf(client.buf,mat2);
			 osgmain.editTransform->setMatrix(mat2);
		 }
		 if(osgmain.justFinishEdit)
		 {
			 osgmain.justFinishEdit=false;
			 osgmain.editHost=false;
		 }

		 if(osgmain.otherC.deleteGeode)
		 {
			 int ID;
			 client.recvINT(ID);
			 osgmain.deleteMainSceneGeode(ID);
		 }

		 if(osgmain.otherC.sendMarkers)
		 {
			 int numMarkers;
			 client.recvINT(numMarkers);
			 mMSM.clear();
			 //char* b=new char[4+4+4*3*4];
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
		 if(osgmain.otherC.addBox)
		 {
			 int ID=osgmain.otherC.BoxID;
			 osg::Vec3 pos(osgmain.otherC.BoxX,osgmain.otherC.BoxY,osgmain.otherC.BoxZ);
#ifndef CHANGE_COLOR
		     osgmain.addBox(pos,ID);
#else
		     osg::Vec4 color(osgmain.otherC.selfColorR,osgmain.otherC.selfColorG,osgmain.otherC.selfColorB,1.0);
		     osgmain.addBox(pos,ID,color);
#endif
		 }
		 if(osgmain.otherC.changeColor)
		 {
			 int ID=osgmain.otherC.changeColorID;
			 osg::Vec4 color(osgmain.otherC.selfColorR,osgmain.otherC.selfColorG,osgmain.otherC.selfColorB,1.0);
			 osgmain.changeColorForOneGeode(ID,color);
		 }
	 }
	 //free(buf);

	 //client.updateTrack();
	 //if((client.translation[0]!=0.0f)||(client.translation[1]!=0.0f),(client.translation[2]!=0.0f))
	 //	 __android_log_print(ANDROID_LOG_ERROR,"jni client","translation: %f, %f, %f",client.translation[0],client.translation[1],client.translation[2]);
	 //osgmain.moveCameraByDisplacement(client.translation[0],client.translation[1],client.translation[2]);
}


JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_resetRotation
  (JNIEnv *, jclass)
{

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
	//osgmain.rotateCameraByRotation(axisX,neg_axisZ, axisY, angle);
	if(useSensor)
		osgmain.rotateCameraByRotation(axisX,neg_axisZ, axisY, angle);
	osgmain.SelfCamInfo.setCamRot(osgmain.MFPCM->getRotation());
}

JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_sendKeyDown
(JNIEnv *, jclass, jint key)
{
	osgmain.keyboardEventKeyDown(key);
}


JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_sendGeometry
  (JNIEnv *, jclass)
{
	if(osgmain.picker->NodeForSendSelected)
	{
		osgmain.startSendNode=true;
		osgmain.picker->NodeForSendSelected=false;
	}
}

JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_recvGeometry
  (JNIEnv *, jclass)
{
	/*
	size_t infoSize=72;
	 if(!osgmain.RecvingGeometry)
		 {
			 client.recvMSG((int)infoSize);

			 tpl_node *tnu;
			 tnu = tpl_map(CamInfoFmt, &osgmain.otherC );
			 tpl_load( tnu, TPL_MEM|TPL_EXCESS_OK, client.buf,bufferSize);
			 tpl_unpack(tnu,0);
			 osgmain.readCamInfo();
			 //__android_log_print(ANDROID_LOG_INFO,"jni client","loaded acceInfo x:%f, y:%f, z:%f",client.other_acceInfo.x,client.other_acceInfo.y,client.other_acceInfo.z);
			 __android_log_print(ANDROID_LOG_INFO,"jni client","send: %f,%f,%f",osgmain.otherC.posx,osgmain.otherC.posy,osgmain.otherC.posz);
			 //free(buf);
			 if(osgmain.otherC.startSendNode)
			 {
				 osgmain.RecvingGeometry=true;
				 char *read;
				 int arraylength=osgmain.otherC.numBytesofNode;
				 client.recvLongBuffer(read, arraylength);//not implemented yet
				 osgmain.readRecvNode(read);
				 free(read);
				 osgmain.RecvingGeometry=false;
			 }
		 }
		 */
}
JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_sendMainScene
  (JNIEnv *, jclass)
{
	osgmain.startSendMainSceneGeodes=true;
}

JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_toggleDrawLine
  (JNIEnv *, jclass)
{
#ifndef CHANGE_COLOR
	osgmain.toggleDrawLine(osgmain.findNewID());
#else
	osgmain.toggleDrawLine(osgmain.findNewID(),osgmain.currentColor);
#endif
	//osgmain.toggleDrawLine(osgmain.mNumGeodeMap.size());
}


JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_getNewPt
  (JNIEnv *, jclass)
{
	//__android_log_print(ANDROID_LOG_ERROR,"jni client","press new point");
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
#ifndef CHANGE_COLOR
	osgmain.addBox(ID,pos);
#else
    osgmain.addBox(ID,pos,osgmain.currentColor);
#endif
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

JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_MarkerDetection
(JNIEnv *, jclass, jlong addrRgba, jint height, jint width, jint RectLimit)
{

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

//begin
	if(NumRect>0)
	{
		DecodeMarker2DCode(&ipl, MR,NumRect);
		MarkerInsight=true;
#ifndef MARKERLESS_PART
		useSensor=false;
#endif
	}
	else
	{
		MarkerInsight=false;
#ifndef MARKERLESS_PART
		useSensor=true;
#endif
		//return;
	}
#ifndef QUATLERP
	if(MarkerInsight)
	{

		int ID=MR[0].MarkerID;
		if(MR[0].Direction==-1||ID==-1)
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
		//end
	}
#else
	if(MarkerInsight)
	{
		if(NumRect==2)
		{
			EstimateMarkerRotation_TranlationVector(mCV.camera_mat,mCV.distort_mat,MR,NumRect);
			for(int i=0;i<NumRect;i++)
			{
				//__android_log_print(ANDROID_LOG_INFO,"OK","OK 1 %d",i);
				int ID=MR[i].MarkerID;
				if(mMSM.find(ID)==mMSM.end())
					return;
				//__android_log_print(ANDROID_LOG_INFO,"OK","OK 2 %d, ID: %d",i,ID);
				mMSM[ID].EstimateTR(MR[i].outer_corners,mCV.camera_mat,mCV.distort_mat,mCV.MarkerRot,mCV.MarkerTrans);

				cvRodrigues2(mCV.MarkerRot,mCV.Rot,NULL);

				MatrixToAxisAngle(mCV.Rot,mCV.RotAxis,mCV.RotAngle);
				CV_MAT_ELEM(*mCV.RotAxis,float,0,0)=-CV_MAT_ELEM(*mCV.RotAxis,float,0,0);
				CV_MAT_ELEM(*mCV.RotAxis,float,0,1)=-CV_MAT_ELEM(*mCV.RotAxis,float,0,1);
				CV_MAT_ELEM(*mCV.RotAxis,float,0,2)=-CV_MAT_ELEM(*mCV.RotAxis,float,0,2);
				AxisAngleToMatrix(mCV.Rot,mCV.RotAxis,mCV.RotAngle);
				//__android_log_print(ANDROID_LOG_INFO,"OK","OK 3 %d",i);
				CV_MAT_ELEM(*mCV.Rot44,float,0,0)=CV_MAT_ELEM(*mCV.Rot,float,0,0); CV_MAT_ELEM(*mCV.Rot44,float,0,1)=CV_MAT_ELEM(*mCV.Rot,float,0,1); CV_MAT_ELEM(*mCV.Rot44,float,0,2)=CV_MAT_ELEM(*mCV.Rot,float,0,2); CV_MAT_ELEM(*mCV.Rot44,float,0,3)=0.0;
				CV_MAT_ELEM(*mCV.Rot44,float,1,0)=CV_MAT_ELEM(*mCV.Rot,float,1,0); CV_MAT_ELEM(*mCV.Rot44,float,1,1)=CV_MAT_ELEM(*mCV.Rot,float,1,1); CV_MAT_ELEM(*mCV.Rot44,float,1,2)=CV_MAT_ELEM(*mCV.Rot,float,1,2); CV_MAT_ELEM(*mCV.Rot44,float,1,3)=0.0;
				CV_MAT_ELEM(*mCV.Rot44,float,2,0)=CV_MAT_ELEM(*mCV.Rot,float,2,0); CV_MAT_ELEM(*mCV.Rot44,float,2,1)=CV_MAT_ELEM(*mCV.Rot,float,2,1); CV_MAT_ELEM(*mCV.Rot44,float,2,2)=CV_MAT_ELEM(*mCV.Rot,float,2,2); CV_MAT_ELEM(*mCV.Rot44,float,2,3)=0.0;
				CV_MAT_ELEM(*mCV.Rot44,float,3,0)=0.0;                             CV_MAT_ELEM(*mCV.Rot44,float,3,1)=0.0;                             CV_MAT_ELEM(*mCV.Rot44,float,3,2)=0.0;                             CV_MAT_ELEM(*mCV.Rot44,float,3,3)=1.0;

				//__android_log_print(ANDROID_LOG_INFO,"OK","OK 4 %d",i);
				//CV_MAT_ELEM(*mCV.Trans,float,0,0)=CV_MAT_ELEM(*mCV.MarkerTrans,float,0,0); CV_MAT_ELEM(*mCV.Trans,float,1,0)=CV_MAT_ELEM(*mCV.MarkerTrans,float,0,1); CV_MAT_ELEM(*mCV.Trans,float,2,0)=CV_MAT_ELEM(*mCV.MarkerTrans,float,0,2);CV_MAT_ELEM(*mCV.Trans,float,3,0)=1.0;
				//cvGEMM(mCV.Rot44,mCV.Trans,1.0,NULL,0.0,mCV.Result,0);
				Vtrans.at(i).set(CV_MAT_ELEM(*mCV.MarkerTrans,float,0,0),CV_MAT_ELEM(*mCV.MarkerTrans,float,0,1),CV_MAT_ELEM(*mCV.MarkerTrans,float,0,2));
				//__android_log_print(ANDROID_LOG_INFO,"OK","OK 4-2 %d",i);
				//OSG is column major
				MarkerRot.set(CV_MAT_ELEM(*mCV.Rot44,float,0,0),CV_MAT_ELEM(*mCV.Rot44,float,1,0),CV_MAT_ELEM(*mCV.Rot44,float,2,0),0.0,
							  CV_MAT_ELEM(*mCV.Rot44,float,0,1),CV_MAT_ELEM(*mCV.Rot44,float,1,1),CV_MAT_ELEM(*mCV.Rot44,float,2,1),0.0,
							  CV_MAT_ELEM(*mCV.Rot44,float,0,2),CV_MAT_ELEM(*mCV.Rot44,float,1,2),CV_MAT_ELEM(*mCV.Rot44,float,2,2),0.0,
							  0.0,							  0.0, 							  0.0							 ,1.0);
				//MarkerRot.preMult(Correction);
				//__android_log_print(ANDROID_LOG_INFO,"OK","OK 5 %d",i);
				quats.at(i).set(MarkerRot);
				//__android_log_print(ANDROID_LOG_INFO,"OK","OK 6 %d",i);
			}
			float dist1=sqrt(MR[0].translation_vector.x*MR[0].translation_vector.x+MR[0].translation_vector.y*MR[0].translation_vector.y+MR[0].translation_vector.z*MR[0].translation_vector.z);
			float dist2=sqrt(MR[1].translation_vector.x*MR[1].translation_vector.x+MR[1].translation_vector.y*MR[1].translation_vector.y+MR[1].translation_vector.z*MR[1].translation_vector.z);
			lerpRatio=dist2/(dist1+dist2);
			osg::Vec3 v;
			v=Vtrans.at(0)*lerpRatio+Vtrans.at(1)*(1.0-lerpRatio);
			osg::Quat q;
			//__android_log_print(ANDROID_LOG_INFO,"OK","OK 7");
			q.slerp(lerpRatio,quats.at(0),quats.at(1));
			MarkerRot.set(q);
			//__android_log_print(ANDROID_LOG_INFO,"OK","OK 8");
			for(int i=0;i<4;i++)
			{
				for(int j=0;j<4;j++)
					CV_MAT_ELEM(*mCV.Rot44,float,j,i)=MarkerRot(i,j);
			}

			CV_MAT_ELEM(*mCV.Trans,float,0,0)=v.x(); CV_MAT_ELEM(*mCV.Trans,float,1,0)=v.y(); CV_MAT_ELEM(*mCV.Trans,float,2,0)=v.z();CV_MAT_ELEM(*mCV.Trans,float,3,0)=1.0;
			cvGEMM(mCV.Rot44,mCV.Trans,1.0,NULL,0.0,mCV.Result,0);
			MarkerRot.preMult(Correction);
			osgmain.setCameraOrientation(MarkerRot);
			//__android_log_print(ANDROID_LOG_INFO,"OK","OK 9");
			float res[3];
			res[0]=CV_MAT_ELEM(*mCV.Result,float,0,0)/CV_MAT_ELEM(*mCV.Result,float,3,0);
			res[1]=CV_MAT_ELEM(*mCV.Result,float,1,0)/CV_MAT_ELEM(*mCV.Result,float,3,0);
			res[2]=CV_MAT_ELEM(*mCV.Result,float,2,0)/CV_MAT_ELEM(*mCV.Result,float,3,0);
			//__android_log_print(ANDROID_LOG_INFO,"Marker translation Final","x: %f, y: %f, z: %f",-res[0],-res[1],-res[2]);
			osgmain.setCameraPosition(-res[0],-res[1],-res[2]);
			//__android_log_print(ANDROID_LOG_INFO,"Marker ID",": %d and %d",MR[0].MarkerID,MR[1].MarkerID);
		}
		else if(NumRect==1)
		{
			int ID=MR[0].MarkerID;
			//__android_log_print(ANDROID_LOG_INFO,"OK 2","OK 1, ID: %d",ID);
			MSMap::iterator it;
			it=mMSM.find(ID);
			if(it==mMSM.end())
				return;
			//__android_log_print(ANDROID_LOG_INFO,"OK 2","OK 1-1");
			mMSM[ID].EstimateTR(MR[0].outer_corners,mCV.camera_mat,mCV.distort_mat,mCV.MarkerRot,mCV.MarkerTrans);

			//__android_log_print(ANDROID_LOG_INFO,"OK 2","OK 2");
			cvRodrigues2(mCV.MarkerRot,mCV.Rot,NULL);

			MatrixToAxisAngle(mCV.Rot,mCV.RotAxis,mCV.RotAngle);
			CV_MAT_ELEM(*mCV.RotAxis,float,0,0)=-CV_MAT_ELEM(*mCV.RotAxis,float,0,0);
			CV_MAT_ELEM(*mCV.RotAxis,float,0,1)=-CV_MAT_ELEM(*mCV.RotAxis,float,0,1);
			CV_MAT_ELEM(*mCV.RotAxis,float,0,2)=-CV_MAT_ELEM(*mCV.RotAxis,float,0,2);
			AxisAngleToMatrix(mCV.Rot,mCV.RotAxis,mCV.RotAngle);

			//__android_log_print(ANDROID_LOG_INFO,"OK 2","OK 3");
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
			//__android_log_print(ANDROID_LOG_INFO,"OK 2","OK 4");
			float res[3];
			res[0]=CV_MAT_ELEM(*mCV.Result,float,0,0)/CV_MAT_ELEM(*mCV.Result,float,3,0);
			res[1]=CV_MAT_ELEM(*mCV.Result,float,1,0)/CV_MAT_ELEM(*mCV.Result,float,3,0);
			res[2]=CV_MAT_ELEM(*mCV.Result,float,2,0)/CV_MAT_ELEM(*mCV.Result,float,3,0);
			//__android_log_print(ANDROID_LOG_INFO,"Marker translation Final","x: %f, y: %f, z: %f",-res[0],-res[1],-res[2]);
			osgmain.setCameraPosition(-res[0],-res[1],-res[2]);
			//__android_log_print(ANDROID_LOG_INFO,"Marker ID",": %d",MR[0].MarkerID);
		}
	}
#endif





#ifdef MARKERLESS_PART
	   if(testKF.valid)
	    	__android_log_print(ANDROID_LOG_INFO,"numImgPt","test KF, numImgFeatures: %d", testKF.imgFeatures.size());
	    cv::Mat gray;
	    cv::cvtColor(*pMatRgb,gray,CV_BGR2GRAY);
	    /*
	   if(MarkerInsight)
	   {
		   useSensor=false;
		   if(testKF.valid)
		   {
		  	  	std::vector<Point3f> select3f;
		  	   	std::vector<Point2f> select2f;
		  	   	bool tracked=trackFeatureInKFByNewOpticalflow(testKF,gray,select3f,select2f,mCV,binTh,GaussSize,sigma1,sigma2,winSize,maxLevel,criteria,derivLambda , LKFlag,minEigTh,FundConfi, FundDist);
		  	   	if(!tracked)
		  	   	{
		  	   		__android_log_print(ANDROID_LOG_ERROR,"No marker, not tracked"," No");
		  	   		return;
		  	   	}
		  	}
		  	else
		  	  	__android_log_print(ANDROID_LOG_INFO,"KF not valid","No");
	   }
	   */
	   if(MarkerInsight&&!testKF.valid)
	   {
		   useSensor=false;
		    if(mCV.first)
		    {
		    	std::vector<KeyPoint> keypoints_1;
			  	__android_log_write(ANDROID_LOG_INFO,"create Marker KF first","create Marker KF first start");
			   	keypoints_1.clear();
				cv::FAST(gray,keypoints_1,mCV.FASTthreshold,mCV.nonMax);
			 	mCV.previous=gray.clone();
			  	mCV.LKPts1.clear();
			  	for(int i=0;i<keypoints_1.size();i++)
			  	{
			  		mCV.LKPts1.push_back(keypoints_1.at(i).pt);
			  		//cv::circle(*pMatRgb,mCV.LKPts1.at(i),3,CV_RGB(0,255,0));
			  	}
			  	mCV.first=false;
			    CV_MAT_ELEM(*mCV.rots.at(0),float,0,0)=CV_MAT_ELEM(*mCV.MarkerRot,float,0,0);
			    CV_MAT_ELEM(*mCV.rots.at(0),float,0,1)=CV_MAT_ELEM(*mCV.MarkerRot,float,0,1);
			    CV_MAT_ELEM(*mCV.rots.at(0),float,0,2)=CV_MAT_ELEM(*mCV.MarkerRot,float,0,2);
			    CV_MAT_ELEM(*mCV.transs.at(0),float,0,0)=CV_MAT_ELEM(*mCV.MarkerTrans,float,0,0);
			    CV_MAT_ELEM(*mCV.transs.at(0),float,0,1)=CV_MAT_ELEM(*mCV.MarkerTrans,float,0,1);
			    CV_MAT_ELEM(*mCV.transs.at(0),float,0,2)=CV_MAT_ELEM(*mCV.MarkerTrans,float,0,2);
			    __android_log_write(ANDROID_LOG_INFO,"create Marker KF first","done");

			}
			else//second img
			{
			  	  //disable testKF
			   	  if(testKF.valid)
			 		 	return;

			   	  if(mCV.LKPts1.size()<1)
			   	  {
			        mCV.first=true;
			        return;
			   	  }
			   	 __android_log_write(ANDROID_LOG_INFO,"OK1","create Marker KF second");
			     float X[3],XX[3];
			     for(int i=0;i<3;i++)
			     {
			    	 X[i]=CV_MAT_ELEM(*mCV.rots.at(0),float,0,i);
			     }
			     int selectR=0;
			     XX[0]=CV_MAT_ELEM(*mCV.MarkerTrans,float,0,0);
			     XX[1]=CV_MAT_ELEM(*mCV.MarkerTrans,float,0,1);
			     XX[2]=CV_MAT_ELEM(*mCV.MarkerTrans,float,0,2);
			     float dist=0.0f;
			     for(int i=0;i<3;i++)
			     {
			    	 float temp=X[i]-XX[i];
			    	 dist+=temp*temp;
			     }
			     if(50.0f>sqrt(dist))
			    	 return;

			     mCV.LKPts2.clear();
			     mCV.LKPts2=mCV.LKPts1;
			     if(!newLKTrack(mCV.previous,gray,mCV.LKPts1,mCV.LKPts2,binTh,GaussSize,sigma1,sigma2,winSize,maxLevel,criteria,derivLambda,LKFlag,minEigTh,FundConfi,FundDist))
			     {
			    	 __android_log_write(ANDROID_LOG_INFO,"fail","track for first KF");
			    	 mCV.first=true;
			    	 testKF.valid=false;
			    	 return;
			     }
			     else
			    	 __android_log_print(ANDROID_LOG_INFO,"Ok","track for first KF, num pts tracked: %d, previous size: %d",mCV.LKPts2.size(),mCV.LKPts1.size());
			     std::vector<Point3f> sWPts;
			     std::vector<Point2f> matchPts;

				    CV_MAT_ELEM(*mCV.rots.at(1),float,0,0)=CV_MAT_ELEM(*mCV.MarkerRot,float,0,0);
				    CV_MAT_ELEM(*mCV.rots.at(1),float,0,1)=CV_MAT_ELEM(*mCV.MarkerRot,float,0,1);
				    CV_MAT_ELEM(*mCV.rots.at(1),float,0,2)=CV_MAT_ELEM(*mCV.MarkerRot,float,0,2);
				    CV_MAT_ELEM(*mCV.transs.at(1),float,0,0)=CV_MAT_ELEM(*mCV.MarkerTrans,float,0,0);
				    CV_MAT_ELEM(*mCV.transs.at(1),float,0,1)=CV_MAT_ELEM(*mCV.MarkerTrans,float,0,1);
				    CV_MAT_ELEM(*mCV.transs.at(1),float,0,2)=CV_MAT_ELEM(*mCV.MarkerTrans,float,0,2);

			     int numGoodPts=newTriangulationFrom2Views(mCV.camera_mat,mCV.distort_mat,mCV.rots.at(0),mCV.rots.at(1),mCV.transs.at(0),mCV.transs.at(1),mCV.LKPts1,mCV.LKPts2,matchPts,sWPts);
			     if(0==numGoodPts)
			     {
			       	 __android_log_write(ANDROID_LOG_ERROR,"fail","newTriangulation");
			    	 mCV.first=true;
			    	 testKF.valid=false;
			    	 return;
			     }
			     else
			     {
			       	 __android_log_print(ANDROID_LOG_INFO,"OK","newTriangulation, match size: %d",matchPts.size());
			       	 mCV.LKPts2.clear();
			       	 mCV.LKPts2=matchPts;
			     }

				    CV_MAT_ELEM(*mCV.rots.at(0),float,0,0)=CV_MAT_ELEM(*mCV.MarkerRot,float,0,0);
				    CV_MAT_ELEM(*mCV.rots.at(0),float,0,1)=CV_MAT_ELEM(*mCV.MarkerRot,float,0,1);
				    CV_MAT_ELEM(*mCV.rots.at(0),float,0,2)=CV_MAT_ELEM(*mCV.MarkerRot,float,0,2);
				    CV_MAT_ELEM(*mCV.transs.at(0),float,0,0)=CV_MAT_ELEM(*mCV.MarkerTrans,float,0,0);
				    CV_MAT_ELEM(*mCV.transs.at(0),float,0,1)=CV_MAT_ELEM(*mCV.MarkerTrans,float,0,1);
				    CV_MAT_ELEM(*mCV.transs.at(0),float,0,2)=CV_MAT_ELEM(*mCV.MarkerTrans,float,0,2);
			     __android_log_write(ANDROID_LOG_INFO,"done","Marker KF Update First Pose");

			     mCV.LKPts1.clear();
			     for(int i=0;i<mCV.LKPts2.size();i++)
			     {
			    		// cv::circle(*pMatRgb,mCV.LKPts2.at(i),3,CV_RGB(255,0,0));
			    		 //if(mCV.status.at(i))
			    			 mCV.LKPts1.push_back(mCV.LKPts2.at(i));
			      }
			      __android_log_write(ANDROID_LOG_INFO,"OK","Draw Track");

			     if(numGoodPts>mCV.numGoodPtsThreshold)
			     {
			    	 __android_log_write(ANDROID_LOG_INFO,"done1","create Marker KF add KFnnn");
			    	 //KFM.addKeyFrame(matchPts,pt3D,gray);
			    	 testKF.updateKeyFrame(matchPts,sWPts,gray);
			    	 testKF.valid=true;
			    	 __android_log_print(ANDROID_LOG_ERROR,"tesf KF num Features",": %d, matchPts size: %d", testKF.imgFeatures.size(),matchPts.size());
			    	 __android_log_write(ANDROID_LOG_INFO,"done2","create Marker KF add KFnnn");
			      }
			     else
			     {
			    	 mCV.first=true;
			    	 testKF.valid=false;
			    	 return;
			     }


			    	 mCV.previous=gray.clone();
			    	 __android_log_write(ANDROID_LOG_INFO,"Ok","all 3D triangulate");
			}
	   }
	   else if(!MarkerInsight&&testKF.valid)
	   {
		   useSensor=false;
	    	std::vector<Point3f> select3f;
	    	std::vector<Point2f> select2f;
	    	Mat backupImg=testKF.image.clone();
	    	std::vector<Point3f> backup3Dp=testKF.sWPts;
	    	std::vector<Point2f> backup2Dp=testKF.imgFeatures;
	    	bool tracked=trackFeatureInKFByNewOpticalflow(testKF,gray,select3f,select2f,mCV,binTh,GaussSize,sigma1,sigma2,winSize,maxLevel,criteria,derivLambda , LKFlag,minEigTh,FundConfi, FundDist);
	    	if(!tracked)
	    	{
	    		__android_log_print(ANDROID_LOG_ERROR,"tracking without marker"," failed");
	    		useSensor=true;

	    	}
	    	else
	    	{
	    	//drawProjPtsFromKFPose(select3f,select2f,mCV, pMatRgb);
	    	/*
	    	if(!drawProjPtsFromKFPoseAndRemoveBadProjForKF(select3f,select2f,mCV,pMatRgb,testKF))
	    	{
	    		testKF.updateKeyFrame(backup2Dp,backup3Dp,backupImg);
	    		return;
	    	}
	    	*/

	    	cvRodrigues2(mCV.KFRot,mCV.Rot,NULL);

	    	MatrixToAxisAngle(mCV.Rot,mCV.RotAxis,mCV.RotAngle);
	    	CV_MAT_ELEM(*mCV.RotAxis,float,0,0)=-CV_MAT_ELEM(*mCV.RotAxis,float,0,0);
	    	CV_MAT_ELEM(*mCV.RotAxis,float,0,1)=-CV_MAT_ELEM(*mCV.RotAxis,float,0,1);
	    	CV_MAT_ELEM(*mCV.RotAxis,float,0,2)=-CV_MAT_ELEM(*mCV.RotAxis,float,0,2);
	    	AxisAngleToMatrix(mCV.Rot,mCV.RotAxis,mCV.RotAngle);

	    	CV_MAT_ELEM(*mCV.Rot44,float,0,0)=CV_MAT_ELEM(*mCV.Rot,float,0,0); CV_MAT_ELEM(*mCV.Rot44,float,0,1)=CV_MAT_ELEM(*mCV.Rot,float,0,1); CV_MAT_ELEM(*mCV.Rot44,float,0,2)=CV_MAT_ELEM(*mCV.Rot,float,0,2); CV_MAT_ELEM(*mCV.Rot44,float,0,3)=0.0;
	    	CV_MAT_ELEM(*mCV.Rot44,float,1,0)=CV_MAT_ELEM(*mCV.Rot,float,1,0); CV_MAT_ELEM(*mCV.Rot44,float,1,1)=CV_MAT_ELEM(*mCV.Rot,float,1,1); CV_MAT_ELEM(*mCV.Rot44,float,1,2)=CV_MAT_ELEM(*mCV.Rot,float,1,2); CV_MAT_ELEM(*mCV.Rot44,float,1,3)=0.0;
	    	CV_MAT_ELEM(*mCV.Rot44,float,2,0)=CV_MAT_ELEM(*mCV.Rot,float,2,0); CV_MAT_ELEM(*mCV.Rot44,float,2,1)=CV_MAT_ELEM(*mCV.Rot,float,2,1); CV_MAT_ELEM(*mCV.Rot44,float,2,2)=CV_MAT_ELEM(*mCV.Rot,float,2,2); CV_MAT_ELEM(*mCV.Rot44,float,2,3)=0.0;
	    	CV_MAT_ELEM(*mCV.Rot44,float,3,0)=0.0;                             CV_MAT_ELEM(*mCV.Rot44,float,3,1)=0.0;                             CV_MAT_ELEM(*mCV.Rot44,float,3,2)=0.0;                             CV_MAT_ELEM(*mCV.Rot44,float,3,3)=1.0;
	    	CV_MAT_ELEM(*mCV.Trans,float,0,0)=CV_MAT_ELEM(*mCV.KFTrans,float,0,0); CV_MAT_ELEM(*mCV.Trans,float,1,0)=CV_MAT_ELEM(*mCV.KFTrans,float,0,1); CV_MAT_ELEM(*mCV.Trans,float,2,0)=CV_MAT_ELEM(*mCV.KFTrans,float,0,2);CV_MAT_ELEM(*mCV.Trans,float,3,0)=1.0;
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
	    	__android_log_print(ANDROID_LOG_INFO,"KF translation Final","x: %f, y: %f, z: %f",-res[0],-res[1],-res[2]);
	    	osgmain.setCameraPosition(-res[0],-res[1],-res[2]);


	    	if(testKF.imgFeatures.size()<mCV.reFindKFThreshold)
	    	{
	    		__android_log_print(ANDROID_LOG_INFO,"testKF","cleared");
	    		mCV.first=true;
	    		testKF.clearPts();
	    	}
	    	}
	   }
	   else
	   {
		 useSensor=true;
	   }



#endif
}

JNIEXPORT void JNICALL Java_nativeFunctions_NativeLib_InitializedOpenCVGlobalVar
  (JNIEnv *, jclass, jint width, jint height)
{
	MR=new MarkerRectangle[1];
	setMarkerDetectScreenResolution(width,height);

	mCV.camera_mat=cvCreateMat(3,3,CV_32FC1);
	mCV.distort_mat=cvCreateMat(1,4,CV_32FC1);
	//HTC
	/*
	CV_MAT_ELEM(*mCV.camera_mat,float,0,0)=488.418243;
	CV_MAT_ELEM(*mCV.camera_mat,float,0,1)=0.0;
	CV_MAT_ELEM(*mCV.camera_mat,float,0,2)=240.47612;
	CV_MAT_ELEM(*mCV.camera_mat,float,1,0)=0.0;
	CV_MAT_ELEM(*mCV.camera_mat,float,1,1)=489.925385;
	CV_MAT_ELEM(*mCV.camera_mat,float,1,2)=200.961441;
	CV_MAT_ELEM(*mCV.camera_mat,float,2,0)=0.0;
	CV_MAT_ELEM(*mCV.camera_mat,float,2,1)=0.0;
	CV_MAT_ELEM(*mCV.camera_mat,float,2,2)=1.0;



	CV_MAT_ELEM(*mCV.distort_mat,float,0,0)=0.168246;
	CV_MAT_ELEM(*mCV.distort_mat,float,0,1)=-0.763412;
	CV_MAT_ELEM(*mCV.distort_mat,float,0,2)=0.020337;
	CV_MAT_ELEM(*mCV.distort_mat,float,0,3)=-0.004868;
	*/

	//S2 320x240
	CV_MAT_ELEM(*mCV.camera_mat,float,0,0)=271.926147;
	CV_MAT_ELEM(*mCV.camera_mat,float,0,1)=0.0;
	CV_MAT_ELEM(*mCV.camera_mat,float,0,2)= 152.421799;
	CV_MAT_ELEM(*mCV.camera_mat,float,1,0)=0.0;
	CV_MAT_ELEM(*mCV.camera_mat,float,1,1)=309.993622;
	CV_MAT_ELEM(*mCV.camera_mat,float,1,2)=127.563507;
	CV_MAT_ELEM(*mCV.camera_mat,float,2,0)=0.0;
	CV_MAT_ELEM(*mCV.camera_mat,float,2,1)=0.0;
	CV_MAT_ELEM(*mCV.camera_mat,float,2,2)=1.0;

	CV_MAT_ELEM(*mCV.distort_mat,float,0,0)=0.241287;
	CV_MAT_ELEM(*mCV.distort_mat,float,0,1)=-0.814837;
	CV_MAT_ELEM(*mCV.distort_mat,float,0,2)=0.043473;
	CV_MAT_ELEM(*mCV.distort_mat,float,0,3)=-0.019282;


    mCV.MarkerRot=cvCreateMat(1,3,CV_32FC1);
    mCV.MarkerTrans=cvCreateMat(1,3,CV_32FC1);
	//mCV.rotVec=cvCreateMat(1,3,CV_32FC1);
	//mCV.transVec=cvCreateMat(1,3,CV_32FC1);
    mCV.Rot=cvCreateMat(3,3,CV_32FC1);
	mCV.Rot44=cvCreateMat(4,4,CV_32FC1);
	mCV.Trans=cvCreateMat(4,1,CV_32FC1);
	mCV.Result=cvCreateMat(4,1,CV_32FC1);
	mCV.RotAxis=cvCreateMat(1,3,CV_32FC1);

	/*
	Correction.set( 0.0, -1.0,  0.0,  0.0,
				    1.0,  0.0,  0.0,  0.0,//1.0,  0.0,  0.0,  0.0,
					0.0,  0.0,  1.0,  0.0,
					0.0,  0.0,  0.0,  1.0);


	Correction.set( 1.0,  0.0,  0.0,  0.0,
				    0.0,  1.0,  0.0,  0.0,
					0.0,  0.0,  1.0,  0.0,
					0.0,  0.0,  0.0,  1.0);
	*/

	Correction.set( 1.0,  0.0,  0.0,  0.0,
				    0.0, -1.0,  0.0,  0.0,
					0.0,  0.0, -1.0,  0.0,
					0.0,  0.0,  0.0,  1.0);


	loadMarkers();
	initGlobalVars();
	FirstPose=true;

#ifdef MARKERLESS_PART
	//markerless part
    mCV.rots.resize(4);
    mCV.transs.resize(4);
    for(int i=0;i<4;i++)
    {
    	mCV.rots.at(i)=cvCreateMat(1,3,CV_32FC1);
    	mCV.transs.at(i)=cvCreateMat(1,3,CV_32FC1);
    }

    mCV.KFRot=cvCreateMat(1,3,CV_32FC1);
    mCV.KFTrans=cvCreateMat(1,3,CV_32FC1);
#endif
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
		//exit(-1);
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
		/*
		Correction.set( 0.0, -1.0,  0.0,  0.0,
					    1.0,  0.0,  0.0,  0.0,//1.0,  0.0,  0.0,  0.0,
						0.0,  0.0,  1.0,  0.0,
						0.0,  0.0,  0.0,  1.0);


		Correction.set( 1.0,  0.0,  0.0,  0.0,
					    0.0,  1.0,  0.0,  0.0,
						0.0,  0.0,  1.0,  0.0,
						0.0,  0.0,  0.0,  1.0);
		*/

		Correction.set( 1.0,  0.0,  0.0,  0.0,
					    0.0, -1.0,  0.0,  0.0,
						0.0,  0.0, -1.0,  0.0,
						0.0,  0.0,  0.0,  1.0);


		loadMarkers(storagePath);
		initGlobalVars();
		FirstPose=true;

#ifdef MARKERLESS_PART
	//markerless part
    mCV.rots.resize(4);
    mCV.transs.resize(4);
    for(int i=0;i<4;i++)
    {
    	mCV.rots.at(i)=cvCreateMat(1,3,CV_32FC1);
    	mCV.transs.at(i)=cvCreateMat(1,3,CV_32FC1);
    }

    mCV.KFRot=cvCreateMat(1,3,CV_32FC1);
    mCV.KFTrans=cvCreateMat(1,3,CV_32FC1);
#endif
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
		//exit(-1);
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
