#ifndef FUNCTIONS_H
#define FUNCTIONS_H
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include "CVAuxFunc.h"
#include <time.h>
using namespace cv;
/*
#define rotAxisX 0
#define rotAxisY 1
#define rotAxisZ 2
#define rotAngle 3
#define transX 4
#define transY 5
#define transZ 6
#define ptXinJac 0
#define ptYinJac 1
#define ptXin3D 0
#define ptYin3D 1
#define ptZin3D 2
*/
float showAngle(Point3f center, Point3f p1, Point3f p2);
void Point2fVectorToCvMat(std::vector<Point2f> & srcPt, CvMat* & dstPt,bool rowMajor);
//get vector length
float P3fLength(Point3f p);
//get distance between 2 Point3f
float P3fDist(Point3f p1,Point3f p2);
//normalize vector
void normalizeP3f(Point3f & p);
//linear triangulation
void LinearSTTriangulation(int numViews,CvMat* & cam_mat,std::vector<CvMat*> &Rots, std::vector<CvMat*> & Trans, std::vector<std::vector<Point2f> > & imgPts, std::vector<Point3f> & WPts);
void LinearSTTriangulation(int numViews,CvMat* & cam_mat,std::vector<CvMat*> &Rots, std::vector<CvMat*> & Trans,std::vector<CvMat*> & ProjMat, std::vector<std::vector<Point2f> > & imgPts, std::vector<Point3f> & WPts);
//iterative linear triangulation
void IterativeLinearSTTriangulation(int numViews,CvMat* & cam_mat,std::vector<CvMat*> &Rots, std::vector<CvMat*> & Trans, std::vector<std::vector<Point2f> > & imgPts, std::vector<Point3f> & WPts);
//used to store marker's info (2D images from camera preview and info of coordinates
struct MarkerStorage
{
    int ID;
    std::vector<CvMat*> WorldRots;
    std::vector<CvMat*> WorldTranss;
    std::vector<std::vector<Point2f> >imgPts;
    std::vector<int> imgIndex;
    std::vector<Point3f> WPts;
    std::vector<float> oldestTime;
    CvMat* WPt2;
    CvMat* camera_mat;
    CvMat* distort_mat;
    int buildTime;
    bool valid;
    int numImgs;
    float currentBuildEdgeError;
    float currentBuildAngleError;
    float currentBuildReprojError;
    //reset data
    void clearData()
    {
    	for(int i=0;i<numImgs;i++)
    	{
    		cvReleaseMat(&WorldRots.at(i));
    		cvReleaseMat(&WorldTranss.at(i));
    		imgPts.at(i).clear();
    	}
    	WorldRots.clear();
    	WorldTranss.clear();
    	imgPts.clear();
    	imgIndex.clear();
    	WPts.clear();
    	oldestTime.clear();
    	WPts.clear();
    	cvReleaseMat(&WPt2);
    	WPt2=cvCreateMat(4,3,CV_32FC1);
    	buildTime=9999;
    	valid=false;
    	numImgs=0;
    	currentBuildEdgeError=9999.0;
    	currentBuildAngleError=9999.0;
    	currentBuildReprojError=9999.0;
    }
    //clear stored data
    void release()
    {
    	for(int i=0;i<numImgs;i++)
    	{
    		cvReleaseMat(&WorldRots.at(i));
    		cvReleaseMat(&WorldTranss.at(i));
    		imgPts.at(i).clear();
    	}
    	WorldRots.clear();
    	WorldTranss.clear();
    	imgPts.clear();
    	imgIndex.clear();
    	WPts.clear();
    	oldestTime.clear();
    	WPts.clear();
    	cvReleaseMat(&WPt2);
    }
    //copy info
    void clone(MarkerStorage & nMS)
    {
    	int TnumImgs=nMS.numImgs;
    	clearData();
    	for(int i=0;i<TnumImgs;i++)
    	{
    		addImageInfo(nMS.imgPts.at(i),nMS.WorldRots.at(i),nMS.WorldTranss.at(i),nMS.oldestTime.at(i));
    	}
    	buildTime=nMS.buildTime;
    	valid=nMS.valid;
    	WPts=nMS.WPts;
    	currentBuildEdgeError=nMS.currentBuildEdgeError;
    	currentBuildAngleError=nMS.currentBuildAngleError;
    	currentBuildReprojError=nMS.currentBuildReprojError;
    }
    //add image info from camera preview
    void addImageInfo(CvPoint2D32f corners[],CvMat* & Rot, CvMat* & Trans)
    {
        std::vector<Point2f> pts;
        for(int i=0;i<4;i++)
        {
            Point2f p;
            p.x=corners[i].x;
            p.y=corners[i].y;
            pts.push_back(p);
            //std::cout<<"add image, corner "<<i<<" ,x: "<<p.x<<" ,y: "<<p.y<<std::endl;
        }
        imgPts.push_back(pts);
        CvMat* rot=cvCreateMat(1,3,CV_32FC1);
        rot=cvCloneMat(Rot);
        CvMat* trans=cvCreateMat(1,3,CV_32FC1);
        trans=cvCloneMat(Trans);
        WorldRots.push_back(rot);
        WorldTranss.push_back(trans);
        numImgs++;
    }
    void addImageInfo(CvPoint2D32f corners[],CvMat* & Rot, CvMat* & Trans,int OldestTime)
    {
        std::vector<Point2f> pts;
        for(int i=0;i<4;i++)
        {
            Point2f p;
            p.x=corners[i].x;
            p.y=corners[i].y;
            pts.push_back(p);
            //std::cout<<"add image, corner "<<i<<" ,x: "<<p.x<<" ,y: "<<p.y<<std::endl;

        }
        oldestTime.push_back((float)OldestTime);
        imgPts.push_back(pts);
        CvMat* rot=cvCreateMat(1,3,CV_32FC1);
        rot=cvCloneMat(Rot);
        CvMat* trans=cvCreateMat(1,3,CV_32FC1);
        trans=cvCloneMat(Trans);
        WorldRots.push_back(rot);
        WorldTranss.push_back(trans);
        numImgs++;
    }
    void addImageInfo(std::vector<Point2f> & corners,CvMat* & Rot, CvMat* & Trans,int OldestTime)
    {
        oldestTime.push_back((float)OldestTime);
        imgPts.push_back(corners);
        CvMat* rot=cvCreateMat(1,3,CV_32FC1);
        rot=cvCloneMat(Rot);
        CvMat* trans=cvCreateMat(1,3,CV_32FC1);
        trans=cvCloneMat(Trans);
        WorldRots.push_back(rot);
        WorldTranss.push_back(trans);
        numImgs++;
    }


    MarkerStorage()
    {
        buildTime=9999;
        numImgs=0;
        valid=false;
        WPt2=cvCreateMat(4,3,CV_32FC1);
        currentBuildEdgeError=9999.0;
        currentBuildAngleError=9999.0;
        currentBuildReprojError=9999.0;
    }
    MarkerStorage(int nID)
    {
        buildTime=9999;
        ID=nID;
        numImgs=0;
        valid=false;
        WPt2=cvCreateMat(4,3,CV_32FC1);
        currentBuildEdgeError=9999.0;
        currentBuildAngleError=9999.0;
        currentBuildReprojError=9999.0;
    }

    MarkerStorage(std::vector<Point3f> nWPt,int nID)
    {
        buildTime=9999;
        numImgs=0;
        valid=true;
        WPt2=cvCreateMat(4,3,CV_32FC1);
        ID=nID;
        WPts=nWPt;
        BuildWPt2();
        currentBuildEdgeError=9999.0;
        currentBuildAngleError=9999.0;
        currentBuildReprojError=9999.0;
    }

    void BuildWPt2()
    {
        if(WPts.size()!=4)
            return;
        for(int i=0;i<4;i++)
        {
            CV_MAT_ELEM(*WPt2,float,i,0)=WPts.at(i).x;
            CV_MAT_ELEM(*WPt2,float,i,1)=WPts.at(i).y;
            CV_MAT_ELEM(*WPt2,float,i,2)=WPts.at(i).z;
        }
    }


    //estimate camera pose
    void EstimateTR(std::vector<Point2f> & ImgPts,CvMat* & camera_mat,CvMat* & distort_mat,CvMat* & OutRot,CvMat* & OutTrans)
    {
        CvMat* ImgP=cvCreateMat(4,2,CV_32FC1);
        for(int i=0;i<4;i++)
        {
            CV_MAT_ELEM(*ImgP,float,i,0)=ImgPts.at(i).x;
            CV_MAT_ELEM(*ImgP,float,i,1)=ImgPts.at(i).y;
        }
        cvFindExtrinsicCameraParams2 (WPt2, ImgP, camera_mat, distort_mat, OutRot, OutTrans,0);
        cvReleaseMat(&ImgP);
    }
    void EstimateTR(CvPoint2D32f  corners [] ,CvMat* & camera_mat,CvMat* & distort_mat,CvMat* & OutRot,CvMat* & OutTrans)
    {
        CvMat* ImgP=cvCreateMat(4,2,CV_32FC1);
        for(int i=0;i<4;i++)
        {
            CV_MAT_ELEM(*ImgP,float,i,0)=corners[i].x;
            CV_MAT_ELEM(*ImgP,float,i,1)=corners[i].y;
            //std::cout<<"estimate TR, corner "<<i<<" ,x: "<<corners[i].x<<" ,y: "<<corners[i].y<<std::endl;
        }
        cvFindExtrinsicCameraParams2 (WPt2, ImgP, camera_mat, distort_mat, OutRot, OutTrans,0);
        cvReleaseMat(&ImgP);
    }
    void EstimateTR(CvPoint2D32f  corners [] ,CvMat* & OutRot,CvMat* & OutTrans)
    {
        CvMat* ImgP=cvCreateMat(4,2,CV_32FC1);
        for(int i=0;i<4;i++)
        {
            CV_MAT_ELEM(*ImgP,float,i,0)=corners[i].x;
            CV_MAT_ELEM(*ImgP,float,i,1)=corners[i].y;
        }
        cvFindExtrinsicCameraParams2 (WPt2, ImgP, camera_mat, distort_mat, OutRot, OutTrans,0);
        cvReleaseMat(&ImgP);
    }

    void showDistBetweenPts()
    {
        if(WPts.size()==0)
        {
            std::cout<<"no WPts"<<std::endl;
            return;
        }
        float x,y,z;
        x=WPts.at(0).x-WPts.at(1).x;
        y=WPts.at(0).y-WPts.at(1).y;
        z=WPts.at(0).z-WPts.at(1).z;
        //std::cout<<"1,2: "<<sqrt(x*x+y*y+z*z)<<std::endl;
        __android_log_print(ANDROID_LOG_INFO,"show dist between pts","0,1: %f",sqrt(x*x+y*y+z*z));
        x=WPts.at(1).x-WPts.at(2).x;
        y=WPts.at(1).y-WPts.at(2).y;
        z=WPts.at(1).z-WPts.at(2).z;
        //std::cout<<"2,3: "<<sqrt(x*x+y*y+z*z)<<std::endl;
        __android_log_print(ANDROID_LOG_INFO,"show dist between pts","1,2: %f",sqrt(x*x+y*y+z*z));
        x=WPts.at(2).x-WPts.at(3).x;
        y=WPts.at(2).y-WPts.at(3).y;
        z=WPts.at(2).z-WPts.at(3).z;
        //std::cout<<"2,3: "<<sqrt(x*x+y*y+z*z)<<std::endl;
        __android_log_print(ANDROID_LOG_INFO,"show dist between pts","2,3: %f",sqrt(x*x+y*y+z*z));
        x=WPts.at(3).x-WPts.at(0).x;
        y=WPts.at(3).y-WPts.at(0).y;
        z=WPts.at(3).z-WPts.at(0).z;
        //std::cout<<"4,1: "<<sqrt(x*x+y*y+z*z)<<std::endl;
        __android_log_print(ANDROID_LOG_INFO,"show dist between pts","3,0: %f",sqrt(x*x+y*y+z*z));
    }

    void showAngleForEachCorner()
    {
        if(WPts.size()==0)
        {
            std::cout<<"no WPts"<<std::endl;
            return;
        }
        Point3f center,p1,p2;
        center=WPts.at(0);
        p1=WPts.at(1);
        p2=WPts.at(3);
        //std::cout<<"corner 0 angle: "<<showAngle(center,p1,p2)<<std::endl;
        __android_log_print(ANDROID_LOG_INFO,"show Angle for each corner","corner 0: %f",showAngle(center,p1,p2));
        center=WPts.at(1);
        p1=WPts.at(0);
        p2=WPts.at(2);
        //std::cout<<"corner 1 angle: "<<showAngle(center,p1,p2)<<std::endl;
        __android_log_print(ANDROID_LOG_INFO,"show Angle for each corner","corner 1: %f",showAngle(center,p1,p2));
        center=WPts.at(2);
        p1=WPts.at(1);
        p2=WPts.at(3);
        //std::cout<<"corner 2 angle: "<<showAngle(center,p1,p2)<<std::endl;
        __android_log_print(ANDROID_LOG_INFO,"show Angle for each corner","corner 2: %f",showAngle(center,p1,p2));
        center=WPts.at(3);
        p1=WPts.at(0);
        p2=WPts.at(2);
        //std::cout<<"corner 3 angle: "<<showAngle(center,p1,p2)<<std::endl;
        __android_log_print(ANDROID_LOG_INFO,"show Angle for each corner","corner 3: %f",showAngle(center,p1,p2));
    }

    float showAngleForOneCorner(int whichCorner)
    {
        if(WPts.size()==0)
        {
            std::cout<<"no WPts"<<std::endl;
            return -5.0;
        }
        Point3f center,p1,p2;
        switch(whichCorner)
        {
        case 0:
            center=WPts.at(0);
            p1=WPts.at(1);
            p2=WPts.at(3);
            break;
        case 1:
            center=WPts.at(1);
            p1=WPts.at(0);
            p2=WPts.at(2);
            break;
        case 2:
            center=WPts.at(2);
            p1=WPts.at(1);
            p2=WPts.at(3);
            break;
        case 3:
            center=WPts.at(3);
            p1=WPts.at(0);
            p2=WPts.at(2);
            break;
        }
        return showAngle(center,p1,p2);
    }
    float showEdgeDist(int a,int b)
    {
        if(WPts.size()==0)
        {
            std::cout<<"no WPts"<<std::endl;
            return -1.0;
        }
        if((a==0&&b==1)||(a==1&&b==0))
            return P3fDist(WPts.at(0),WPts.at(1));
        if((a==1&&b==2)||(a==2&&b==1))
            return P3fDist(WPts.at(1),WPts.at(2));
        if((a==2&&b==3)||(a==3&&b==2))
            return P3fDist(WPts.at(2),WPts.at(3));
        if((a==3&&b==0)||(a==0&&b==3))
            return P3fDist(WPts.at(3),WPts.at(0));
    }
};
//my point struct, used for plane fitting
struct mpoint
{
    float x,y,z;
    mpoint(float X,float Y,float Z)
    {
        x=X;y=Y,z=Z;
    }
    mpoint()
    {
        x=y=z=0.0;
    }
    void clone(mpoint p)
    {
        x=p.x;
        y=p.y;
        z=p.z;
    }

     mpoint operator + (const mpoint & np)
    {
    	return mpoint(np.x+x,np.y+y,np.z+z);
    }
    mpoint operator - (const mpoint &np)
    {
    	return mpoint(x-np.x,y-np.y,z-np.z);
    }
    mpoint operator ^ (const mpoint & np)
    {
    	float x,y,z;
    	x=-this->z*np.y+this->y*np.z;
    	y=this->z*np.x-this->x*np.z;
    	z=-this->y*np.x+this->x*np.y;
    	return mpoint(x,y,z);
    }
    mpoint operator = (const mpoint & np)
    {
    	x=np.x;
    	y=np.y;
    	z=np.z;
    	return *this;
    }
    float operator * (const mpoint & np)
	{
    	return x*np.x+y*np.y+z*np.z;
	}
     mpoint  operator * (const float f)
    {
    	float x=this->x*f;
    	float y=this->y*f;
    	float z=this->z*f;
    	return mpoint(x,y,z);
    }
    float length()
    {
    	return sqrt(x*x+y*y+z*z);
    }
    void normalize()
    {
    	float len=length();
    	x=x/len;
    	y=y/len;
    	z=z/len;
    }


};
typedef struct mpoint mPoint;
//my plane structure for plane fitting
struct plane
{
    mPoint pts[4];
    float mat[3][3];
    float  Res[3];
    float params[4];
    plane()
    {

    }

    void buildMatAndRes()
    {
        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                mat[i][j]=0.0f;
            }
        }
        float xi2=0.0;
        for(int i=0;i<4;i++)
            xi2+=pts[i].x*pts[i].x;
        float xi=0.0;
        for(int i=0;i<4;i++)
            xi+=pts[i].x;
        float yi2=0.0;
        for(int i=0;i<4;i++)
            yi2+=pts[i].y*pts[i].y;
        float yi=0.0;
        for(int i=0;i<4;i++)
            yi+=pts[i].y;

        float xiyi=0.0;
        for(int i=0;i<4;i++)
            xiyi+=pts[i].y*pts[i].x;
        mat[0][0]=xi2;
        mat[0][1]=mat[1][0]=xiyi;
        mat[0][2]=mat[2][0]=xi;
        mat[1][1]=yi2;
        mat[1][2]=mat[2][1]=yi;
        mat[2][2]=4;
        float xizi=0.0;
        for(int i=0;i<4;i++)
            xizi+=pts[i].x*pts[i].z;
        float yizi=0.0;
        for(int i=0;i<4;i++)
            yizi+=pts[i].y*pts[i].z;

        float zi=0.0;
        for(int i=0;i<4;i++)
            zi+=pts[i].z;
        Res[0]=xizi;
        Res[1]=yizi;
        Res[2]=zi;
    }
    void showMat()
    {
        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
                std::cout<<mat[i][j]<<" ";
            std::cout<<std::endl;
        }

    }

    void solveNormal()
    {
        buildMatAndRes();
        float ratio1,ratio2;
        ratio1=mat[1][0]/mat[0][0];
        ratio2=mat[2][0]/mat[0][0];
        for(int i=0;i<3;i++)
        {
            mat[1][i]+=(-mat[0][i]*ratio1);
            mat[2][i]+=(-mat[0][i]*ratio2);
        }
        Res[1]+=(-Res[0]*ratio1);
        Res[2]+=(-Res[0]*ratio2);
        ratio1=mat[2][1]/mat[1][1];
        mat[2][1]+=(-ratio1*mat[1][1]);
        mat[2][2]+=(-ratio1*mat[1][2]);
        Res[2]+=(-Res[1]*ratio1);
        params[2]=Res[2]/mat[2][2];
        params[1]=(Res[1]-mat[1][2]*params[2])/mat[1][1];
        params[0]=(Res[0]-mat[0][2]*params[2]-mat[0][1]*params[1])/mat[0][0];
        params[3]=-params[2];
        params[2]=1.0f;
        params[1]=-params[1];
        params[0]=-params[0];
    }
};
typedef struct plane Plane;
typedef struct MarkerStorage MS;
typedef std::map<int,MS> MSMap;
typedef std::pair<int,MS> MSPair;
//void BundleMarker(MS & mMS,CvMat* & cam_mat, CvMat* & distort_mat);
//void BundleMarker2(MS & mMS,CvMat* & cam_mat, CvMat* & distort_mat);
//use RANSAC to calculate marker's 3D position
bool RANSACTriangulation(MS & mMS,CvMat* & cam_mat, int rounds);
bool RANSACTriangulation(int numSampleImg,MS & mMS,CvMat* & cam_mat);//check by rectangle requirement
bool RANSACTriangulation(MS & mMS,CvMat* & cam_mat,CvMat* & distort_mat, int rounds);
bool RANSACTriangulation(int numSampleImg,MS & mMS,CvMat* & cam_mat,CvMat* & distort_mat);//check by reprojection
void RANSACTriangulation2Methods(MS & mMS1,MS & mMS2,CvMat* & cam_mat,CvMat* & distort_mat,bool & MS1good, bool & MS2good, int numImgUse,int rounds);
//marker 3D position calculation based on rectangle fitting criteria
bool RANSACTriangulationByRect(int numSampleImg,MS & mMS,CvMat* & cam_mat,CvMat* &distort_mat);//check by rectangle requirement
//marker 3D position calculation based on reprojection error criteria
bool RANSACTriangulationByReproj(int numSampleImg,MS & mMS,CvMat* & cam_mat,CvMat* & distort_mat);//check by reprojection
//undistorion image points
bool UndistortPixel(std::vector<Point2f> & src, std::vector<Point2f> & dst,CvMat* & cam_mat,CvMat* & distort_mat);
void projMScorners(MS & mMS);
mPoint projPointToPlane(mPoint & pt,Plane & E);
void rectifyRectangle(std::vector<mPoint> & pts);
void rectifyRectangle(std::vector<Point3f> & pts);
void rectifyRectangle(MS & mMS);
void rotateByAxisAngle(float angle, mPoint axis, mPoint & inOut);
//transform calculated marker to be a perfect square
void rectifyRectangle2(std::vector<mPoint> & pts);
void rectifyRectangle2(std::vector<Point3f> & pts);
void rectifyRectangle2(MS & mMS);
#endif // FUNCTIONS_H
