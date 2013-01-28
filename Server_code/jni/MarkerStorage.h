#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <android/log.h>
#include <iostream>
using namespace cv;
float P3fLength(Point3f p);
float P3fDist(Point3f p1,Point3f p2);
void normalizeP3f(Point3f & p);
float showAngle(Point3f center, Point3f p1, Point3f p2);
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
    }
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

    MarkerStorage()
    {
        buildTime=9999;
        numImgs=0;
        valid=false;
        WPt2=cvCreateMat(4,3,CV_32FC1);
        currentBuildEdgeError=9999.0;
        currentBuildAngleError=9999.0;
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
            __android_log_print(ANDROID_LOG_INFO,"EstimateTR","EstimateTR, corner x: %f, y: %f",corners[i].x,corners[i].y);

        }
        cvFindExtrinsicCameraParams2 (WPt2, ImgP, camera_mat, distort_mat, OutRot, OutTrans,0);
        cvReleaseMat(&ImgP);
        __android_log_print(ANDROID_LOG_INFO,"EstimateTR","EstimateTR, finish");
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
typedef struct MarkerStorage MS;
typedef std::map<int,MS> MSMap;
typedef std::pair<int,MS> MSPair;
