#include "CVAuxFunc.h"

void AxisAngleToMatrix(CvMat* & Mat,CvMat* & Axis,float & angle)
{
    float x=CV_MAT_ELEM(*Axis,float,0,0);
    float y=CV_MAT_ELEM(*Axis,float,0,1);
    float z=CV_MAT_ELEM(*Axis,float,0,2);

    float c=(float)cos(angle);
    float mc=1-c;
    float s=(float)sin(angle);

    CV_MAT_ELEM(*Mat,float,0,0)=c+x*x*mc  ;  CV_MAT_ELEM(*Mat,float,0,1)=x*y*mc-z*s  ;  CV_MAT_ELEM(*Mat,float,0,2)=x*z*mc+y*s;
    CV_MAT_ELEM(*Mat,float,1,0)=x*y*mc+z*s;  CV_MAT_ELEM(*Mat,float,1,1)=c+y*y*mc  ;  CV_MAT_ELEM(*Mat,float,1,2)=y*z*mc-x*s;
    CV_MAT_ELEM(*Mat,float,2,0)=z*x*mc-y*s;  CV_MAT_ELEM(*Mat,float,2,1)=z*y*mc+x*s;  CV_MAT_ELEM(*Mat,float,2,2)=c+z*z*mc;
}


void MatrixToAxisAngle(CvMat* & Mat,CvMat* & Axis,float & angle)
{
    float m[3][3];
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
            m[i][j]=CV_MAT_ELEM(*Mat,float,i,j);
    }

    angle=acos((m[0][0]+m[1][1]+m[2][2]-1)/2);
    float a=(m[2][1]-m[1][2])*(m[2][1]-m[1][2]);
    float b=(m[0][2]-m[2][0])*(m[0][2]-m[2][0]);
    float c=(m[1][0]-m[0][1])*(m[1][0]-m[0][1]);
    float d=sqrt(a+b+c);
    float x=(m[2][1]-m[1][2])/d;
    float y=(m[0][2]-m[2][0])/d;
    float z=(m[1][0]-m[0][1])/d;

    CV_MAT_ELEM(*Axis,float,0,0)=x;
    CV_MAT_ELEM(*Axis,float,0,1)=y;
    CV_MAT_ELEM(*Axis,float,0,2)=z;
}

void point2fToCvMat(CvMat* & mat,std::vector<Point2f> & points,bool TwoByN)
{
	int numPts=points.size();
	if(TwoByN)
	{
		cvReleaseMat(&mat);
		mat=cvCreateMat(2,numPts,CV_32FC1);
		for(int i=0;i<numPts;i++)
		{
			 CV_MAT_ELEM(*mat,float,0,i)=(float)points.at(i).x;
			 CV_MAT_ELEM(*mat,float,1,i)=(float)points.at(i).y;
		}
	}
	else
	{
		cvReleaseMat(&mat);
		mat=cvCreateMat(numPts,2,CV_32FC1);
		for(int i=0;i<numPts;i++)
		{
			 CV_MAT_ELEM(*mat,float,i,0)=(float)points.at(i).x;
			 CV_MAT_ELEM(*mat,float,i,1)=(float)points.at(i).y;
		}
	}
}
void CvMatToPoint2f(CvMat* & mat, std::vector<Point2f> &  points,bool TwoByN)
{
	points.clear();
	int numPts;
	if(TwoByN)
	{
		numPts=mat->cols;
		for(int i=0;i<numPts;i++)
		{
			Point2f p;
			p.x=CV_MAT_ELEM(*mat,float,0,i);
			p.y=CV_MAT_ELEM(*mat,float,1,i);
			points.push_back(p);
		}
	}
	else
	{
		numPts=mat->rows;
		for(int i=0;i<numPts;i++)
		{
			Point2f p;
			p.x=CV_MAT_ELEM(*mat,float,i,0);
			p.y=CV_MAT_ELEM(*mat,float,i,1);
			points.push_back(p);
		}
	}
}


void MundistortPoints( InputArray _src, OutputArray _dst,
                          InputArray _cameraMatrix,
                          InputArray _distCoeffs,
                          InputArray _Rmat,
                          InputArray _Pmat )
{
    Mat src = _src.getMat(), cameraMatrix = _cameraMatrix.getMat();
    Mat distCoeffs = _distCoeffs.getMat(), R = _Rmat.getMat(), P = _Pmat.getMat();
    if( src.isContinuous() )
    {
    	if(src.depth() == CV_32F || src.depth() == CV_64F)
    	{

    	}

    }

    _dst.create(src.size(), src.type(), -1, true);
    Mat dst = _dst.getMat();

    CvMat _csrc = src, _cdst = dst, _ccameraMatrix = cameraMatrix;
    CvMat matR, matP, _cdistCoeffs, *pR=0, *pP=0, *pD=0;
    if( R.data )
        pR = &(matR = R);
    if( P.data )
        pP = &(matP = P);
    if( distCoeffs.data )
        pD = &(_cdistCoeffs = distCoeffs);
    cvUndistortPoints(&_csrc, &_cdst, &_ccameraMatrix, pD, pR, pP);
}

void computeTrans(CvMat* & trans,CvMat* & rot,CvMat* & Rot, CvMat* & Rot44,CvMat* & Result, float & x, float &y, float & z)
{
	cvRodrigues2(rot,Rot,NULL);
	__android_log_print(ANDROID_LOG_INFO,"compute trans","OK1");
    CV_MAT_ELEM(*Rot44,float,0,0)=CV_MAT_ELEM(*Rot,float,0,0); CV_MAT_ELEM(*Rot44,float,0,1)=CV_MAT_ELEM(*Rot,float,0,1); CV_MAT_ELEM(*Rot44,float,0,2)=CV_MAT_ELEM(*Rot,float,0,2); CV_MAT_ELEM(*Rot44,float,0,3)=0.0;
    CV_MAT_ELEM(*Rot44,float,1,0)=CV_MAT_ELEM(*Rot,float,1,0); CV_MAT_ELEM(*Rot44,float,1,1)=CV_MAT_ELEM(*Rot,float,1,1); CV_MAT_ELEM(*Rot44,float,1,2)=CV_MAT_ELEM(*Rot,float,1,2); CV_MAT_ELEM(*Rot44,float,1,3)=0.0;
    CV_MAT_ELEM(*Rot44,float,2,0)=CV_MAT_ELEM(*Rot,float,2,0); CV_MAT_ELEM(*Rot44,float,2,1)=CV_MAT_ELEM(*Rot,float,2,1); CV_MAT_ELEM(*Rot44,float,2,2)=CV_MAT_ELEM(*Rot,float,2,2); CV_MAT_ELEM(*Rot44,float,2,3)=0.0;
    CV_MAT_ELEM(*Rot44,float,3,0)=0.0;                         CV_MAT_ELEM(*Rot44,float,3,1)=0.0;                         CV_MAT_ELEM(*Rot44,float,3,2)=0.0;                         CV_MAT_ELEM(*Rot44,float,3,3)=1.0;
    __android_log_print(ANDROID_LOG_INFO,"compute trans","OK2");
    cvGEMM(Rot44,trans,1.0,NULL,0.0,Result,0);
    __android_log_print(ANDROID_LOG_INFO,"compute trans","OK3");
    x=CV_MAT_ELEM(*Result,float,0,0)/CV_MAT_ELEM(*Result,float,3,0);
    y=CV_MAT_ELEM(*Result,float,1,0)/CV_MAT_ELEM(*Result,float,3,0);
    z=CV_MAT_ELEM(*Result,float,2,0)/CV_MAT_ELEM(*Result,float,3,0);
    __android_log_print(ANDROID_LOG_INFO,"compute trans","OK4");
}
void AxisAngleToRotVec(CvMat* & rot,CvMat* & Axis,float & angle)
{
    CvMat* Mat=cvCreateMat(3,3,CV_32FC1);
    float x=CV_MAT_ELEM(*Axis,float,0,0);
    float y=CV_MAT_ELEM(*Axis,float,0,1);
    float z=CV_MAT_ELEM(*Axis,float,0,2);

    float c=(float)cos(angle);
    float mc=1-c;
    float s=(float)sin(angle);

    CV_MAT_ELEM(*Mat,float,0,0)=c+x*x*mc  ;  CV_MAT_ELEM(*Mat,float,0,1)=x*y*mc-z*s  ;  CV_MAT_ELEM(*Mat,float,0,2)=x*z*mc+y*s;
    CV_MAT_ELEM(*Mat,float,1,0)=x*y*mc+z*s;  CV_MAT_ELEM(*Mat,float,1,1)=c+y*y*mc  ;  CV_MAT_ELEM(*Mat,float,1,2)=y*z*mc-x*s;
    CV_MAT_ELEM(*Mat,float,2,0)=z*x*mc-y*s;  CV_MAT_ELEM(*Mat,float,2,1)=z*y*mc+x*s;  CV_MAT_ELEM(*Mat,float,2,2)=c+z*z*mc;
    cvRodrigues2(Mat,rot);
    cvReleaseMat(&Mat);
}
void RotVecToAxisAngle(CvMat* & rot,CvMat* & Axis,float & angle)
{
    CvMat* Mat=cvCreateMat(3,3,CV_32FC1);
    cvRodrigues2(rot,Mat);
    float m[3][3];
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
            m[i][j]=CV_MAT_ELEM(*Mat,float,i,j);
    }

    angle=acos((m[0][0]+m[1][1]+m[2][2]-1)/2);
    float a=(m[2][1]-m[1][2])*(m[2][1]-m[1][2]);
    float b=(m[0][2]-m[2][0])*(m[0][2]-m[2][0]);
    float c=(m[1][0]-m[0][1])*(m[1][0]-m[0][1]);
    float d=sqrt(a+b+c);
    float x=(m[2][1]-m[1][2])/d;
    float y=(m[0][2]-m[2][0])/d;
    float z=(m[1][0]-m[0][1])/d;

    CV_MAT_ELEM(*Axis,float,0,0)=x;
    CV_MAT_ELEM(*Axis,float,0,1)=y;
    CV_MAT_ELEM(*Axis,float,0,2)=z;
    cvReleaseMat(&Mat);
}
void projectPoints(std::vector<Point3f> & objPts,CvMat* & rot,CvMat* & trans,CvMat* & cam_mat,CvMat* & distort,std::vector<Point2f> & result)
{
    CvMat* point3=cvCreateMat(objPts.size(),3,CV_32FC1);
    CvMat* point2=cvCreateMat(objPts.size(),2,CV_32FC1);

    for(int i=0;i<objPts.size();i++)
    {
            CV_MAT_ELEM(*point3,float,i,0)=objPts.at(i).x;
            CV_MAT_ELEM(*point3,float,i,1)=objPts.at(i).y;
            CV_MAT_ELEM(*point3,float,i,2)=objPts.at(i).z;

    }
    cvProjectPoints2(point3,
                    rot,trans,
                    cam_mat,distort,point2,
            NULL,NULL,NULL,NULL,NULL,0);
    result.clear();
    for(int i=0;i<objPts.size();i++)
    {
            Point2f p;
            p.x=CV_MAT_ELEM(*point2,float,i,0);
            p.y=CV_MAT_ELEM(*point2,float,i,1);
            result.push_back(p);
    }

    cvReleaseMat(&point3);
    cvReleaseMat(&point2);
}
