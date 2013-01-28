#include "MarkerlessFunc.h"

void FindCamPoseByKF(const CvMat* camera_matrix,const CvMat* distortion_coeffs,std::vector<Point3f> & Pt3Ds, std::vector<Point2f> & imgPts,CvMat* & rot, CvMat* & trans)
{



	int PtSize=Pt3Ds.size();
	CvMat* realPts=cvCreateMat(PtSize,3,CV_32FC1);
	CvMat* imgpts=cvCreateMat(PtSize,2,CV_32FC1);
#ifdef DEBUG
	 __android_log_print(ANDROID_LOG_INFO,"FindCamPoseByKF","OK1");
#endif
	for(int i=0;i<Pt3Ds.size();i++)
	{
		Point3f p3=Pt3Ds.at(i);
		CV_MAT_ELEM(*realPts,float,i,0)=p3.x;
		CV_MAT_ELEM(*realPts,float,i,1)=p3.y;
		CV_MAT_ELEM(*realPts,float,i,2)=p3.z;
		// __android_log_print(ANDROID_LOG_INFO,"FindCamPoseByKF","OK2");
		CV_MAT_ELEM(*imgpts,float,i,0)=(float)imgPts.at(i).x;
		CV_MAT_ELEM(*imgpts,float,i,1)=(float)imgPts.at(i).y;
		 //__android_log_print(ANDROID_LOG_INFO,"FindCamPoseByKF","OK3");
	}
#ifdef DEBUG
	__android_log_print(ANDROID_LOG_INFO,"FindCamPoseByKF","OK3-1");
#endif
	cvFindExtrinsicCameraParams2 (realPts,imgpts, camera_matrix, distortion_coeffs, rot, trans);
#ifdef DEBUG
	__android_log_print(ANDROID_LOG_INFO,"FindCamPoseByKF","OK4");
#endif
	cvReleaseMat(&realPts);
	cvReleaseMat(&imgpts);
#ifdef DEBUG
	__android_log_print(ANDROID_LOG_INFO,"FindCamPoseByKF","OK5");
#endif
	/*
	Mat cam(camera_matrix);
	Mat dist(distortion_coeffs);
	Mat r(rot);
	Mat t(trans);
	solvePnP(Pt3Ds,imgPts,cam,dist,r,t,false);
	CV_MAT_ELEM(*rot,float,0,0)=r.at<float>(0,0);
	CV_MAT_ELEM(*rot,float,0,1)=r.at<float>(0,1);
	CV_MAT_ELEM(*rot,float,0,2)=r.at<float>(0,2);
	CV_MAT_ELEM(*trans,float,0,0)=t.at<float>(0,0);
	CV_MAT_ELEM(*trans,float,0,1)=t.at<float>(0,1);
	CV_MAT_ELEM(*trans,float,0,2)=t.at<float>(0,2);
	*/

}

bool newLKTrack(Mat & prev,Mat & next,std::vector<Point2f> & prevPts, std::vector<Point2f> & nextPts,double binTh,Size GaussSize,double sigma1,double sigma2, Size winsize, int maxLevel,TermCriteria criteria,double derivLambda , int flags, double minEigTh,double FundConfi, double FundDist)
{
	/*
    std::vector<Point2f> Pts2;
    std::vector<uchar> good;
    std::vector<float> err;

    Mat bin1=prev.clone();
    Mat bin2=next.clone();
    cv::threshold(prev,bin1,binTh,255.0,cv::THRESH_BINARY);
    cv::threshold(next,bin2,binTh,255.0,cv::THRESH_BINARY);
    Mat Blur1;
    Mat Blur2;
    cv::GaussianBlur(bin1,Blur1,GaussSize,sigma1,sigma2);
    cv::GaussianBlur(bin2,Blur2,GaussSize,sigma1,sigma2);
    cv::threshold(Blur1,bin1,binTh,255.0,cv::THRESH_BINARY);
    cv::threshold(Blur2,bin2,binTh,255.0,cv::THRESH_BINARY);
    Pts2=prevPts;
    calcOpticalFlowPyrLK(bin1,bin2,prevPts,Pts2,good,err,winsize,maxLevel,criteria,derivLambda,flags,minEigTh);

    cv::Mat fundemental;

    std::vector<Point2f> goodPts1;
    std::vector<Point2f> goodPts2;
    for(int i=0;i<prevPts.size();i++)
    {
       if(good.at(i))
       {
            goodPts1.push_back(prevPts.at(i));
            goodPts2.push_back(Pts2.at(i));
       }
    }
    if(goodPts1.size()<8)
    	return false;
    std::vector<uchar> inliers(goodPts1.size(),0);
    //double confidence=0.99;
    //double distance=3.0;
    fundemental= cv::findFundamentalMat(
           cv::Mat(goodPts1),cv::Mat(goodPts2), // matching points
            inliers,       // match status (inlier or outlier)
            CV_FM_RANSAC, // RANSAC method
            FundDist,      // distance to epipolar line
            FundConfi);

    std::vector<Point2f> newTrackPt;
    for(int i=0;i<goodPts2.size();i++)
    {
        if(inliers.at(i))
        {
        	//circle(*pMatRgb,goodPts2.at(i),3,CV_RGB(0,0,255));
        	newTrackPt.push_back(goodPts2.at(i));
        }
    }
    nextPts.clear();
    nextPts=newTrackPt;
    */
	std::vector<int> finalGoodIndex;
	std::vector<Point2f> nPts1;
	if(newLKTrack(prev,next,prevPts,nextPts,finalGoodIndex,binTh,GaussSize,sigma1,sigma2,winsize, maxLevel,criteria,derivLambda,flags, minEigTh,FundConfi,FundDist))
	{
		for(int i=0;i<finalGoodIndex.size();i++)
		{
			nPts1.push_back(prevPts.at(finalGoodIndex.at(i)));
		}
		prevPts.clear();
		prevPts=nPts1;
		return true;
	}
    return false;
}
bool newLKTrack(Mat & prev,Mat & next,std::vector<Point2f> & prevPts, std::vector<Point2f> & nextPts,std::vector<int> & finalGoodIndex,double binTh,Size GaussSize,double sigma1,double sigma2, Size winsize, int maxLevel,TermCriteria criteria,double derivLambda, int flags, double minEigTh,double FundConfi, double FundDist)
{
    std::vector<Point2f> Pts2;
    std::vector<uchar> good;
    std::vector<float> err;

    Mat bin1=prev.clone();
    Mat bin2=next.clone();
    cv::threshold(prev,bin1,binTh,255.0,cv::THRESH_BINARY);
    cv::threshold(next,bin2,binTh,255.0,cv::THRESH_BINARY);
    Mat Blur1;
    Mat Blur2;
    cv::GaussianBlur(bin1,Blur1,GaussSize,sigma1,sigma2);
    cv::GaussianBlur(bin2,Blur2,GaussSize,sigma1,sigma2);
    cv::threshold(Blur1,bin1,binTh,255.0,cv::THRESH_BINARY);
    cv::threshold(Blur2,bin2,binTh,255.0,cv::THRESH_BINARY);
    Pts2=prevPts;
    calcOpticalFlowPyrLK(bin1,bin2,prevPts,Pts2,good,err,winsize,maxLevel,criteria,derivLambda,flags,minEigTh);

    cv::Mat fundemental;

    std::vector<Point2f> goodPts1;
    std::vector<Point2f> goodPts2;
    std::vector<int> tempIdx;
    int ind=0;
    for(int i=0;i<prevPts.size();i++)
    {
       if(good.at(i))
       {
            goodPts1.push_back(prevPts.at(i));
            goodPts2.push_back(Pts2.at(i));
            tempIdx.push_back(ind);
            ind++;
       }
    }
    if(goodPts1.size()<8)
    	return false;
    std::vector<uchar> inliers(goodPts1.size(),0);
    //double confidence=0.99;
    //double distance=3.0;
    fundemental= cv::findFundamentalMat(
           cv::Mat(goodPts1),cv::Mat(goodPts2), // matching points
            inliers,       // match status (inlier or outlier)
            CV_FM_RANSAC, // RANSAC method
            FundDist,      // distance to epipolar line
            FundConfi);

    std::vector<Point2f> newTrackPt;
    finalGoodIndex.clear();
    for(int i=0;i<goodPts2.size();i++)
    {
        if(inliers.at(i))
        {
        	//circle(*pMatRgb,goodPts2.at(i),3,CV_RGB(0,0,255));
        	newTrackPt.push_back(goodPts2.at(i));
        	finalGoodIndex.push_back(tempIdx.at(i));
        }
    }
    nextPts.clear();
    nextPts=newTrackPt;
    /*
    currentImg=gray.clone();
    if(trackPts.size()<30)
    {
    	TrackFirst=true;
    }
    */
    return true;
}

bool trackFeatureInKFByNewOpticalflow(KeyFrame & KF,Mat & currentFrame,std::vector<Point3f> & select3f,std::vector<Point2f> & select2f,CVStuff & mCV,double binTh,Size GaussSize,double sigma1,double sigma2, Size winsize, int maxLevel,TermCriteria criteria,double derivLambda , int flags, double minEigTh,double FundConfi, double FundDist)
{
	std::vector<int> finalGoodIndex;
	if(newLKTrack(KF.image,currentFrame,KF.imgFeatures, select2f,finalGoodIndex,binTh, GaussSize,sigma1,sigma2,winsize,maxLevel,criteria,derivLambda,flags,minEigTh,FundConfi,FundDist))
	{
		if(select2f.size()<4||finalGoodIndex.size()<4)
		{
			__android_log_print(ANDROID_LOG_ERROR,"no enough points for KF pose"," select2f size: %d, finalGoodIndex size: %d",select2f.size(),finalGoodIndex.size());
			return false;
		}

		select3f.clear();
		for(int i=0;i<finalGoodIndex.size();i++)
			select3f.push_back(KF.sWPts.at(finalGoodIndex.at(i)));
	}
	else
	{
		__android_log_print(ANDROID_LOG_ERROR,"new KF track","new KF LK track fail");
		for(int i=0;i<KF.imgFeatures.size();i++)
		{
			__android_log_print(ANDROID_LOG_ERROR,"new KF track","new KF Img Feature, x: %f, y: %f",KF.imgFeatures.at(i).x,KF.imgFeatures.at(i).y);
		}
		if(KF.imgFeatures.size()<8)
			KF.valid=false;
		return false;
	}

//__android_log_print(ANDROID_LOG_INFO,"Find match pt in KF by LK","GO");
if(KF.imgFeatures.size()<1)
{
	__android_log_print(ANDROID_LOG_ERROR,"test KF","no Features");
	KF.valid=false;
	return false;
}

/*
__android_log_print(ANDROID_LOG_INFO,"Find Pose By KF"," FO");
FindCamPoseByKF(mCV.camera_mat,mCV.distort_mat,select3f,select2f,mCV.KFRot, mCV.KFTrans);//compute camera pose
__android_log_print(ANDROID_LOG_INFO,"Find Pose By KF"," finish, update image and pts in KF");
KF.updateKeyFrame(select2f,select3f,currentFrame);//store good 2D feature in current frame and their corresponding 3D coordinates
return true;
*/

if(FindCamPoseByKFRANSAC(mCV.camera_mat,mCV.distort_mat,select3f,select2f,mCV.KFRot, mCV.KFTrans))
{
__android_log_print(ANDROID_LOG_INFO,"Find Pose By KF"," finish, update image and pts in KF");
KF.updateKeyFrame(select2f,select3f,currentFrame);//store good 2D feature in current frame and their corresponding 3D coordinates
return true;
}
else
	return false;
}
void LinearSTTriangulation(int numViews,CvMat* & cam_mat,std::vector<CvMat*> &Rots, std::vector<CvMat*> & Trans,std::vector<CvMat*> & ProjMat, std::vector<std::vector<Point2f> > & imgPts, std::vector<Point3f> & WPts)
{
    if(numViews!=Rots.size())
    {
        std::cout<<"Rot dimension wrong"<<std::endl;
        return;
    }
    if(numViews!=Trans.size())
    {
        std::cout<<"Trans dimension wrong"<<std::endl;
        return;
    }
    if(numViews!=imgPts.size())
    {
        std::cout<<"imgPts dimension wrong"<<std::endl;
        return;
    }

    //std::vector<CvMat*> ProjMat;
    for(int i=0;i<ProjMat.size();i++)
        cvReleaseMat(&ProjMat.at(i));
    ProjMat.clear();

    ProjMat.resize(numViews);
    CvMat* tempProj=cvCreateMat(3,4,CV_32FC1);
    CvMat* Rot33=cvCreateMat(3,3,CV_32FC1);
    for(int i=0;i<numViews;i++)
    {
        cvRodrigues2(Rots.at(i),Rot33);
        ProjMat.at(i)=cvCreateMat(3,4,CV_32FC1);
        for(int j=0;j<3;j++)
        {
            for(int k=0;k<3;k++)
            {
                 CV_MAT_ELEM(*tempProj,float,j,k)= CV_MAT_ELEM(*Rot33,float,j,k);
            }
        }
        CV_MAT_ELEM(*tempProj,float,0,3)=CV_MAT_ELEM(*Trans.at(i),float,0,0);
        CV_MAT_ELEM(*tempProj,float,1,3)=CV_MAT_ELEM(*Trans.at(i),float,0,1);
        CV_MAT_ELEM(*tempProj,float,2,3)=CV_MAT_ELEM(*Trans.at(i),float,0,2);
        cvGEMM(cam_mat,tempProj,1.0,NULL,0.0,ProjMat.at(i),0);
    }
    //int MatArows=2*numViews;
    int MatArows=3*numViews;
    int MatAcols=4;
    int numPts=imgPts.at(0).size();
    CvMat* MatA=cvCreateMat(MatArows,MatAcols,CV_32FC1);
    CvMat* W=cvCreateMat(MatArows,MatAcols,CV_32FC1);
    CvMat* U=cvCreateMat(MatArows,MatArows,CV_32FC1);
    CvMat* V=cvCreateMat(MatAcols,MatAcols,CV_32FC1);
    WPts.clear();
    for(int PtInd=0;PtInd<numPts;PtInd++)
    {
        for(int imgInd=0;imgInd<numViews;imgInd++)
        {
            float u=imgPts.at(imgInd).at(PtInd).x;
            float v=imgPts.at(imgInd).at(PtInd).y;
            CV_MAT_ELEM(*MatA,float,3*imgInd,0)=u*CV_MAT_ELEM(*ProjMat.at(imgInd),float,2,0)-CV_MAT_ELEM(*ProjMat.at(imgInd),float,0,0);//uP20-P00
            CV_MAT_ELEM(*MatA,float,3*imgInd+1,0)=v*CV_MAT_ELEM(*ProjMat.at(imgInd),float,2,0)-CV_MAT_ELEM(*ProjMat.at(imgInd),float,1,0);//vP20-P10
            CV_MAT_ELEM(*MatA,float,3*imgInd+2,0)=u*CV_MAT_ELEM(*ProjMat.at(imgInd),float,1,0)-v*CV_MAT_ELEM(*ProjMat.at(imgInd),float,0,0);//uP10-vP00

            CV_MAT_ELEM(*MatA,float,3*imgInd,1)=u*CV_MAT_ELEM(*ProjMat.at(imgInd),float,2,1)-CV_MAT_ELEM(*ProjMat.at(imgInd),float,0,1);//uP21-P01
            CV_MAT_ELEM(*MatA,float,3*imgInd+1,1)=v*CV_MAT_ELEM(*ProjMat.at(imgInd),float,2,1)-CV_MAT_ELEM(*ProjMat.at(imgInd),float,1,1);//vP21-P11
            CV_MAT_ELEM(*MatA,float,3*imgInd+2,1)=u*CV_MAT_ELEM(*ProjMat.at(imgInd),float,1,1)-v*CV_MAT_ELEM(*ProjMat.at(imgInd),float,0,1);//uP11-vP01

            CV_MAT_ELEM(*MatA,float,3*imgInd,2)=u*CV_MAT_ELEM(*ProjMat.at(imgInd),float,2,2)-CV_MAT_ELEM(*ProjMat.at(imgInd),float,0,2);//uP22-P02
            CV_MAT_ELEM(*MatA,float,3*imgInd+1,2)=v*CV_MAT_ELEM(*ProjMat.at(imgInd),float,2,2)-CV_MAT_ELEM(*ProjMat.at(imgInd),float,1,2);//vP22-P12
            CV_MAT_ELEM(*MatA,float,3*imgInd+2,2)=u*CV_MAT_ELEM(*ProjMat.at(imgInd),float,1,2)-v*CV_MAT_ELEM(*ProjMat.at(imgInd),float,0,2);//uP12-vP02

            CV_MAT_ELEM(*MatA,float,3*imgInd,3)=u*CV_MAT_ELEM(*ProjMat.at(imgInd),float,2,3)-CV_MAT_ELEM(*ProjMat.at(imgInd),float,0,3);//uP23-P03
            CV_MAT_ELEM(*MatA,float,3*imgInd+1,3)=v*CV_MAT_ELEM(*ProjMat.at(imgInd),float,2,3)-CV_MAT_ELEM(*ProjMat.at(imgInd),float,1,3);//vP23-P13
            CV_MAT_ELEM(*MatA,float,3*imgInd+2,3)=u*CV_MAT_ELEM(*ProjMat.at(imgInd),float,1,3)-v*CV_MAT_ELEM(*ProjMat.at(imgInd),float,0,3);//uP13-vP03
        }


        cvSVD(MatA,W,U,V,CV_SVD_V_T);


        float X= CV_MAT_ELEM(*V,float,3,0);
        float Y= CV_MAT_ELEM(*V,float,3,1);
        float Z= CV_MAT_ELEM(*V,float,3,2);
        float WW= CV_MAT_ELEM(*V,float,3,3);

        /*
        float X= CV_MAT_ELEM(*V,float,0,3);
        float Y= CV_MAT_ELEM(*V,float,1,3);
        float Z= CV_MAT_ELEM(*V,float,2,3);
        float WW= CV_MAT_ELEM(*V,float,3,3);
        */
        X=X/WW;Y=Y/WW;Z=Z/WW;
        Point3f p;
        p.x=X;
        p.y=Y;
        p.z=Z;
        WPts.push_back(p);
    }
    cvReleaseMat(&MatA);
    cvReleaseMat(&U);
    cvReleaseMat(&V);
    cvReleaseMat(&W);
    cvReleaseMat(&tempProj);
    cvReleaseMat(&Rot33);
}
void LinearSTTriangulation(int numViews,CvMat* & cam_mat,std::vector<CvMat*> &Rots, std::vector<CvMat*> & Trans, std::vector<std::vector<Point2f> > & imgPts, std::vector<Point3f> & WPts)
{
	 std::vector<CvMat*> ProjMat;
	 LinearSTTriangulation(numViews,cam_mat,Rots,Trans,ProjMat,imgPts,WPts);
	 /*
	    for(int i=0;i<WPts.size();i++)
	    {
	    	__android_log_print(ANDROID_LOG_INFO,"Trianglulated Pts"," WPts: x: %f, y: %f, z: %f",WPts.at(i).x,WPts.at(i).y,WPts.at(i).z);
	    }
	    */
}
void IterativeLinearSTTriangulation(int numViews,CvMat* & cam_mat,std::vector<CvMat*> &Rots, std::vector<CvMat*> & Trans, std::vector<std::vector<Point2f> > & imgPts, std::vector<Point3f> & WPts)
{
    std::vector<std::vector<float> >wi;
    int numPts=imgPts.at(0).size();
    for(int i=0;i<numViews;i++)
    {
        std::vector<float> twi;
        for(int j=0;j<numPts;j++)
            twi.push_back(1.0);
        wi.push_back(twi);
    }

    std::vector<Point3f> tWPts;
    std::vector<CvMat*> ProjMat;
    float eps=10E-4;
    int MatArows=2*numViews;
    int MatAcols=4;

    CvMat* MatA=cvCreateMat(MatArows,MatAcols,CV_32FC1);
    CvMat* W=cvCreateMat(MatArows,MatAcols,CV_32FC1);
    CvMat* U=cvCreateMat(MatArows,MatArows,CV_32FC1);
    CvMat* V=cvCreateMat(MatAcols,MatAcols,CV_32FC1);
    LinearSTTriangulation(numViews,cam_mat,Rots,Trans,ProjMat,imgPts,tWPts);
    for(int mainLoop=0;mainLoop<30;mainLoop++)
    {
        int lessEps=0;
        for(int imgInd=0;imgInd<numViews;imgInd++)//update wi
        {
            for(int ptInd=0;ptInd<numPts;ptInd++)
            {
                Point3f p=tWPts.at(ptInd);
                float tw=CV_MAT_ELEM(*ProjMat.at(imgInd),float,2,0)*p.x+CV_MAT_ELEM(*ProjMat.at(imgInd),float,2,1)*p.y+CV_MAT_ELEM(*ProjMat.at(imgInd),float,2,2)*p.z+CV_MAT_ELEM(*ProjMat.at(imgInd),float,2,3);
                if(fabs(tw-wi.at(imgInd).at(ptInd))<eps)
                {
                    lessEps++;
                    continue;
                }
                wi.at(imgInd).at(ptInd)=tw;
            }
        }
        if(lessEps==numViews*numPts)
            break;

        for(int PtInd=0;PtInd<numPts;PtInd++)//update WPt
        {
            for(int imgInd=0;imgInd<numViews;imgInd++)
            {
                float u=imgPts.at(imgInd).at(PtInd).x;
                float v=imgPts.at(imgInd).at(PtInd).y;
                float www=wi.at(imgInd).at(PtInd);
                CV_MAT_ELEM(*MatA,float,2*imgInd,0)=u*CV_MAT_ELEM(*ProjMat.at(imgInd),float,2,0)/www-CV_MAT_ELEM(*ProjMat.at(imgInd),float,0,0)/www;//uP20-P00
                CV_MAT_ELEM(*MatA,float,2*imgInd+1,0)=v*CV_MAT_ELEM(*ProjMat.at(imgInd),float,2,0)/www-CV_MAT_ELEM(*ProjMat.at(imgInd),float,1,0)/www;//vP20-P10
                //CV_MAT_ELEM(*MatA,float,3*imgInd+2,0)=u*CV_MAT_ELEM(*ProjMat.at(imgInd),float,1,0)-v*CV_MAT_ELEM(*ProjMat.at(imgInd),float,0,0);//uP10-vP00

                CV_MAT_ELEM(*MatA,float,2*imgInd,1)=u*CV_MAT_ELEM(*ProjMat.at(imgInd),float,2,1)/www-CV_MAT_ELEM(*ProjMat.at(imgInd),float,0,1)/www;//uP21-P01
                CV_MAT_ELEM(*MatA,float,2*imgInd+1,1)=v*CV_MAT_ELEM(*ProjMat.at(imgInd),float,2,1)/www-CV_MAT_ELEM(*ProjMat.at(imgInd),float,1,1)/www;//vP21-P11
                //CV_MAT_ELEM(*MatA,float,3*imgInd+2,1)=u*CV_MAT_ELEM(*ProjMat.at(imgInd),float,1,1)-v*CV_MAT_ELEM(*ProjMat.at(imgInd),float,0,1);//uP11-vP01

                CV_MAT_ELEM(*MatA,float,2*imgInd,2)=u*CV_MAT_ELEM(*ProjMat.at(imgInd),float,2,2)/www-CV_MAT_ELEM(*ProjMat.at(imgInd),float,0,2)/www;//uP22-P02
                CV_MAT_ELEM(*MatA,float,2*imgInd+1,2)=v*CV_MAT_ELEM(*ProjMat.at(imgInd),float,2,2)/www-CV_MAT_ELEM(*ProjMat.at(imgInd),float,1,2)/www;//vP22-P12
                //CV_MAT_ELEM(*MatA,float,3*imgInd+2,2)=u*CV_MAT_ELEM(*ProjMat.at(imgInd),float,1,2)-v*CV_MAT_ELEM(*ProjMat.at(imgInd),float,0,2);//uP12-vP02

                CV_MAT_ELEM(*MatA,float,2*imgInd,3)=u*CV_MAT_ELEM(*ProjMat.at(imgInd),float,2,3)/www-CV_MAT_ELEM(*ProjMat.at(imgInd),float,0,3)/www;//uP23-P03
                CV_MAT_ELEM(*MatA,float,2*imgInd+1,3)=v*CV_MAT_ELEM(*ProjMat.at(imgInd),float,2,3)/www-CV_MAT_ELEM(*ProjMat.at(imgInd),float,1,3)/www;//vP23-P13
                //CV_MAT_ELEM(*MatA,float,3*imgInd+2,3)=u*CV_MAT_ELEM(*ProjMat.at(imgInd),float,1,3)-v*CV_MAT_ELEM(*ProjMat.at(imgInd),float,0,3);//uP13-vP03
            }


            cvSVD(MatA,W,U,V,CV_SVD_V_T);


            float X= CV_MAT_ELEM(*V,float,3,0);
            float Y= CV_MAT_ELEM(*V,float,3,1);
            float Z= CV_MAT_ELEM(*V,float,3,2);
            float WW= CV_MAT_ELEM(*V,float,3,3);

            /*
            float X= CV_MAT_ELEM(*V,float,0,3);
            float Y= CV_MAT_ELEM(*V,float,1,3);
            float Z= CV_MAT_ELEM(*V,float,2,3);
            float WW= CV_MAT_ELEM(*V,float,3,3);
            */
            X=X/WW;Y=Y/WW;Z=Z/WW;
            Point3f p;
            p.x=X;
            p.y=Y;
            p.z=Z;
            tWPts.at(PtInd)=p;
        }
    }
    WPts.clear();
    WPts=tWPts;


    for(int i=0;i<WPts.size();i++)
    {
    	__android_log_print(ANDROID_LOG_INFO,"Trianglulated Pts"," WPts: x: %f, y: %f, z: %f",WPts.at(i).x,WPts.at(i).y,WPts.at(i).z);
    }

    cvReleaseMat(&MatA);
    cvReleaseMat(&U);
    cvReleaseMat(&V);
    cvReleaseMat(&W);
    for(int i=0;i<ProjMat.size();i++)
        cvReleaseMat(&ProjMat.at(i));

}
int newTriangulationFrom2Views(CvMat* & cam_mat,CvMat* & distort_mat,CvMat* & firstRot,CvMat* & secRot, CvMat* & firstTrans,CvMat* & secTrans,std::vector<Point2f> & imgPts1,std::vector<Point2f> & imgPts2, std::vector<Point2f> & matchPts,std::vector<Point3f> & WPts)
{
	int numViews=2;
	if(imgPts1.size()!=imgPts2.size())
		return 0;
	std::vector<CvMat*> Rots;
	std::vector<CvMat*> Trans;
	std::vector<std::vector<Point2f> > imgPts;
	Rots.push_back(firstRot);
	Rots.push_back(secRot);
	Trans.push_back(firstTrans);
	Trans.push_back(secTrans);
	imgPts.push_back(imgPts1);
	imgPts.push_back(imgPts2);
	std::vector<Point3f> tWPts;
	//IterativeLinearSTTriangulation(numViews,cam_mat,Rots,Trans,imgPts,tWPts);
	LinearSTTriangulation(numViews,cam_mat,Rots,Trans,imgPts,tWPts);
	std::vector<float> error;
	int ptSize=imgPts1.size();
	error.resize(ptSize);
	for(int i=0;i<ptSize;i++)
		error.at(i)=0.0;
	std::vector<Point2f> result;
	for(int i=0;i<numViews;i++)
	{
		result.clear();
		projectPoints(tWPts,Rots.at(i),Trans.at(i),cam_mat,distort_mat,result);
		for(int j=0;j<ptSize;j++)
		{
			float x=result.at(j).x-imgPts.at(i).at(j).x;
			float y=result.at(j).y-imgPts.at(i).at(j).y;
			error.at(j)+=sqrt(x*x+y*y);
		}
	}
	float threshold=10.0f;
	float err;
	matchPts.clear();
	WPts.clear();
	for(int i=0;i<ptSize;i++)
	{
		err=error.at(i)/(float)numViews;
		if(err<threshold)
		{
			matchPts.push_back(imgPts2.at(i));
			WPts.push_back(tWPts.at(i));
			__android_log_print(ANDROID_LOG_INFO,"Trianglulated Pts"," imgPts: x: %f, y: %f",imgPts2.at(i).x,imgPts2.at(i).y);
		    __android_log_print(ANDROID_LOG_INFO,"Trianglulated Pts"," WPts: x: %f, y: %f, z: %f",tWPts.at(i).x,tWPts.at(i).y,tWPts.at(i).z);
		}
	}
	return matchPts.size();
}
bool FindCamPoseByKFRANSAC(CvMat* camera_matrix,CvMat* distortion_coeffs,std::vector<Point3f> & Pt3Ds, std::vector<Point2f> & imgPts,CvMat* & rot, CvMat* & trans)
{
	int PtSize=Pt3Ds.size();
	CvMat* realPts=cvCreateMat(4,3,CV_32FC1);
	CvMat* imgpts=cvCreateMat(4,2,CV_32FC1);
	std::vector<int> index;
	for(int i=0;i<PtSize;i++)
		index.push_back(i);
	//std::vector<int>::iterator it;
	float goodRatio=0.6;
	for(int mainRand=0;mainRand<10;mainRand++)
	{

#ifdef DEBUG
	 __android_log_print(ANDROID_LOG_INFO,"FindCamPoseByKF","OK1");
#endif
	 std::random_shuffle(index.begin(),index.end());
	 for(int i=0;i<4;i++)
	{
		Point3f p3=Pt3Ds.at(i);
		CV_MAT_ELEM(*realPts,float,i,0)=Pt3Ds.at(index.at(i)).x;
		CV_MAT_ELEM(*realPts,float,i,1)=Pt3Ds.at(index.at(i)).y;
		CV_MAT_ELEM(*realPts,float,i,2)=Pt3Ds.at(index.at(i)).z;
		// __android_log_print(ANDROID_LOG_INFO,"FindCamPoseByKF","OK2");
		CV_MAT_ELEM(*imgpts,float,i,0)=(float)imgPts.at(index.at(i)).x;
		CV_MAT_ELEM(*imgpts,float,i,1)=(float)imgPts.at(index.at(i)).y;
		 //__android_log_print(ANDROID_LOG_INFO,"FindCamPoseByKF","OK3");
	}
	 cvFindExtrinsicCameraParams2 (realPts,imgpts, camera_matrix, distortion_coeffs, rot, trans);
	 std::vector<Point2f> result;
	 projectPoints(Pt3Ds,rot,trans,camera_matrix,distortion_coeffs,result);
	 int count=0;
	 float eps=15.0;
	 for(int i=0;i<PtSize;i++)
	 {
		 float x1=imgPts.at(i).x;
		 float y1=imgPts.at(i).y;
		 float x2=result.at(i).x;
		 float y2=result.at(i).y;
		 float x=x1-x2;
		 float y=y1-y2;
		 if(sqrt(x*x+y*y)<eps)
				 count++;
	 }

	 if((float)count>goodRatio*(float)PtSize)
	{
			cvReleaseMat(&realPts);
			cvReleaseMat(&imgpts);
			__android_log_print(ANDROID_LOG_INFO,"OK","FindCamPoseByKFRANSAC");
			return true;
	}
	}

	__android_log_print(ANDROID_LOG_ERROR,"NOOOOO","FindCamPoseByKFRANSAC");
#ifdef DEBUG
	__android_log_print(ANDROID_LOG_INFO,"FindCamPoseByKF","OK3-1");
#endif

#ifdef DEBUG
	__android_log_print(ANDROID_LOG_INFO,"FindCamPoseByKF","OK4");
#endif
	cvReleaseMat(&realPts);
	cvReleaseMat(&imgpts);
	return false;
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
            //cv::circle(colorFrame,p,3,CV_RGB(255,255,0));
            result.push_back(p);
    }

    cvReleaseMat(&point3);
    cvReleaseMat(&point2);
}
