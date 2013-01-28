#include "functions.h"
void Point2fVectorToCvMat(std::vector<Point2f> & srcPt, CvMat* & dstPt,bool rowMajor)
{
    int Size=srcPt.size();
    if(dstPt->data.ptr!=NULL)
        cvReleaseMat(&dstPt);
    if(rowMajor)
    {
        dstPt=cvCreateMat(2,Size*2,CV_32FC1);
        for(int i=0;i<Size;i++)
        {
            CV_MAT_ELEM(*dstPt,float,0,i*2)=srcPt.at(i).x;
            CV_MAT_ELEM(*dstPt,float,1,i*2+1)=srcPt.at(i).y;
        }
    }
    else
    {
        dstPt=cvCreateMat(Size*2,2,CV_32FC1);
        for(int i=0;i<Size;i++)
        {
            CV_MAT_ELEM(*dstPt,float,i*2,0)=srcPt.at(i).x;
            CV_MAT_ELEM(*dstPt,float,i*2+1,1)=srcPt.at(i).y;
        }
    }
}

float showAngle(Point3f center, Point3f p1, Point3f p2)
{
    Point3f v1=p1-center;
    Point3f v2=p2-center;
    normalizeP3f(v1);
    normalizeP3f(v2);
    return acos(v1.dot(v2));

}
float P3fLength(Point3f p)
{
    float x=p.x;
    float y=p.y;
    float z=p.z;
    return sqrt(x*x+y*y+z*z);
}
void normalizeP3f(Point3f & p)
{
    float length=P3fLength(p);
    p.x=p.x/length;
    p.y=p.y/length;
    p.z=p.z/length;
}
float P3fDist(Point3f p1,Point3f p2)
{
    return P3fLength(p2-p1);
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
    cvReleaseMat(&MatA);
    cvReleaseMat(&U);
    cvReleaseMat(&V);
    cvReleaseMat(&W);
    for(int i=0;i<ProjMat.size();i++)
        cvReleaseMat(&ProjMat.at(i));

}

bool RANSACTriangulation(MS & mMS,CvMat* & cam_mat, int rounds)
{

    int numImgs=mMS.numImgs;
    int triangulationImgs=4;
    std::vector<CvMat*> tRot;tRot.resize(triangulationImgs);
    std::vector<CvMat*> tTrans;tTrans.resize(triangulationImgs);
    for(int i=0;i<triangulationImgs;i++)
    {
        tRot.at(i)=cvCreateMat(1,3,CV_32FC1);
        tTrans.at(i)=cvCreateMat(1,3,CV_32FC1);
    }
    std::vector<std::vector<Point2f> > tImgPts;tImgPts.resize(triangulationImgs);
    std::vector<Point3f> tWPts;
    std::vector<int> rest;
    std::vector<int> chosen;
    for(int i=0;i<numImgs;i++)
        rest.push_back(i);

    std::vector<int> restoreRest;
    restoreRest=rest;
    srand(time(NULL));
    int randSize;
    int choose;
    bool getGoodResult=false;
    float buildTime;
    bool getOneGoodResult=false;
    int cBuildTime=9999;
    float cAngleError=9999;
    float cEdgeError=9999;
    float errorAllowRatio=0.3f;
    for(int mainRANSAC=0;mainRANSAC<rounds;mainRANSAC++)
    {
    randSize=numImgs;
    rest.clear();
    rest=restoreRest;
    chosen.clear();
    buildTime=0;

    for(int i=0;i<triangulationImgs;i++)
    {
        choose=rand()%randSize;
        int currentChose=rest.at(choose);
        chosen.push_back(currentChose);
        std::vector<int> temp;
        for(int j=0;j<rest.size();j++)
        {
            if(rest.at(j)==currentChose)
                continue;
            temp.push_back(rest.at(j));
        }
        rest.clear();
        rest=temp;
        randSize=rest.size();
    }
    for(int i=0;i<chosen.size();i++)
    {
        int index=chosen.at(i);
        tRot.at(i)=cvCloneMat(mMS.WorldRots.at(index));
        tTrans.at(i)=cvCloneMat(mMS.WorldTranss.at(index));
        tImgPts.at(i)=mMS.imgPts.at(index);
        buildTime+=mMS.oldestTime.at(index);
    }

    buildTime=buildTime/(float)chosen.size();
    float angleThreshold=5.0*(1+errorAllowRatio*buildTime);
    float angleUpThreshold=(90.0+angleThreshold)/180.0*3.1415926535;
    float angleDownThreshold=(90.0-angleThreshold)/180.0*3.1415926535;
    float edgeThreshold=7.0*(1+errorAllowRatio*buildTime);
    float edgeUpThreshold=90.0+edgeThreshold;
    float edgeDownThreshold=90.0-edgeThreshold;
    IterativeLinearSTTriangulation(triangulationImgs,cam_mat,tRot, tTrans, tImgPts, mMS.WPts);
    bool passAngle=true;
    bool passEdge=true;
    for(int i=0;i<4;i++)
    {
        float angle=mMS.showAngleForOneCorner(i);
        if(angle>angleUpThreshold||angle<angleDownThreshold)
            passAngle=false;
        float dist=mMS.showEdgeDist(i%4,(i+1)%4);
        if(dist>edgeUpThreshold||dist<edgeDownThreshold)
            passEdge=false;
    }
    if(passAngle&&passEdge)
    {
        getOneGoodResult=true;

        float tEdgeError=0.0;
        float tAngleError=0.0;
        for(int i=0;i<4;i++)
        {
            float angle=mMS.showAngleForOneCorner(i)-1.5703;
            tAngleError+=angle*angle;
            float dist=mMS.showEdgeDist(i%4,(i+1)%4)-90.0;
            tEdgeError+=dist*dist;
        }
        tAngleError=tAngleError/4.0;
        tEdgeError=tEdgeError/4.0;
        tAngleError=sqrt(tAngleError);
        tEdgeError=sqrt(tEdgeError);
        if(tAngleError<cAngleError&&tEdgeError<cEdgeError)
        {
            cAngleError=tAngleError;
            cEdgeError=tEdgeError;
            tWPts.clear();
            tWPts=mMS.WPts;
            cBuildTime=(int)(buildTime+0.5)+1;
        }
    }

    }
    if(getOneGoodResult)
    {

        if(cAngleError<mMS.currentBuildAngleError&&cEdgeError<mMS.currentBuildEdgeError)
        {
            std::cout<<"RANSAC got good result"<<std::endl;
            mMS.valid=true;
            mMS.buildTime=cBuildTime;
            mMS.WPts.clear();
            mMS.WPts=tWPts;
            mMS.BuildWPt2();
            std::cout<<"build with buildTime: "<<mMS.buildTime<<std::endl;
            mMS.currentBuildAngleError=cAngleError;
            mMS.currentBuildEdgeError=cEdgeError;
        }
    }
    else
        std::cout<<"NO good result"<<std::endl;

    return getGoodResult;

}
ptrdiff_t myrandom (ptrdiff_t i) {return rand()%i;}
bool RANSACTriangulation(int numSampleImg,MS & mMS,CvMat* & cam_mat)//check by rectangle requirement
{
    int numImgs=mMS.numImgs;
    int triangulationImgs=numSampleImg;
    std::vector<CvMat*> tRot;tRot.resize(triangulationImgs);
    std::vector<CvMat*> tTrans;tTrans.resize(triangulationImgs);
    for(int i=0;i<triangulationImgs;i++)
    {
        tRot.at(i)=cvCreateMat(1,3,CV_32FC1);
        tTrans.at(i)=cvCreateMat(1,3,CV_32FC1);
    }
    std::vector<std::vector<Point2f> > tImgPts;tImgPts.resize(triangulationImgs);
    std::vector<Point3f> tWPts;
    std::vector<int> rest;
    std::vector<int> chosen;
    for(int i=0;i<numImgs;i++)
        rest.push_back(i);

    std::vector<int> restoreRest;
    restoreRest=rest;
    srand(time(NULL));
    int randSize;
    int choose;
    bool getGoodResult=false;
    float buildTime;
    bool getOneGoodResult=false;
    int cBuildTime=9999;
    float cAngleError=9999;
    float cEdgeError=9999;
    float errorAllowRatio=0.3f;
    int from=numImgs;
    int Numchoose=numSampleImg;
    int rounds;
    int tf=from;
    int tNc=Numchoose;
    for(int i=0;i<numSampleImg-1;i++)
    {
    	from=from*(tf-1);
    	Numchoose=Numchoose*(tNc-1);
    	tf--;
    	tNc--;
    }
    rounds=from/Numchoose;
    for(int mainRANSAC=0;mainRANSAC<rounds;mainRANSAC++)
    {
    randSize=numImgs;
    rest.clear();
    rest=restoreRest;
    chosen.clear();
    buildTime=0;

    ptrdiff_t (*p_myrandom)(ptrdiff_t)=myrandom;
    std::random_shuffle(rest.begin(),rest.end(),p_myrandom);
    for(int i=0;i<triangulationImgs;i++)
    {
    	chosen.push_back(rest.at(i));
    }
    for(int i=0;i<chosen.size();i++)
    {
        int index=chosen.at(i);
        tRot.at(i)=cvCloneMat(mMS.WorldRots.at(index));
        tTrans.at(i)=cvCloneMat(mMS.WorldTranss.at(index));
        tImgPts.at(i)=mMS.imgPts.at(index);
        buildTime+=mMS.oldestTime.at(index);
    }

    buildTime=buildTime/(float)chosen.size();
    float angleThreshold=5.0*(1+errorAllowRatio*buildTime);
    float angleUpThreshold=(90.0+angleThreshold)/180.0*3.1415926535;
    float angleDownThreshold=(90.0-angleThreshold)/180.0*3.1415926535;
    float edgeThreshold=7.0*(1+errorAllowRatio*buildTime);
    float edgeUpThreshold=90.0+edgeThreshold;
    float edgeDownThreshold=90.0-edgeThreshold;
    IterativeLinearSTTriangulation(triangulationImgs,cam_mat,tRot, tTrans, tImgPts, mMS.WPts);
    bool passAngle=true;
    bool passEdge=true;
    for(int i=0;i<4;i++)
    {
        float angle=mMS.showAngleForOneCorner(i);
        if(angle>angleUpThreshold||angle<angleDownThreshold)
            passAngle=false;
        float dist=mMS.showEdgeDist(i%4,(i+1)%4);
        if(dist>edgeUpThreshold||dist<edgeDownThreshold)
            passEdge=false;
    }
    if(passAngle&&passEdge)
    {
        getOneGoodResult=true;

        float tEdgeError=0.0;
        float tAngleError=0.0;
        for(int i=0;i<4;i++)
        {
            float angle=mMS.showAngleForOneCorner(i)-1.5703;
            tAngleError+=angle*angle;
            float dist=mMS.showEdgeDist(i%4,(i+1)%4)-90.0;
            tEdgeError+=dist*dist;
        }
        tAngleError=tAngleError/4.0;
        tEdgeError=tEdgeError/4.0;
        tAngleError=sqrt(tAngleError);
        tEdgeError=sqrt(tEdgeError);
        if(tAngleError<cAngleError&&tEdgeError<cEdgeError)
        {
            cAngleError=tAngleError;
            cEdgeError=tEdgeError;
            tWPts.clear();
            tWPts=mMS.WPts;
            cBuildTime=(int)(buildTime+0.5)+1;
        }
    }

    }
    if(getOneGoodResult)
    {

        if(cAngleError<mMS.currentBuildAngleError&&cEdgeError<mMS.currentBuildEdgeError)
        {
            std::cout<<"RANSAC got good result"<<std::endl;
            mMS.valid=true;
            mMS.buildTime=cBuildTime;
            mMS.WPts.clear();
            mMS.WPts=tWPts;
            mMS.BuildWPt2();
            std::cout<<"build with buildTime: "<<mMS.buildTime<<std::endl;
            mMS.currentBuildAngleError=cAngleError;
            mMS.currentBuildEdgeError=cEdgeError;
        }
    }
    else
        std::cout<<"NO good result"<<std::endl;

    return getGoodResult;
}
bool RANSACTriangulation(MS & mMS,CvMat* & cam_mat,CvMat* & distort_mat, int rounds)
{

    int numImgs=mMS.numImgs;
    int triangulationImgs=4;
    std::vector<CvMat*> tRot;tRot.resize(triangulationImgs);
    std::vector<CvMat*> tTrans;tTrans.resize(triangulationImgs);
    for(int i=0;i<triangulationImgs;i++)
    {
        tRot.at(i)=cvCreateMat(1,3,CV_32FC1);
        tTrans.at(i)=cvCreateMat(1,3,CV_32FC1);
    }
    std::vector<std::vector<Point2f> > tImgPts;tImgPts.resize(triangulationImgs);
    std::vector<Point3f> tWPts;
    std::vector<int> rest;
    std::vector<int> chosen;
    for(int i=0;i<numImgs;i++)
        rest.push_back(i);

    std::vector<int> restoreRest;
    restoreRest=rest;
    srand(time(NULL));
    int randSize;
    int choose;
    bool getGoodResult=false;
    float buildTime;
    bool getOneGoodResult=false;
    int cBuildTime=9999;
    float errorAllowRatio=0.3f;
#ifdef CHECK_BY_RECT
    float cAngleError=9999;
    float cEdgeError=9999;

#endif
#ifdef CHECK_BY_REPROJECT
    float cReprojError=9999;
#endif
    for(int mainRANSAC=0;mainRANSAC<rounds;mainRANSAC++)
    {
    randSize=numImgs;
    rest.clear();
    rest=restoreRest;
    chosen.clear();
    buildTime=0;

    ptrdiff_t (*p_myrandom)(ptrdiff_t)=myrandom;
    std::random_shuffle(rest.begin(),rest.end(),p_myrandom);
    for(int i=0;i<triangulationImgs;i++)
    {
    	chosen.push_back(rest.at(i));
    }
    for(int i=0;i<chosen.size();i++)
    {
        int index=chosen.at(i);
        tRot.at(i)=cvCloneMat(mMS.WorldRots.at(index));
        tTrans.at(i)=cvCloneMat(mMS.WorldTranss.at(index));
        tImgPts.at(i)=mMS.imgPts.at(index);
        buildTime+=mMS.oldestTime.at(index);
    }

    buildTime=buildTime/(float)chosen.size();
#ifdef CHECK_BY_RECT
    float angleThreshold=5.0*(1+errorAllowRation*buildTime);
    float angleUpThreshold=(90.0+angleThreshold)/180.0*3.1415926535;
    float angleDownThreshold=(90.0-angleThreshold)/180.0*3.1415926535;
    float edgeThreshold=7.0*(1+errorAllowRation*buildTime);
    float edgeUpThreshold=90.0+edgeThreshold;
    float edgeDownThreshold=90.0-edgeThreshold;
#endif
    IterativeLinearSTTriangulation(triangulationImgs,cam_mat,tRot, tTrans, tImgPts, mMS.WPts);
#ifdef CHECK_BY_RECT
    bool passAngle=true;
    bool passEdge=true;
    for(int i=0;i<4;i++)
    {
        float angle=mMS.showAngleForOneCorner(i);
        if(angle>angleUpThreshold||angle<angleDownThreshold)
            passAngle=false;
        float dist=mMS.showEdgeDist(i%4,(i+1)%4);
        if(dist>edgeUpThreshold||dist<edgeDownThreshold)
            passEdge=false;
    }
    if(passAngle&&passEdge)
    {
        getOneGoodResult=true;

        float tEdgeError=0.0;
        float tAngleError=0.0;
        for(int i=0;i<4;i++)
        {
            float angle=mMS.showAngleForOneCorner(i)-1.5703;
            tAngleError+=angle*angle;
            float dist=mMS.showEdgeDist(i%4,(i+1)%4)-90.0;
            tEdgeError+=dist*dist;
        }
        tAngleError=tAngleError/4.0;
        tEdgeError=tEdgeError/4.0;
        tAngleError=sqrt(tAngleError);
        tEdgeError=sqrt(tEdgeError);
        if(tAngleError<cAngleError&&tEdgeError<cEdgeError)
        {
            cAngleError=tAngleError;
            cEdgeError=tEdgeError;
            tWPts.clear();
            tWPts=mMS.WPts;
            cBuildTime=(int)(buildTime+0.5)+1;
        }

    }
#endif

#ifdef CHECK_BY_REPROJECT
    bool passReproj=true;
    std::vector<std::vector<Point2f> > result;
    result.resize(triangulationImgs);
    float reprojThreshold=3.0*(1+errorAllowRation*buildTime);
    float tReprojError=0.0;
    for(int i=0;i<triangulationImgs;i++)
    {
    	result.at(i).resize(4);//4 corners
    	projectPoints(mMS.WPts,tRot.at(i),tTrans.at(i),cam_mat,distort,result.at(i));
    	float reprojError=0.0;
    	for(int j=0;j<4;j++)
    	{
    		float x=tImgPts.at(i).at(j).x-result.at(i).at(j).x;
    		float y=tImgPts.at(i).at(j).y-result.at(i).at(j).y;
    		reprojError+=sqrt(x*x+y*y);
    	}
    	if(reprojError/4.0>reprojThreshold)
    		passReproj=false;
    	else
    		tReprojError+=reprojError/4.0;
    }
    if(passReproj)
    {
    	getOneGoodResult=true;
    	tReprojError=tReprojError/(float)triangulationImgs;
    	if(tReprojError<cReprojError)
    	{
    		cReprojError=tReprojError;
            tWPts.clear();
            tWPts=mMS.WPts;
            cBuildTime=(int)(buildTime+0.5)+1;
    	}
    }
#endif
    }//end main RANSAC Loop
    if(getOneGoodResult)
    {
#ifdef CHECK_BY_RECT
        if(cAngleError<mMS.currentBuildAngleError&&cEdgeError<mMS.currentBuildEdgeError)
        {
            std::cout<<"RANSAC got good result"<<std::endl;
            mMS.valid=true;
            mMS.buildTime=cBuildTime;
            mMS.WPts.clear();
            mMS.WPts=tWPts;
            mMS.BuildWPt2();
            std::cout<<"build with buildTime: "<<mMS.buildTime<<std::endl;
            mMS.currentBuildAngleError=cAngleError;
            mMS.currentBuildEdgeError=cEdgeError;
        }
#endif
#ifdef CHECK_BY_REPROJECT
        if(cReprojError<mMS.currentBuildReprojError)
        {
        	 mMS.valid=true;
        	 mMS.buildTime=cBuildTime;
        	 mMS.WPts.clear();
        	 mMS.WPts=tWPts;
        	 mMS.BuildWPt2();
        	 mMS.currentBuildReprojError=cReprojError;
        }
#endif
    }
    else
        std::cout<<"NO good result"<<std::endl;

    return getGoodResult;

}

bool RANSACTriangulation(int numSampleImg,MS & mMS,CvMat* & cam_mat,CvMat* & distort_mat)//check by reprojection
{
    int numImgs=mMS.numImgs;
    int triangulationImgs=numSampleImg;
    std::vector<CvMat*> tRot;tRot.resize(triangulationImgs);
    std::vector<CvMat*> tTrans;tTrans.resize(triangulationImgs);
    for(int i=0;i<triangulationImgs;i++)
    {
        tRot.at(i)=cvCreateMat(1,3,CV_32FC1);
        tTrans.at(i)=cvCreateMat(1,3,CV_32FC1);
    }
    std::vector<std::vector<Point2f> > tImgPts;tImgPts.resize(triangulationImgs);
    std::vector<Point3f> tWPts;
    std::vector<int> rest;
    std::vector<int> chosen;
    for(int i=0;i<numImgs;i++)
        rest.push_back(i);

    std::vector<int> restoreRest;
    restoreRest=rest;
    srand(time(NULL));
    int randSize;
    int choose;
    bool getGoodResult=false;
    float buildTime;
    bool getOneGoodResult=false;
    int cBuildTime=9999;
    float cReprojError=9999.0f;
    float errorAllowRatio=0.3f;

    int from=numImgs;
    int Numchoose=numSampleImg;
    int rounds;
    int tf=from;
    int tNc=Numchoose;
    for(int i=0;i<numSampleImg-1;i++)
    {
    	from=from*(tf-1);
    	Numchoose=Numchoose*(tNc-1);
    	tf--;
    	tNc--;
    }
    rounds=tf/tNc;
    for(int mainRANSAC=0;mainRANSAC<rounds;mainRANSAC++)
    {
    randSize=numImgs;
    rest.clear();
    rest=restoreRest;
    chosen.clear();
    buildTime=0;

    ptrdiff_t (*p_myrandom)(ptrdiff_t)=myrandom;
    std::random_shuffle(rest.begin(),rest.end(),p_myrandom);
    for(int i=0;i<triangulationImgs;i++)
    {
    	chosen.push_back(rest.at(i));
    }
    for(int i=0;i<chosen.size();i++)
    {
        int index=chosen.at(i);
        tRot.at(i)=cvCloneMat(mMS.WorldRots.at(index));
        tTrans.at(i)=cvCloneMat(mMS.WorldTranss.at(index));
        tImgPts.at(i)=mMS.imgPts.at(index);
        buildTime+=mMS.oldestTime.at(index);
    }
    buildTime=buildTime/(float)chosen.size();
    IterativeLinearSTTriangulation(triangulationImgs,cam_mat,tRot, tTrans, tImgPts, mMS.WPts);
    //check rectangle
    bool passReproj=true;
    std::vector<std::vector<Point2f> > result;
    result.resize(triangulationImgs);
    float reprojThreshold=3.0*(1+errorAllowRatio*buildTime);
    float tReprojError=0.0;
    for(int i=0;i<triangulationImgs;i++)
    {
    	result.at(i).resize(4);//4 corners
    	projectPoints(mMS.WPts,tRot.at(i),tTrans.at(i),cam_mat,distort_mat,result.at(i));
    	float reprojError=0.0;
    	for(int j=0;j<4;j++)
    	{
    		float x=tImgPts.at(i).at(j).x-result.at(i).at(j).x;
    		float y=tImgPts.at(i).at(j).y-result.at(i).at(j).y;
    		reprojError+=sqrt(x*x+y*y);
    	}
    	if(reprojError/4.0>reprojThreshold)
    		passReproj=false;
    	else
    		tReprojError+=reprojError/4.0;
    }
    if(passReproj)
    {
    	getOneGoodResult=true;
    	tReprojError=tReprojError/(float)triangulationImgs;
    	if(tReprojError<cReprojError)
    	{
    		cReprojError=tReprojError;
            tWPts.clear();
            tWPts=mMS.WPts;
            cBuildTime=(int)(buildTime+0.5)+1;
    	}
    }
    }//end main RANSAC Loop
    if(getOneGoodResult)
    {
        if(cReprojError<mMS.currentBuildReprojError)
        {
        	 mMS.valid=true;
        	 mMS.buildTime=cBuildTime;
        	 mMS.WPts.clear();
        	 mMS.WPts=tWPts;
        	 mMS.BuildWPt2();
        	 mMS.currentBuildReprojError=cReprojError;
        }
    }
    else
        std::cout<<"NO good result"<<std::endl;

    return getGoodResult;


}
void RANSACTriangulation2Methods(MS & mMS1,MS & mMS2,CvMat* & cam_mat,CvMat* & distort_mat,bool & MS1good, bool & MS2good, int numImgUse,int rounds)
{
    int numImgs=mMS1.numImgs;
    int triangulationImgs=numImgUse;
    std::vector<CvMat*> tRot1;tRot1.resize(triangulationImgs);
    std::vector<CvMat*> tTrans1;tTrans1.resize(triangulationImgs);
    std::vector<CvMat*> tRot2;tRot2.resize(triangulationImgs);
    std::vector<CvMat*> tTrans2;tTrans2.resize(triangulationImgs);
    for(int i=0;i<triangulationImgs;i++)
    {
        tRot1.at(i)=cvCreateMat(1,3,CV_32FC1);
        tTrans1.at(i)=cvCreateMat(1,3,CV_32FC1);
        tRot2.at(i)=cvCreateMat(1,3,CV_32FC1);
        tTrans2.at(i)=cvCreateMat(1,3,CV_32FC1);
    }
    std::vector<std::vector<Point2f> > tImgPts;tImgPts.resize(triangulationImgs);
    std::vector<Point3f> tWPts1,tWPts2;
    std::vector<int> rest;
    std::vector<int> chosen;
    for(int i=0;i<numImgs;i++)
        rest.push_back(i);

    std::vector<int> restoreRest;
    restoreRest=rest;
    srand(time(NULL));
    int randSize;
    int choose;
    bool getGoodResult1=false;
    bool getGoodResult2=false;
    float buildTime1;
    float buildTime2;
    bool getOneGoodResult1=false;
    bool getOneGoodResult2=false;
    int cBuildTime1=9999;
    int cBuildTime2=9999;
    float errorAllowRatio=0.3f;
    float cAngleError=9999;
    float cEdgeError=9999;
    float cReprojError=9999;
    for(int mainRANSAC=0;mainRANSAC<rounds;mainRANSAC++)
    {
    randSize=numImgs;
    rest.clear();
    rest=restoreRest;
    chosen.clear();
    buildTime1=0;
    buildTime2=0;

    ptrdiff_t (*p_myrandom)(ptrdiff_t)=myrandom;
    std::random_shuffle(rest.begin(),rest.end(),p_myrandom);
    for(int i=0;i<triangulationImgs;i++)
    {
    	chosen.push_back(rest.at(i));
    }
    for(int i=0;i<chosen.size();i++)
    {
        int index=chosen.at(i);
        tRot1.at(i)=cvCloneMat(mMS1.WorldRots.at(index));
        tTrans1.at(i)=cvCloneMat(mMS1.WorldTranss.at(index));
        tRot2.at(i)=cvCloneMat(mMS2.WorldRots.at(index));
        tTrans2.at(i)=cvCloneMat(mMS2.WorldTranss.at(index));
        tImgPts.at(i)=mMS1.imgPts.at(index);
        buildTime1+=mMS1.oldestTime.at(index);
        buildTime2+=mMS2.oldestTime.at(index);
    }

    buildTime1=buildTime1/(float)chosen.size();
    buildTime2=buildTime2/(float)chosen.size();
    float angleThreshold=5.0*(1+errorAllowRatio*buildTime1);
    float angleUpThreshold=(90.0+angleThreshold)/180.0*3.1415926535;
    float angleDownThreshold=(90.0-angleThreshold)/180.0*3.1415926535;
    float edgeThreshold=7.0*(1+errorAllowRatio*buildTime1);
    float edgeUpThreshold=90.0+edgeThreshold;
    float edgeDownThreshold=90.0-edgeThreshold;
    IterativeLinearSTTriangulation(triangulationImgs,cam_mat,tRot1, tTrans1, tImgPts, mMS1.WPts);
    IterativeLinearSTTriangulation(triangulationImgs,cam_mat,tRot1, tTrans1, tImgPts, mMS2.WPts);
    //check rectangle
    bool passAngle=true;
    bool passEdge=true;
    for(int i=0;i<4;i++)
    {
        float angle=mMS1.showAngleForOneCorner(i);
        if(angle>angleUpThreshold||angle<angleDownThreshold)
            passAngle=false;
        float dist=mMS1.showEdgeDist(i%4,(i+1)%4);
        if(dist>edgeUpThreshold||dist<edgeDownThreshold)
            passEdge=false;
    }
    if(passAngle&&passEdge)
    {
        getOneGoodResult1=true;

        float tEdgeError=0.0;
        float tAngleError=0.0;
        for(int i=0;i<4;i++)
        {
            float angle=mMS1.showAngleForOneCorner(i)-1.5703;
            tAngleError+=angle*angle;
            float dist=mMS1.showEdgeDist(i%4,(i+1)%4)-90.0;
            tEdgeError+=dist*dist;
        }
        tAngleError=tAngleError/4.0;
        tEdgeError=tEdgeError/4.0;
        tAngleError=sqrt(tAngleError);
        tEdgeError=sqrt(tEdgeError);
        if(tAngleError<cAngleError&&tEdgeError<cEdgeError)
        {
            cAngleError=tAngleError;
            cEdgeError=tEdgeError;
            tWPts1.clear();
            tWPts1=mMS1.WPts;
            cBuildTime1=(int)(buildTime1+0.5)+1;
        }
    }

    bool passReproj=true;
    std::vector<std::vector<Point2f> > result;
    result.resize(triangulationImgs);
    float reprojThreshold=3.0*(1+errorAllowRatio*buildTime2);
    float tReprojError=0.0;
    for(int i=0;i<triangulationImgs;i++)
    {
    	result.at(i).resize(4);//4 corners
    	projectPoints(mMS2.WPts,tRot2.at(i),tTrans2.at(i),cam_mat,distort_mat,result.at(i));
    	float reprojError=0.0;
    	for(int j=0;j<4;j++)
    	{
    		float x=tImgPts.at(i).at(j).x-result.at(i).at(j).x;
    		float y=tImgPts.at(i).at(j).y-result.at(i).at(j).y;
    		reprojError+=sqrt(x*x+y*y);
    	}
    	if(reprojError/4.0>reprojThreshold)
    		passReproj=false;
    	else
    		tReprojError+=reprojError/4.0;
    }
    if(passReproj)
    {
    	getOneGoodResult2=true;
    	tReprojError=tReprojError/(float)triangulationImgs;
    	if(tReprojError<cReprojError)
    	{
    		cReprojError=tReprojError;
            tWPts2.clear();
            tWPts2=mMS2.WPts;
            cBuildTime2=(int)(buildTime2+0.5)+1;
    	}
    }
    }//end main RANSAC Loop
    if(getOneGoodResult1)
    {
        if(cAngleError<mMS1.currentBuildAngleError&&cEdgeError<mMS1.currentBuildEdgeError)
        {
            std::cout<<"RANSAC got good result"<<std::endl;
            mMS1.valid=true;
            mMS1.buildTime=cBuildTime1;
            mMS1.WPts.clear();
            mMS1.WPts=tWPts1;
            mMS1.BuildWPt2();
            std::cout<<"build with buildTime: "<<mMS1.buildTime<<std::endl;
            mMS1.currentBuildAngleError=cAngleError;
            mMS1.currentBuildEdgeError=cEdgeError;
        }
    }
    if(getOneGoodResult2)
    {
        if(cReprojError<mMS2.currentBuildReprojError)
        {
        	 mMS2.valid=true;
        	 mMS2.buildTime=cBuildTime2;
        	 mMS2.WPts.clear();
        	 mMS2.WPts=tWPts2;
        	 mMS2.BuildWPt2();
        	 mMS2.currentBuildReprojError=cReprojError;
        }
    }
}

#define UNDISTORT
bool RANSACTriangulationByRect(int numSampleImg,MS & mMS,CvMat* & cam_mat,CvMat* &distort_mat)
{
    int numImgs=mMS.numImgs;
    int triangulationImgs=numSampleImg;
    std::vector<CvMat*> tRot;tRot.resize(triangulationImgs);
    std::vector<CvMat*> tTrans;tTrans.resize(triangulationImgs);
    for(int i=0;i<triangulationImgs;i++)
    {
        tRot.at(i)=cvCreateMat(1,3,CV_32FC1);
        tTrans.at(i)=cvCreateMat(1,3,CV_32FC1);
    }
#ifdef UNDISTORT
    std::vector<std::vector<Point2f> > undistortImgPts;
    undistortImgPts.resize(numImgs);
    for(int i=0;i<numImgs;i++)
    {
        UndistortPixel(mMS.imgPts.at(i),undistortImgPts.at(i),cam_mat,distort_mat);
    }
#endif
    std::vector<std::vector<Point2f> > tImgPts;tImgPts.resize(triangulationImgs);
    std::vector<Point3f> tWPts;
    std::vector<int> rest;
    std::vector<int> chosen;
    for(int i=0;i<numImgs;i++)
        rest.push_back(i);

    std::vector<int> restoreRest;
    restoreRest=rest;
    srand(time(NULL));
    int randSize;
    int choose;
    bool getGoodResult=false;
    float buildTime;
    bool getOneGoodResult=false;
    int cBuildTime=9999;
    float cAngleError=9999;
    float cEdgeError=9999;
    float errorAllowRatio=2.0f;
    int from=numImgs;
    int Numchoose=numSampleImg;
    int rounds;
    int tf=from;
    int tNc=Numchoose;
    for(int i=0;i<numSampleImg-1;i++)
    {
        from=from*(tf-1);
        Numchoose=Numchoose*(tNc-1);
        tf--;
        tNc--;
    }
    rounds=from/Numchoose;
    MS tMS;
    tMS.clone(mMS);
    for(int mainRANSAC=0;mainRANSAC<rounds;mainRANSAC++)
    {
    randSize=numImgs;
    rest.clear();
    rest=restoreRest;
    chosen.clear();
    buildTime=0;

    ptrdiff_t (*p_myrandom)(ptrdiff_t)=myrandom;
    std::random_shuffle(rest.begin(),rest.end(),p_myrandom);
    for(int i=0;i<triangulationImgs;i++)
    {
        chosen.push_back(rest.at(i));
    }
    for(int i=0;i<chosen.size();i++)
    {
        int index=chosen.at(i);
        tRot.at(i)=cvCloneMat(tMS.WorldRots.at(index));
        tTrans.at(i)=cvCloneMat(tMS.WorldTranss.at(index));
#ifdef UNDISTORT
        tImgPts.at(i)=undistortImgPts.at(index);
#else
        tImgPts.at(i)=tMS.imgPts.at(index);
#endif
        buildTime+=tMS.oldestTime.at(index);
    }

    buildTime=buildTime/(float)chosen.size();
    float angleThreshold=5.0*(1+errorAllowRatio*buildTime);
    float angleUpThreshold=(90.0+angleThreshold)/180.0*3.1415926535;
    float angleDownThreshold=(90.0-angleThreshold)/180.0*3.1415926535;
    float edgeThreshold=7.0*(1+errorAllowRatio*buildTime);
    float edgeUpThreshold=90.0+edgeThreshold;
    float edgeDownThreshold=90.0-edgeThreshold;
    IterativeLinearSTTriangulation(triangulationImgs,cam_mat,tRot, tTrans, tImgPts, tMS.WPts);
    //check rectangle
    bool passAngle=true;
    bool passEdge=true;
    projMScorners(tMS);
    for(int i=0;i<4;i++)
    {
        float angle=tMS.showAngleForOneCorner(i);
        if(angle>angleUpThreshold||angle<angleDownThreshold)
            passAngle=false;
        float dist=tMS.showEdgeDist(i%4,(i+1)%4);
        if(dist>edgeUpThreshold||dist<edgeDownThreshold)
            passEdge=false;
    }
    if(passAngle&&passEdge)
    {
        getOneGoodResult=true;

        float tEdgeError=0.0;
        float tAngleError=0.0;
        for(int i=0;i<4;i++)
        {
            float angle=mMS.showAngleForOneCorner(i)-1.5703;
            tAngleError+=angle*angle;
            float dist=mMS.showEdgeDist(i%4,(i+1)%4)-90.0;
            tEdgeError+=dist*dist;
        }
        tAngleError=tAngleError/4.0;
        tEdgeError=tEdgeError/4.0;
        tAngleError=sqrt(tAngleError);
        tEdgeError=sqrt(tEdgeError);
        if(tAngleError<cAngleError&&tEdgeError<cEdgeError)
        {
            cAngleError=tAngleError;
            cEdgeError=tEdgeError;
            tWPts.clear();
            tWPts=tMS.WPts;
            cBuildTime=(int)(buildTime+0.5)+1;
        }
    }
    }//end main RANSAC
    if(getOneGoodResult)
    {

        if(cAngleError<mMS.currentBuildAngleError&&cEdgeError<mMS.currentBuildEdgeError)
        {
            std::cout<<"RANSAC got good result"<<std::endl;
            mMS.valid=true;
            mMS.buildTime=cBuildTime;
            mMS.WPts.clear();
#ifdef RECT_RECTIFY
            rectifyRenctangle(tWPts);
#endif
            mMS.WPts=tWPts;
            mMS.BuildWPt2();
            std::cout<<"build with buildTime: "<<mMS.buildTime<<std::endl;
            mMS.currentBuildAngleError=cAngleError;
            mMS.currentBuildEdgeError=cEdgeError;
            getGoodResult=true;
        }
    }
    else
        std::cout<<"NO good result"<<std::endl;

    tMS.release();
    return getGoodResult;
}

bool RANSACTriangulationByReproj(int numSampleImg,MS & mMS,CvMat* & cam_mat,CvMat* & distort_mat)
{
    int numImgs=mMS.numImgs;
    int triangulationImgs=numSampleImg;
    std::vector<CvMat*> tRot;tRot.resize(triangulationImgs);
    std::vector<CvMat*> tTrans;tTrans.resize(triangulationImgs);
    for(int i=0;i<triangulationImgs;i++)
    {
        tRot.at(i)=cvCreateMat(1,3,CV_32FC1);
        tTrans.at(i)=cvCreateMat(1,3,CV_32FC1);
    }
#ifdef UNDISTORT
    std::vector<std::vector<Point2f> > undistortImgPts;
    undistortImgPts.resize(numImgs);
    for(int i=0;i<numImgs;i++)
    {
        UndistortPixel(mMS.imgPts.at(i),undistortImgPts.at(i),cam_mat,distort_mat);
    }
#endif
    std::vector<std::vector<Point2f> > tImgPts;tImgPts.resize(triangulationImgs);
    std::vector<Point3f> tWPts;
    std::vector<int> rest;
    std::vector<int> chosen;
    for(int i=0;i<numImgs;i++)
        rest.push_back(i);

    std::vector<int> restoreRest;
    restoreRest=rest;
    int randSize;
    int choose;
    bool getGoodResult=false;
    float buildTime;
    bool getOneGoodResult=false;
    int cBuildTime=9999;
    float cReprojError=9999.0f;
    float errorAllowRatio=2.0f;

    int from=numImgs;
    int Numchoose=numSampleImg;
    int rounds;
    int tf=from;
    int tNc=Numchoose;
    for(int i=0;i<numSampleImg-1;i++)
    {

        from=from*(tf-1);
        Numchoose=Numchoose*(tNc-1);
        tf--;
        tNc--;

    }
    rounds=from/Numchoose;
    MS tMS;
    tMS.clone(mMS);
    for(int mainRANSAC=0;mainRANSAC<rounds;mainRANSAC++)
    {
    randSize=numImgs;
    rest.clear();
    rest=restoreRest;
    chosen.clear();
    buildTime=0;

    for(int i=0;i<triangulationImgs;i++)
    {

        choose=rand()%randSize;
        int currentChose=rest.at(choose);
        chosen.push_back(currentChose);
        std::vector<int> temp;
        for(int j=0;j<rest.size();j++)
        {
            if(rest.at(j)==currentChose)
                continue;
            temp.push_back(rest.at(j));
        }
        rest.clear();
        rest=temp;
        randSize=rest.size();
        //use shuffle
    }
    for(int i=0;i<chosen.size();i++)
    {
        int index=chosen.at(i);
        tRot.at(i)=cvCloneMat(mMS.WorldRots.at(index));
        tTrans.at(i)=cvCloneMat(mMS.WorldTranss.at(index));
#ifdef UNDISTORT
        tImgPts.at(i)=undistortImgPts.at(index);
#else
        tImgPts.at(i)=mMS.imgPts.at(index);
#endif

        buildTime+=mMS.oldestTime.at(index);
    }

    buildTime=buildTime/(float)chosen.size();
    IterativeLinearSTTriangulation(triangulationImgs,cam_mat,tRot, tTrans, tImgPts, tMS.WPts);
    //check rectangle
    bool passReproj=true;
    std::vector<std::vector<Point2f> > result;
    result.resize(triangulationImgs);

    float reprojThreshold=5.0*(1+errorAllowRatio*buildTime);
    float tReprojError=0.0;
    for(int i=0;i<triangulationImgs;i++)
    {
        result.at(i).resize(4);//4 corners
        projectPoints(tMS.WPts,tRot.at(i),tTrans.at(i),cam_mat,distort_mat,result.at(i));
        float reprojError=0.0;
        for(int j=0;j<4;j++)
        {
            float x=tImgPts.at(i).at(j).x-result.at(i).at(j).x;
            float y=tImgPts.at(i).at(j).y-result.at(i).at(j).y;
            reprojError+=sqrt(x*x+y*y);
        }
        if(reprojError/4.0>reprojThreshold)
            passReproj=false;
        else
            tReprojError+=reprojError/4.0;
    }
    if(passReproj)
    {
        getOneGoodResult=true;
        tReprojError=tReprojError/(float)triangulationImgs;
        if(tReprojError<cReprojError)
        {
            cReprojError=tReprojError;
            tWPts.clear();
            projMScorners(tMS);
            rectifyRectangle(tMS);
            tWPts=tMS.WPts;
            cBuildTime=(int)(buildTime+0.5)+1;
        }
    }
    }//end main RANSAC Loop
    if(getOneGoodResult)
    {
        if(cReprojError<mMS.currentBuildReprojError)
        {

             mMS.valid=true;
             mMS.buildTime=cBuildTime;
             mMS.WPts.clear();
             mMS.WPts=tWPts;
             mMS.BuildWPt2();
             mMS.currentBuildReprojError=cReprojError;
             getGoodResult=true;
        }
    }
    else
    {
    	__android_log_print(ANDROID_LOG_ERROR,"Reproj","NO good result");
    	std::cout<<"NO good result"<<std::endl;
    }

    return getGoodResult;


}
bool UndistortPixel(std::vector<Point2f> & src, std::vector<Point2f> & dst,CvMat* & cam_mat,CvMat* & distort_mat)
{
    float k1, k2, p1, p2;        // distortion values
      float k_radial;
      float x, y;
      float delta_x, delta_y;
      float r_2;                         // radial distance squared
      float distx, disty;          // distored xy
      std::vector<Point2f> old_src = src;  // copy of the original distorted point
      // distortion coefficients
      k1 = CV_MAT_ELEM(*distort_mat,float,0,0);
      k2 = CV_MAT_ELEM(*distort_mat,float,0,1);
      p1 = CV_MAT_ELEM(*distort_mat,float,0,2);
      p2 = CV_MAT_ELEM(*distort_mat,float,0,3);
      int numPts=src.size();
      float cx=CV_MAT_ELEM(*cam_mat,float,0,2);
      float fx=CV_MAT_ELEM(*cam_mat,float,0,0);
      float cy=CV_MAT_ELEM(*cam_mat,float,1,2);
      float fy=CV_MAT_ELEM(*cam_mat,float,1,1);
    dst.clear();
    // Shift points to principal point and use focal length
      for(int i=0;i<numPts;i++)
      {
          Point2f desPnt;
          Point2f dstd;
      dstd.x = (src.at(i).x -  cx) /fx  ;//(src.x-cx)/fx
      dstd.y = (src.at(i).y - cy) /fy;
      desPnt.x = dstd.x;
      desPnt.y = dstd.y;
    // Compensate lens distortion
      x = dstd.x;
      y = dstd.y;
    for(int iter = 0; iter < 5; iter++)

    {
        r_2 = x * x + y * y;
        k_radial = 1 + k1 * r_2 + k2 * r_2 * r_2;
        delta_x = 2 * p1 * x * y + p2 * (r_2 + 2 * x * x);
        delta_y = 2 * p2 * x * y + p1 * (r_2 + 2 * y * y);
        x = (desPnt.x - delta_x) / k_radial;
        y = (desPnt.y - delta_y) / k_radial;
    }
      dstd.x = x;
      dstd.y = y;
      dstd.x *= fx;
      dstd.y *= fy;
      dstd.x += cx;
      dstd.y += cy;
      Point2f tempDst;
      tempDst=dstd;
      dst.push_back(tempDst);
      }
      return true;
}
mPoint projPointToPlane(mPoint & pt,Plane & E)
{
    mPoint res;
    float value=E.params[0]*pt.x+E.params[1]*pt.y+E.params[2]*pt.z+E.params[3];
    float nSq=E.params[0]*E.params[0]+E.params[1]*E.params[1]+E.params[2]*E.params[2];
    float ratio=value/nSq;
    res.x=pt.x-ratio*E.params[0];
    res.y=pt.y-ratio*E.params[1];
    res.z=pt.z-ratio*E.params[2];
    return res;
}

void projMScorners(MS & mMS)
{
    Plane E;
    for(int i=0;i<4;i++)
    {
        E.pts[i].x=mMS.WPts.at(i).x;
        E.pts[i].y=mMS.WPts.at(i).y;
        E.pts[i].z=mMS.WPts.at(i).z;
    }
    E.solveNormal();
    for(int i=0;i<4;i++)
    {
        mPoint res=projPointToPlane(E.pts[i],E);
        mMS.WPts.at(i).x=res.x;
        mMS.WPts.at(i).y=res.y;
        mMS.WPts.at(i).z=res.z;
    }
    mMS.BuildWPt2();
}
void rectifyRectangle(std::vector<mPoint> & pts)
{
    mPoint v1,v2,normal,aux,mid,d1,d2,res;
    mPoint center,Ncenter,move;
    center=(pts.at(0)+pts.at(1)+pts.at(2)+pts.at(3))*0.25;
        mid=(pts.at(0)+pts.at(1))*0.5;
        d1=pts.at(0)-mid;
        d1.normalize();
        d2=pts.at(1)-mid;
        d2.normalize();
        v1=pts.at(1)-pts.at(0);
        v1.normalize();
        v2=pts.at(3)-pts.at(0);
        v2.normalize();
        normal=v1^v2;
        normal.normalize();
        aux=normal^v1;
        aux.normalize();
        res=mid+d1*45.0;
        pts.at(0)=res;
        res=res+aux*90.0;
        pts.at(3)=res;
        res=mid+d2*45.0;
        pts.at(1)=res;
        res=res+aux*90.0;
        pts.at(2)=res;
    Ncenter=(pts.at(0)+pts.at(1)+pts.at(2)+pts.at(3))*0.25;
    move=center-Ncenter;
    for(int i=0;i<4;i++)
        pts.at(i)=pts.at(i)+move;
}
void rectifyRectangle(std::vector<Point3f> & pts)
{
	std::vector<mPoint> npt;
	for(int i=0;i<pts.size();i++)
	{
		mPoint p(pts.at(i).x,pts.at(i).y,pts.at(i).z);
		npt.push_back(p);
	}
	rectifyRectangle(npt);
	for(int i=0;i<pts.size();i++)
	{
		pts.at(i).x=npt.at(i).x;
		pts.at(i).y=npt.at(i).y;
		pts.at(i).z=npt.at(i).z;
	}
}

void rectifyRectangle(MS & mMS)
{
	rectifyRectangle(mMS.WPts);
	mMS.BuildWPt2();

}


void rotateByAxisAngle(float angle, mPoint axis, mPoint & inOut)
{
    float R[3][3];
    float c=(float)cos((double)angle);
    float s=(float)sin((double)angle);
    float t=1-c;
    axis.normalize();
    float x=axis.x;
    float y=axis.y;
    float z=axis.z;
    R[0][0]=t*x*x+c;   R[0][1]=t*x*y-z*s; R[0][2]=t*x*z+y*s;
    R[1][0]=t*x*y+z*s; R[1][1]=t*y*y+c;   R[1][2]=t*y*z-x*s;
    R[2][0]=t*x*z-y*s; R[2][1]=t*y*z+x*s; R[2][2]=t*z*z+c;
    float newX=R[0][0]*inOut.x+R[0][1]*inOut.y+R[0][2]*inOut.z;
    float newY=R[1][0]*inOut.x+R[1][1]*inOut.y+R[1][2]*inOut.z;
    float newZ=R[2][0]*inOut.x+R[2][1]*inOut.y+R[2][2]*inOut.z;
    inOut.x=newX;
    inOut.y=newY;
    inOut.z=newZ;

}
void rectifyRectangle2(std::vector<mPoint> & pts)
{
    mPoint v1,v2,normal;
    v1=pts.at(2)-pts.at(0);
    v2=pts.at(3)-pts.at(1);
    v1.normalize();
    v2.normalize();
    float t;
    float y0=pts.at(0).y;
    float y1=pts.at(1).y;
    float x0=pts.at(0).x;
    float x1=pts.at(1).x;
    float v1x=v1.x;
    float v2x=v2.x;
    float v1y=v1.y;
    float v2y=v2.y;
    t=((y0-y1)*v2x-(x0-x1)*v2y)/(v1x*v2y-v1y*v2x);
    mPoint center;
    center=pts.at(0)+v1*t;
    normal=v1^v2;
    float cosValue=v1*v2;
    double angleV1V2d=acos((double)cosValue);
    float angleV1V2=(float)angleV1V2d;
    float rotAngle2;
    bool largerThanPI;
    float PI2=1.57079632679489661923;
    if(angleV1V2>PI2)
    {
        rotAngle2=(angleV1V2-PI2)/2.0;
        largerThanPI=true;
    }
    else
    {
        rotAngle2=(PI2-angleV1V2)/2.0;
        largerThanPI=false;
    }
    mPoint new0,new1,new2,new3;
    new0=pts.at(0)-center;
    new1=pts.at(1)-center;
    new2=pts.at(2)-center;
    new3=pts.at(3)-center;
    if(largerThanPI)
    {
        mPoint Mnormal=normal*(-1.0);
        rotateByAxisAngle(rotAngle2,normal,new0);
        rotateByAxisAngle(rotAngle2,Mnormal,new1);
        rotateByAxisAngle(rotAngle2,normal,new2);
        rotateByAxisAngle(rotAngle2,Mnormal,new3);
        float sqrt2=sqrt(2.0);
        float tempLengthRatio=(45.0*sqrt2)/new0.length();
        pts.at(0)=center+new0*tempLengthRatio;

        tempLengthRatio=(45.0*sqrt2)/new1.length();
        pts.at(1)=center+new1*tempLengthRatio;

        tempLengthRatio=(45.0*sqrt2)/new2.length();
        pts.at(2)=center+new2*tempLengthRatio;

        tempLengthRatio=(45.0*sqrt2)/new3.length();
        pts.at(3)=center+new3*tempLengthRatio;

    }
    else
    {
        mPoint Mnormal=normal*(-1.0);
        rotateByAxisAngle(rotAngle2,Mnormal,new0);
        rotateByAxisAngle(rotAngle2,normal,new1);
        rotateByAxisAngle(rotAngle2,Mnormal,new2);
        rotateByAxisAngle(rotAngle2,normal,new3);

        float sqrt2=sqrt(2.0);
        float tempLengthRatio=(45.0*sqrt2)/new0.length();
        pts.at(0)=center+new0*tempLengthRatio;

        tempLengthRatio=(45.0*sqrt2)/new1.length();
        pts.at(1)=center+new1*tempLengthRatio;

        tempLengthRatio=(45.0*sqrt2)/new2.length();
        pts.at(2)=center+new2*tempLengthRatio;

        tempLengthRatio=(45.0*sqrt2)/new3.length();
        pts.at(3)=center+new3*tempLengthRatio;
    }
}
void rectifyRectangle2(std::vector<Point3f> & pts)
{
	std::vector<mPoint> npt;
	for(int i=0;i<pts.size();i++)
	{
		mPoint p(pts.at(i).x,pts.at(i).y,pts.at(i).z);
		npt.push_back(p);
	}
	rectifyRectangle2(npt);
	for(int i=0;i<pts.size();i++)
	{
		pts.at(i).x=npt.at(i).x;
		pts.at(i).y=npt.at(i).y;
		pts.at(i).z=npt.at(i).z;
	}
}

void rectifyRectangle2(MS & mMS)
{
	rectifyRectangle2(mMS.WPts);
	mMS.BuildWPt2();
}
