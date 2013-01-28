/*
*  MarkerFinder.c
*
*
*  Created by lhaplus8888 2012/01/29.
*
*/

#include "MarkerFinder.h"



//#define VERBOSE	//冗長出力


//マーカーの向きようの定数



static double tickCount=0.0;

static int mScreenWidth;
static int mScreenHeight;
static CvMat* RealMarkerCorners;
static CvMat* ImgMarkerCorners;
static CvMat* EstimateRotation_vector;
static CvMat* EstimateTranslation_vector;

int Decode2D(IplImage* projectedImg,const int direction);

void StartWatch(char* name)
{
	tickCount=(double)cvGetTickCount();
	//fprintf(stdout,"%s",name);
}

void StopWatch()
{
	double freq=0.0f;

	tickCount=(double)cvGetTickCount()-tickCount;
	freq=cvGetTickFrequency()*1000;
	//fprintf(stdout,"%gms\n",(double)(tickCount/freq));

}



//画像のチャネル数、ビット深度などのチェック
int CheckColorImage(IplImage *srcImage)
{

	if(NULL==srcImage)
	{
		//fprintf(stderr,"SrcImage is null@CheckColorImage.\n");
		__android_log_print(ANDROID_LOG_ERROR,"Check color img","Null source");
		return -1;
	}

	//画像のフォーマットがIPL_DEPTH_8U,3チャネルじゃないと拒否する
	if(srcImage->nChannels!=3)
	{
		//fprintf(stderr,"Channel count must 3.@CheckColorImage\n");
		//fprintf(stderr,"img channel is %d @CheckColorImage\n",(int)srcImage->nChannels);
		__android_log_print(ANDROID_LOG_ERROR,"Check color img","Not Channel 3, Num Channels %d",srcImage->nChannels);
		return -2;
	}

	if(srcImage->depth!=IPL_DEPTH_8U)
	{
		//fprintf(stderr,"Image depth must IPL_DEPTH_8U. @CheckColorImage\n");
		//fprintf(stderr,"img depth is %d @CheckColorImage\n",(int) srcImage->depth);
		__android_log_print(ANDROID_LOG_ERROR,"Check color img","Depth wrong");
		return -3;
	}
	return GOOD_COLORIMAGE;
}


//MarkerRectangleの中身を出力。
void printRect(const MarkerRectangle rect)
{
	int i;

	//fprintf(stdout,"-------PrintRect-------\n");
	//fprintf(stdout,"ID:%d\n",rect.MarkerID);
	//fprintf(stdout,"Direction:");
	switch (rect.Direction)
	{
	case DIRECTION_TL:__android_log_write(ANDROID_LOG_INFO,"DecodeMarker2DCode","Top Left");break;//fprintf(stdout,"Top Left\n");break;
	case DIRECTION_BL:__android_log_write(ANDROID_LOG_INFO,"DecodeMarker2DCode","Bottom Left");break;//fprintf(stdout,"Bottom Left\n");break;
	case DIRECTION_BR:__android_log_write(ANDROID_LOG_INFO,"DecodeMarker2DCode","Bottom Right");break;//fprintf(stdout,"Bottom Right\n");break;
	case DIRECTION_TR:__android_log_write(ANDROID_LOG_INFO,"DecodeMarker2DCode","Top Right");break;//fprintf(stdout,"Top Right\n");break;
	default:__android_log_write(ANDROID_LOG_INFO,"DecodeMarker2DCode","Unknown Direction");break;//fprintf(stdout,"Unknown Direction\n");
	}
	//fprintf(stdout,"\n");
	//fprintf(stdout,"Outer Corners\n");
	for(i=0;i<4;i++)
	{
		__android_log_print(ANDROID_LOG_INFO,"Outer Corner Pos","x: %f, y: %f",rect.outer_corners[i].x,rect.outer_corners[i].y);
		__android_log_print(ANDROID_LOG_INFO,"Outer Corner Degree","%f",rect.outer_degrees[i]);
		//fprintf(stdout,"\tPos:(%g,%g)\t",rect.outer_corners[i].x,rect.outer_corners[i].y);
		//fprintf(stdout,"\tdeg:%g\n",rect.outer_degrees[i]);
	}

	//fprintf(stdout,"Inner Corners\n");

	for(i=0;i<4;i++)
	{
		__android_log_print(ANDROID_LOG_INFO,"Inner Corner Pos","x: %f, y: %f",rect.inner_corners[i].x,rect.inner_corners[i].y);
		__android_log_print(ANDROID_LOG_INFO,"Inner Corner Degree","%f",rect.inner_degrees[i]);
		//fprintf(stdout,"\tPos:(%g,%g)\t",rect.inner_corners[i].x,rect.inner_corners[i].y);
		//fprintf(stdout,"\tdeg:%g\n",rect.inner_degrees[i]);
	}

	//fprintf(stdout,"Rotation Vector\n");
	//fprintf(stdout,"\t(x,y,z):(%g,%g,%g)\n",rect.rotation_vector.x,rect.rotation_vector.y,rect.rotation_vector.z);

	//fprintf(stdout,"Translation Vector\n");
	//fprintf(stdout,"\t(x,y,z):(%g,%g,%g)\n",rect.translation_vector.x,rect.translation_vector.y,rect.translation_vector.z);

	//fprintf(stdout,"---------------------\n");
}

//--------------------------------------------------------------------------
//四角形を探します。
//	colorImg: 元画像
//	rectangles:結果格納先
//  rectlimitCount:最大検出数
//	戻り値： 見つけた四角形の数。
//--------------------------------------------------------------------------
int FindRectangle(IplImage* colorImg,MarkerRectangle* rectangles,const int rectLimitCount)
{

	//__android_log_print(ANDROID_LOG_DEBUG,"Find Rectangle","Start");
#ifdef VERBOSE
	//fprintf(stderr,"START  FindRectangle\n");
#endif
	int i;
	int r;

	//輪郭保存用ストレージ
	//create storage for contours
	CvMemStorage *storage;
	CvMemStorage *storagepoly;
	CvSeq *firstcontour=NULL;
	CvSeq *polycontour=NULL;
	IplImage* gsImage;
	IplImage* gsImageContour;
	int contourCount=0;

	CvSeq* c;
	char text[4];

	int detectedRectCount=0;
	int res=0;
	CvSeq* c_vnext;

	double process_time = (double)cvGetTickCount();

	//初期化する
	for(i=0;i<rectLimitCount;i++)
	{
		rectangles[i].MarkerID=-1;//マーカーの番号　　identifier
		for (r=0;r<4;r++)
		{
			rectangles[i].outer_corners[r]=cvPoint2D32f(-1,-1);
			rectangles[i].outer_degrees[r]=-1;

			rectangles[i].inner_corners[r]=cvPoint2D32f(-1,-1);
			rectangles[i].inner_degrees[r]=-1;
			rectangles[i].rotation_vector=cvPoint3D32f(0,0,0);	//移動ベクトル
			rectangles[i].translation_vector=cvPoint3D32f(0,0,0);//回転ベクトル
		}
	}

	storage = cvCreateMemStorage (0);//検出したそのままの輪郭用　　original contours
	storagepoly = cvCreateMemStorage (0);//ポリゴン近似した輪郭用	 polygon approximated contour
	cvClearMemStorage(storage);
	cvClearMemStorage(storagepoly);

	gsImage=cvCreateImage(cvGetSize(colorImg),IPL_DEPTH_8U,1);
	gsImageContour=cvCreateImage(cvGetSize(colorImg),IPL_DEPTH_8U,1);
	//__android_log_print(ANDROID_LOG_DEBUG,"Find Rectangle","OK 1");
	//グレースケールに変換　　convert to grayscale
	cvCvtColor(colorImg,gsImage,CV_BGR2GRAY);

	//平滑化すると、小さなマーカーが検出されないのでコメントアウト
	//cvSmooth(gsImage,gsImage,CV_GAUSSIAN,3, 0, 0,0);


	//２値化	Convert gray to binary
	cvThreshold (gsImage, gsImage, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
	cvNot(gsImage,gsImage);

	//__android_log_print(ANDROID_LOG_DEBUG,"Find Rectangle","OK 2");
	//輪郭検出		 find contours
	cvCopy(gsImage,	gsImageContour,	NULL);
	contourCount=cvFindContours (gsImageContour, storage, &firstcontour, sizeof (CvContour),
		CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE,cvPoint(0,0) );

	//輪郭を直線近似する　Approximate to polygon
	polycontour=cvApproxPoly(firstcontour,sizeof(CvContour),storagepoly,CV_POLY_APPROX_DP,4,1);
	/*
	//test
	cvDrawContours(colorImg,polycontour,CV_RGB(255,255,200),CV_RGB(0,0,0),2,1,8,cvPoint(0,0));
	*/

#ifdef VERBOSE
	//fprintf(stderr,"%d contours.\n",contourCount);
#endif
	//__android_log_print(ANDROID_LOG_DEBUG,"Find Rectangle","OK 3");
	for(c=polycontour;c!=NULL;c=c->h_next)
	{
		int res;

		if(detectedRectCount==rectLimitCount) continue;
		//長過ぎたり、短すぎる輪郭は無視
		//ignore too long or too short contours.
		if(cvContourPerimeter(c)>(colorImg->width+colorImg->height)*1.8)
		{

			continue;
		}

		if(cvContourPerimeter(c)<50)
		{
			continue;
		}

		//四角形の輪郭では無いものを除外
		//ignore conercount!=4
		if(c->total!=4)
		{
			/*
			//テスト用
			for(i=0;i<c->total;i++)
			{
			CvPoint *p=CV_GET_SEQ_ELEM(CvPoint,c,i);
			cvCircle(colorImg,*p,5,CV_RGB(0,255,100),1,8,0);
			}*/

			continue;
		}

		//次のレベルに輪郭が無ければ（内側の輪郭が無ければ）無視
		//さらに、内側の輪郭も４角形じゃなければ無視
		// ignore v_next contour is null or v_next is not rectangle

		if(NULL==c->v_next)
		{


			continue;
		}
		if(4!=c->v_next->total)
		{
			/*
			//テスト用
			for(i=0;i<c->v_next->total;i++)
			{
			CvPoint *p=CV_GET_SEQ_ELEM(CvPoint,c->v_next,i);
			cvCircle(colorImg,*p,3,CV_RGB(0,100,200),-1,8,0);
			}
			*/

			continue;
		}
		res=0;
		c_vnext=c->v_next;

		/*
		//テスト用
		for(i=0;i<c->total;i++)
		{
		CvPoint *p=CV_GET_SEQ_ELEM(CvPoint,c,i);
		cvCircle(colorImg,*p,10,CV_RGB(255,255,100),1,8,0);
		}

		for(i=0;i<c->v_next->total;i++)
		{
		CvPoint *p=CV_GET_SEQ_ELEM(CvPoint,c->v_next,i);
		cvCircle(colorImg,*p,5,CV_RGB(255,255,100),1,8,0);
		}
		*/

		//角度／頂点情報計算
		res=CheckRectangle(c,c_vnext,&rectangles[detectedRectCount]);

		if(GOOD_RECTANGLE==res)
		{
			//Draw marker's contour
#ifdef VERBOSE
			//頂点情報全部出力
			printRect(rectangles[detectedRectCount]);
#endif
			detectedRectCount++;
			//cvDrawContours(colorImg,c,CV_RGB(255,0,0),CV_RGB(0,0,0),0,3,8,cvPoint(0,0));
			//cvDrawContours(colorImg,c_vnext,CV_RGB(0,255,0),CV_RGB(0,255,255),0,2,8,cvPoint(0,0));
		}
	}
	//cvShowImage("marker_inside",marker_inside);
	//cvShowImage("marker_inside",marker_inside);
	process_time = (double)cvGetTickCount()-process_time;
#ifdef VERBOSE
	//fprintf(stdout,"process_time %gms@FindRectangle\n", process_time/(cvGetTickFrequency()*1000.));
	//fprintf(stdout,"%d rectangles found.\n",detectedRectCount);

	cvShowImage("capture_image",colorImg);
	cvShowImage("GS",gsImage);
	cvMoveWindow("GS",640,0);
	char presskey=cvWaitKey (1000);
#endif
	//__android_log_print(ANDROID_LOG_DEBUG,"Find Rectangle","OK 5");

	cvReleaseMemStorage(&storage);
	cvReleaseMemStorage(&storagepoly);
	cvReleaseImage(&gsImage);
	cvReleaseImage(&gsImageContour);


#ifdef VERBOSE
	//fprintf(stdout,"end FindRectangle\n\n");
#endif


	return detectedRectCount;

}

//--------------------------------------------------------------------------
//３点でなす角度を計算します。
//--------------------------------------------------------------------------
double dotDegree(CvPoint common,CvPoint a,CvPoint b)
{
	CvPoint2D32f vecA;
	CvPoint2D32f vecB;
	double normA;
	double normB;
	double costheta=0;
	double rad=0.0f;
	double PI=3.141592;
	double degree=rad*180/PI;

	//common->aとCommon->bの内積を計算して、角度を求めます。
	vecA=cvPoint2D32f(a.x-common.x,a.y-common.y);
	vecB=cvPoint2D32f(b.x-common.x,b.y-common.y);

	normA=sqrt(vecA.x*vecA.x + vecA.y*vecA.y);
	normB=sqrt(vecB.x*vecB.x + vecB.y*vecB.y);


	vecA.x=vecA.x/normA;
	vecA.y=vecA.y/normA;

	vecB.x=vecB.x/normB;
	vecB.y=vecB.y/normB;

	costheta=(vecA.x*vecB.x+vecA.y*vecB.y);

	rad=acos(costheta);
	PI=3.141592;
	degree=rad*180/PI;

	return degree;

#ifdef VERBOSE
	//デバッグプリント
	//fprintf(stdout,"common:%d, %d\t",common.x,common.y);
	//fprintf(stdout,"A:%d, %d\t",a.x,a.y);
	//fprintf(stdout,"B:%d, %d\n",b.x,b.y);


	//fprintf(stdout,"costheta:%g   rad:%g   degree:%g.\n",costheta,rad,degree);
	//fprintf(stdout,"vecA.x:%g vecA.y:%g \nvecB.x:%g VecB.y:%g\n",vecA.x,vecA.y,vecB.x,vecB.y);
#endif


}


//--------------------------------------------------------------------------
//四角形かどうかをチェックします。
//チェック方法：内積をとって,60度0120度の間にいるかをチェック
//return 0:四角形と認定(good rectangle)　　-1:四角形ではない(not rectangle)
//--------------------------------------------------------------------------
int CheckRectangle(CvSeq* contour,CvSeq* inner_contour, MarkerRectangle* rectangle)
{
	CvPoint corners[4];
	double degrees[4];

	CvPoint inner_corners[4];
	double inner_degrees[4];

	//制限値
	double capa_deg=25;
	double upper_threash_degree=180-capa_deg;
	double lower_threash_degree=capa_deg;

	int i=0;
	int passcount=0;
	int prev=0;
	int next=0;

	if(contour->total!=4) return -1;


#ifdef VERBOSE
	//fprintf(stdout,"\nCheckRectangle\n");
#endif

    int start=0;

    int ShortestDist=std::numeric_limits<int>::max();
    int ox=std::numeric_limits<int>::max();
    int oy=std::numeric_limits<int>::max();
    for(int i=0;i<4;i++)
    {
        CvPoint *p=CV_GET_SEQ_ELEM(CvPoint,contour,i);
        if(ox>p->x)
        	ox=p->x;
		if(oy>p->y)
			oy=p->y;
    }
    for(int i=0;i<4;i++)
    {
        CvPoint *p=CV_GET_SEQ_ELEM(CvPoint,contour,i);
        int x=p->x;
        int y=p->y;
        if((x>=(mScreenWidth-3))||x<3||(y>=(mScreenHeight-3)||y<3))
        	return -1;
        x=x-ox;
        y=y-oy;
        int dist=x*x+y*y;
        if(dist<ShortestDist)
        {
            start=i;
            ShortestDist=dist;
        }
    }

    for(i=0;i<4;i++)
    {
        CvPoint *p=CV_GET_SEQ_ELEM(CvPoint,contour,(i+start)%4);
        corners[i].x=p->x;
        corners[i].y=p->y;

        p=CV_GET_SEQ_ELEM(CvPoint,inner_contour,(i+start)%4);
        inner_corners[i].x=p->x;
        inner_corners[i].y=p->y;
    }


	for(i=0;i<4;i++)
	{
		next=(i+1)%4;	// 0-1 0-3, 1-2 1-0, 2-3 2-1, 3-0 3-2 の組み合わせ
		prev=(i-1);
		if(-1==prev) prev=3;
		degrees[i]=dotDegree(corners[i],corners[next],corners[prev]);
		inner_degrees[i]=dotDegree(inner_corners[i],inner_corners[next],inner_corners[prev]);

	}


#ifdef VERBOSE
	//チェック
	//fprintf(stdout,"Outer degrees\t%g\t%g\t%g\t%g\n",degrees[0],degrees[1],degrees[2],degrees[3]);
	//fprintf(stdout,"Inner degrees\t%g\t%g\t%g\t%g\n",inner_degrees[0],inner_degrees[1],inner_degrees[2],inner_degrees[3]);
#endif

	//角度チェックして、全部条件に一致していたらマーカー用の四角形のペアと認定。
	//頂点の順番は同じではないので、あとでチェックする必要あり。
	//（現時点ではouter_corner[0]と、inner_corner[0]はほぼ対応していない


	for(i=0;i<4;i++)
	{
		if((degrees[i]<=upper_threash_degree)&& (degrees[i]>=lower_threash_degree))
			passcount++;

		if((degrees[i]<=upper_threash_degree)&& (inner_degrees[i]>=lower_threash_degree))
			passcount++;
	}

	if(8==passcount)
	{
		//頂点情報と角度情報を埋め込む
		for(i=0;i<4;i++)
		{
			//外側
			rectangle->outer_corners[i]=cvPoint2D32f((float)corners[i].x,(float)corners[i].y);
			rectangle->outer_degrees[i]=degrees[i];

			//内側
			rectangle->inner_corners[i]=cvPoint2D32f((float)inner_corners[i].x,(float)inner_corners[i].y);
			rectangle->inner_degrees[i]=inner_degrees[i];
		}


		return GOOD_RECTANGLE;
	}
	else
	{
		return -1;
	}


}



//--------------------------------------------------------------------------
//マーカーをデコードして、番号を割りふります。
//この関数を使う前に、FindRectangleを使ってマーカーの頂点情報と角度情報を計算しておいてください。
//この関数内で、頂点情報を整列させます。
//	整列後のレイアウト：　　　O[i]: i番目の外枠頂点情報　I[i]:i番目の内枠頂点情報
//	O[0]						O[3]
//		I[0]				I[3]
//			X
//				x	x	x
//				x	x	x
//				x	x	x
//		I[1]				I[2]
//	O[1]						O[2]
//
//
//
//	colorImg: 元画像
//	rectangles:頂点／角度が既に求まっているMarkerRectangleの配列。
//  rectCount:検査個数
//	戻り値： なし
//  マーカー５個で15ms程度　3つで9ms程度(20120129 mac air)
//--------------------------------------------------------------------------
void DecodeMarker2DCode(IplImage* colorImg,MarkerRectangle* rectangles,const int rectCount)
{
	//画像チェック
	int checkRes;
	IplImage* grayImg;
	IplImage* workGrayImg;
	int ProjImgSize;

	IplImage* projectedImg;
	int rectIndex;
	CvPoint2D32f dstCorners[4];	//画像上の変形している四角形の投影先
	int i=0;
	CvMat* map_matrix;
	int Direction;
	int ID;

	checkRes=CheckColorImage(colorImg);
	if(GOOD_COLORIMAGE!=checkRes)
	{
		//fprintf(stderr,"Invalid image@DecodeMarker2DCode\n");
		__android_log_print(ANDROID_LOG_ERROR,"Decode Marker","no good color image");
		return;
	}

	grayImg=cvCreateImage(cvGetSize(colorImg),IPL_DEPTH_8U,1);
	workGrayImg=cvCreateImage(cvGetSize(colorImg),IPL_DEPTH_8U,1);

	//枠の内側を投影したときの画像
	//ProjImgSize=180;
	ProjImgSize=270;
	projectedImg=cvCreateImage(cvSize(ProjImgSize,ProjImgSize),IPL_DEPTH_8U,1);

	cvCvtColor(colorImg,grayImg,CV_BGR2GRAY);
	cvThreshold (grayImg, grayImg, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

	rectIndex=0;
	i=0;
	dstCorners[0]=cvPoint2D32f(0,0);
	dstCorners[1]=cvPoint2D32f(0,ProjImgSize);
	dstCorners[2]=cvPoint2D32f(ProjImgSize,ProjImgSize);
	dstCorners[3]=cvPoint2D32f(ProjImgSize,0);
	map_matrix=cvCreateMat (3, 3, CV_32FC1);

	for(rectIndex=0;rectIndex<rectCount;rectIndex++)
	{
		//テスト用
		//printRect(rectangles[rectIndex]);

		cvCopy(grayImg,workGrayImg,NULL);
		cvErode(workGrayImg,workGrayImg,NULL,1);

		//外枠の方が大きく画像に映るので、ノイズに強いはず
		cvGetPerspectiveTransform(rectangles[rectIndex].outer_corners, dstCorners,map_matrix);
		cvWarpPerspective(workGrayImg,projectedImg,map_matrix,CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll (0));

		Direction=CheckDirection(projectedImg);

		switch (Direction)
		{
		case DIRECTION_TL:__android_log_write(ANDROID_LOG_INFO,"DecodeMarker2DCode","Top Left");break;//fprintf(stdout,"Top Left\n");break;
		case DIRECTION_BL:__android_log_write(ANDROID_LOG_INFO,"DecodeMarker2DCode","Bottom Left");break;//fprintf(stdout,"Bottom Left\n");break;
		case DIRECTION_BR:__android_log_write(ANDROID_LOG_INFO,"DecodeMarker2DCode","Bottom Right");break;//fprintf(stdout,"Bottom Right\n");break;
		case DIRECTION_TR:__android_log_write(ANDROID_LOG_INFO,"DecodeMarker2DCode","Top Right");break;//fprintf(stdout,"Top Right\n");break;
		default:__android_log_write(ANDROID_LOG_INFO,"DecodeMarker2DCode","Unknown Direction");break;//fprintf(stdout,"Unknown Direction\n");
		}
		//AlignMarkerCorners(MarkerRectangle* rectangles,const int rectCount);
		correctMarkerPoints(rectangles[rectIndex],Direction);
		ID=Decode2D(projectedImg,Direction);

		rectangles[rectIndex].MarkerID=ID;
		rectangles[rectIndex].Direction=Direction;

	}

	//片付け
	cvReleaseImage(&grayImg);
	cvReleaseImage(&workGrayImg);
	cvReleaseImage(&projectedImg);
	cvReleaseMat(&map_matrix);

#ifdef VERBOSE
	//fprintf(stderr,"END DecodeMarker2DCode\n");
#endif
	return;

}

//マーカーの方向を探します
//マーカーの外側の大きさは90mm,内側は50mmです。さらに内側から5mmは余白にしてあります。
//なので(25,25)0(35,35)の画素の平均値を調べてしきい値以下だったら、左上にキーマーカーがあることになります。
//同じように左下、右下、右上も調べます。
//１回あたり0.015ms程度。
int CheckDirection(IplImage* projectedImg)
{


	double val_TL;	//左上	Top left
	double val_BL;	//左下	Bottom left
	double val_BR;	//右下	Bottom right
	double val_TR;	//右上	Top right
	int tmpKey;
	int KeyCount;
	double keyThreshold;
	int ROISize;
	//ROI: the top left small square of the marker, use to find direction
	CvRect ROI;
	int StartX;
	int StartY;
	CvScalar avg;	//ROIの画素平均値 計算用　  for average of ROI.


	assert(projectedImg!=NULL);
	ROISize=ceil(projectedImg->width*(float)(MARKER_DOTSIZE/MARKER_OUTER_SIZE));
	val_TL=0.0f;
	val_BL=0.0f;
	val_BR=0.0f;
	val_TR=0.0f;

	ROI.width=ROISize;
	ROI.height=ROISize;

	//左上
	StartX=ceil(projectedImg->width*(float)((INNER_START)/MARKER_OUTER_SIZE));
	StartY=StartX;
	ROI.x=StartX;
	ROI.y=StartY;
	cvSetImageROI(projectedImg,ROI);
	avg=cvAvg(projectedImg,NULL);
	cvResetImageROI(projectedImg);
	val_TL=avg.val[0];

	//左下
	ROI.y=ceil(projectedImg->height*(float)(INNER_END/MARKER_OUTER_SIZE))-ROISize;
	cvSetImageROI(projectedImg,ROI);
	avg=cvAvg(projectedImg,NULL);
	cvResetImageROI(projectedImg);
	val_BL=avg.val[0];

	//右下
	ROI.x=ceil(projectedImg->width*(float)(INNER_END/MARKER_OUTER_SIZE)-ROISize);
	cvSetImageROI(projectedImg,ROI);
	avg=cvAvg(projectedImg,NULL);
	cvResetImageROI(projectedImg);
	val_BR=avg.val[0];


	//右上
	ROI.y=ceil(projectedImg->width*(float)(INNER_START/MARKER_OUTER_SIZE));
	cvSetImageROI(projectedImg,ROI);
	avg=cvAvg(projectedImg,NULL);
	cvResetImageROI(projectedImg);
	val_TR=avg.val[0];


#ifdef VERBOSE
	//fprintf(stdout,"AVERAGES\n");
	//fprintf(stdout,"\t%g\t%g\n",val_TL,val_TR);
	//fprintf(stdout,"\t%g\t%g\n",val_BL,val_BR);
#endif

	KeyCount=0;
	keyThreshold= 60;//キー候補にするしきい値。
	tmpKey=DIRECTION_UNKNOWN;
	if(val_TL<keyThreshold)
	{
		KeyCount++;
		keyThreshold=val_TL;
		tmpKey=DIRECTION_TL;
		//__android_log_print(ANDROID_LOG_INFO,"TL: ","%f",val_TL);
	}

	if(val_BL<keyThreshold)
	{
		KeyCount++;
		keyThreshold=val_BL;
		tmpKey=DIRECTION_BL;
		//__android_log_print(ANDROID_LOG_INFO,"BL: ","%f,  %f",val_BL,val_TL);
	}

	if(val_BR<keyThreshold)
	{
		KeyCount++;
		keyThreshold=val_BR;
		tmpKey=DIRECTION_BR;
		//__android_log_print(ANDROID_LOG_INFO,"BR: ","%f",val_BR);
	}

	if(val_TR<keyThreshold)
	{
		KeyCount++;
		tmpKey=DIRECTION_TR;
		//__android_log_print(ANDROID_LOG_INFO,"TR: ","%f",val_TR);
	}
	return tmpKey;
}
/*
int CheckDirection(IplImage* colorImage, IplImage* projectedImg)
{
	double val_TL;	//左上	Top left
	double val_BL;	//左下	Bottom left
	double val_BR;	//右下	Bottom right
	double val_TR;	//右上	Top right
	int tmpKey;
	int KeyCount;
	double keyThreshold;
	int ROISize;
	//ROI: the top left small square of the marker, use to find direction
	CvRect ROI;
	int StartX;
	int StartY;
	CvScalar avg;	//ROIの画素平均値 計算用　  for average of ROI.


	assert(projectedImg!=NULL);
	ROISize=ceil(projectedImg->width*(float)(MARKER_DOTSIZE/MARKER_OUTER_SIZE));
	val_TL=0.0f;
	val_BL=0.0f;
	val_BR=0.0f;
	val_TR=0.0f;

	ROI.width=ROISize;
	ROI.height=ROISize;

	//左上
	StartX=ceil(projectedImg->width*(float)((INNER_START)/MARKER_OUTER_SIZE));
	StartY=StartX;
	ROI.x=StartX;
	ROI.y=StartY;
	cvSetImageROI(projectedImg,ROI);
	avg=cvAvg(projectedImg,NULL);
	cvResetImageROI(projectedImg);
	val_TL=avg.val[0];

	//左下
	ROI.y=ceil(projectedImg->height*(float)(INNER_END/MARKER_OUTER_SIZE))-ROISize;
	cvSetImageROI(projectedImg,ROI);
	avg=cvAvg(projectedImg,NULL);
	cvResetImageROI(projectedImg);
	val_BL=avg.val[0];

	//右下
	ROI.x=ceil(projectedImg->width*(float)(INNER_END/MARKER_OUTER_SIZE)-ROISize);
	cvSetImageROI(projectedImg,ROI);
	avg=cvAvg(projectedImg,NULL);
	cvResetImageROI(projectedImg);
	val_BR=avg.val[0];


	//右上
	ROI.y=ceil(projectedImg->width*(float)(INNER_START/MARKER_OUTER_SIZE));
	cvSetImageROI(projectedImg,ROI);
	avg=cvAvg(projectedImg,NULL);
	cvResetImageROI(projectedImg);
	val_TR=avg.val[0];


#ifdef VERBOSE
	//fprintf(stdout,"AVERAGES\n");
	//fprintf(stdout,"\t%g\t%g\n",val_TL,val_TR);
	//fprintf(stdout,"\t%g\t%g\n",val_BL,val_BR);
#endif

	KeyCount=0;
	keyThreshold= 60;//キー候補にするしきい値。
	tmpKey=DIRECTION_UNKNOWN;
	if(val_TL<keyThreshold)
	{
		KeyCount++;
		//keyThreshold=val_TL;
		tmpKey=DIRECTION_TL;
	}

	if(val_BL<keyThreshold)
	{
		KeyCount++;
		//keyThreshold=val_BL;
		tmpKey=DIRECTION_BL;
	}

	if(val_BR<keyThreshold)
	{
		KeyCount++;
		//keyThreshold=val_BR;
		tmpKey=DIRECTION_BR;
	}

	if(val_TR<keyThreshold)
	{
		KeyCount++;
		tmpKey=DIRECTION_TR;
	}
	return tmpKey;
}
*/
//マーカーの中身をデコードします。
//回転しているマーカーと、向きを受け取り、補正してからデコードします
//  X
//		x	x	x
//		x	x	x
//		x	x	使っちゃダメ
//
//1回あたり1.4ms.
int Decode2D(IplImage* projectedImg,const int direction)
{



	CvPoint2D32f src[4];
	CvPoint2D32f dst[4];
	CvMat* map_matrix;
	IplImage* rotatedImg;
	CvPoint bit[8];

	int startidx;
	int idx;
	int i;
	int ROIBase;
	double ratio;
	CvRect ROI;
	CvScalar val;
	int ID;

	//__android_log_print(ANDROID_LOG_INFO,"Decode 2D","start");


	assert(NULL!=projectedImg);
	if(DIRECTION_UNKNOWN==direction)
	{
		__android_log_print(ANDROID_LOG_ERROR,"Decode 2D","Direction Unknown");
		return -1;
	}
	rotatedImg=cvCreateImage(cvGetSize(projectedImg),IPL_DEPTH_8U,1);
	map_matrix=cvCreateMat (3, 3, CV_32FC1);
	//__android_log_print(ANDROID_LOG_INFO,"Decode 2D","OK 0, direction %d", direction);
	//元の場所
	src[0]=cvPoint2D32f(0,0);
	src[1]=cvPoint2D32f(0,projectedImg->height);
	src[2]=cvPoint2D32f(projectedImg->width,projectedImg->height);
	src[3]=cvPoint2D32f(projectedImg->width,0);



	//回転先
	switch (direction)
	{
	case DIRECTION_TL:startidx = 0;	break;
	//case DIRECTION_BL:startidx = 3;	break;
	case DIRECTION_BL:startidx = 1;	break;
	case DIRECTION_BR:startidx = 2;	break;
	//case DIRECTION_TR:startidx = 1;	break;
	case DIRECTION_TR:startidx = 3;	break;
	default:startidx = -1;
	}
	if(-1==startidx)
	{
		__android_log_print(ANDROID_LOG_ERROR,"Decode 2D","startidx==-1");
		return -1;
	}
	//__android_log_print(ANDROID_LOG_INFO,"Decode 2D","OK 1, direction %d", direction);

	//キーが左上: 0,1,2,3
	//キーが左下: 3,0,1,2
	//キーが右下: 2,3,0,1
	//キーが右上: 1,2,3,0


	for(i=0;i<4;i++)
	{
		//idx=((4-startidx)+i)%4;
		idx=(startidx+i)%4;
		dst[idx]=src[i];
	}

	//向きを回転させる
	cvGetPerspectiveTransform(src, dst,map_matrix);
	cvWarpPerspective(projectedImg,rotatedImg,map_matrix,
		CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll (0));

	//マーカーデコード開始
	//中心座標指定

	//__android_log_print(ANDROID_LOG_INFO,"Decode 2D","OK 2");
	bit[7]=cvPoint(40,40);
	bit[6]=cvPoint(50,40);
	bit[5]=cvPoint(60,40);
	bit[4]=cvPoint(40,50);
	bit[3]=cvPoint(50,50);
	bit[2]=cvPoint(60,50);
	bit[1]=cvPoint(40,60);
	bit[0]=cvPoint(50,60);

	//7 6 5
	//4 3 2
	//1 0
	//右下(60,60)は使っちゃダメ
	ratio=0.7;//マーカーの中央付近のみを参照する

	ROIBase=(int)ceil(0.5* ratio*MARKER_DOTSIZE/MARKER_OUTER_SIZE*(float)projectedImg->width);
	ROI.width=ROIBase*2;
	ROI.height=ROIBase*2;
	//__android_log_print(ANDROID_LOG_INFO,"Decode 2D","OK 3, ROI width %d, height %d", ROI.width, ROI.height);
	ID=0;
	int B=1;
	for(i=0;i<8;i++)
	{
		CvPoint ROICenter;
		double x=bit[i].x*(projectedImg->width/MARKER_OUTER_SIZE);
		double y=bit[i].y*(projectedImg->width/MARKER_OUTER_SIZE);
		ROICenter=cvPoint(ceil(x),ceil(y));
		ROI.x=ROICenter.x-ROIBase;
		ROI.y=ROICenter.y-ROIBase;
		cvSetImageROI(rotatedImg,ROI);
		val=cvAvg(rotatedImg,NULL);
		cvResetImageROI(rotatedImg);

		//平均値２０以下なら、塗りつぶされてると判断する
		float threshold=20.0;
		if(val.val[0]<threshold)
		{
			ID=ID+B;
		}
		B=B*2;
		//else
		//	__android_log_print(ANDROID_LOG_ERROR,"Marker Decode","bit %d, avg %f", i, val.val[0]);

	}
	//__android_log_print(ANDROID_LOG_INFO,"Decode 2D","OK 4");
	cvReleaseMat(&map_matrix);
	cvReleaseImage(&rotatedImg);

	//fprintf(stdout,"ID=%d\n",ID);
	//__android_log_print(ANDROID_LOG_ERROR,"Marker Decode","Computed ID %d", i, val.val[0]);
	return ID;
}


//マーカーの頂点を並び替えます。
//マーカーの方向は、outer_cornerの並びから計算しているので、
//まずはouter_cornerを並び替えてからinner_cornerの並び替えを行います。
void AlignMarkerCorners(MarkerRectangle* rectangles,const int rectCount)
{

#ifdef VERBOSE
	//fprintf(stdout,"START AlignMarkerCorners\n");
#endif


	CvPoint2D32f aligned_outer[4];
	CvPoint2D32f aligned_inner[4];

	double aligned_outer_degrees[4];
	double aligned_inner_degrees[4];

	int rectIndex=0;
	int startidx=-1;
	int Direction=0;
	int i=0;
	int idx;
	int outerIdx;
	int innerIdx=0;
	CvPoint2D32f outerPoint;
	int minDistIdx=0;
	CvPoint2D32f newInnerPoint;


	for(rectIndex=0;rectIndex<rectCount;rectIndex++)
	{
		int outerIdx;
		int innerIdx=0;


		//外側
		startidx=-1;
		Direction=rectangles[rectIndex].Direction;
		switch (Direction)
		{
		case DIRECTION_TL:startidx=0;break;
		case DIRECTION_BL:startidx=1;break;
		case DIRECTION_BR:startidx=2;break;
		case DIRECTION_TR:startidx=3;break;
		}
		if(-1==startidx) return;

		i=0;
		//キーが左上: 0,1,2,3
		//キーが左下: 1,2,3,0
		//キーが右下: 2,3,0,1
		//キーが右上: 3,0,1,2
#ifdef VERBOSE
		//fprintf(stdout,"startidx : %d\n",startidx);
		printRect(*rectangle);
#endif

		for(i=0;i<4;i++)
		{
			int idx=((i+startidx))%4;
			aligned_outer[i]=rectangles[rectIndex].outer_corners[idx];
			aligned_outer_degrees[i]=rectangles[rectIndex].outer_degrees[idx];

		}

		//外側　格納
		for(i=0;i<4;i++)
		{
			rectangles[rectIndex].outer_corners[i]=aligned_outer[i];
			rectangles[rectIndex].outer_degrees[i]=aligned_outer_degrees[i];

		}

		//次、内側
		//外側のポイントに一番近いところにする
		for(outerIdx=0;outerIdx<4;outerIdx++)
		{
			CvPoint2D32f outerPoint;
			double dist[4];
			double diffX;
			double diffY;
			int minDistIdx;

			outerPoint=rectangles[rectIndex].outer_corners[outerIdx];

			for(innerIdx=0;innerIdx<4;innerIdx++)
			{
				diffX=outerPoint.x-rectangles[rectIndex].inner_corners[innerIdx].x;
				diffY=outerPoint.y-rectangles[rectIndex].inner_corners[innerIdx].y;
				dist[innerIdx]=diffX*diffX+diffY*diffY;
#ifdef VERBOSE
				//fprintf(stdout,"ID:%d, dist:%g\n",innerIdx,dist[innerIdx]);
#endif
			}

			//一番距離が短いのを探す
			minDistIdx=0;
			for(innerIdx=0;innerIdx<4;innerIdx++)
			{
				if(dist[minDistIdx]>dist[innerIdx])
				{
					minDistIdx=innerIdx;
				}

			}

			newInnerPoint=rectangles[rectIndex].inner_corners[minDistIdx];
#ifdef VERBOSE
			//fprintf(stdout,"Min ID:%d, dist:%g\n",minDistIdx,dist[minDistIdx]);
#endif

			aligned_inner[outerIdx]=rectangles[rectIndex].inner_corners[minDistIdx];
			aligned_inner_degrees[outerIdx]=rectangles[rectIndex].inner_degrees[minDistIdx];

		}

		//内側　格納
		for(innerIdx=0;innerIdx<4;innerIdx++)
		{
			rectangles[rectIndex].inner_corners[innerIdx]=aligned_inner[innerIdx];
			rectangles[rectIndex].inner_degrees[innerIdx]=aligned_inner_degrees[innerIdx];
		}

#ifdef VERBOSE
		//fprintf(stdout,"AFTER\n\n");
		printRect(*rectangle);

#endif

	}


#ifdef VERBOSE
	//fprintf(stdout,"END AlignMarkerCorners\n");
#endif


}
/*
//----------------------------------------------
//マーカーの移動ベクトルと回転ベクトルを求めます。
//１回の呼び出しあたり0.3msぐらい
void EstimateMarkerRotation_TranlationVector(
	const CvMat* camera_matrix,
	const CvMat* distortion_coeffs,
	MarkerRectangle* rectangle,
	const int rectCount)
{

	char funcname[50];
	CvPoint3D32f RealMarkerCorners[4];	//Outer 0,1,2,3
	CvPoint2D32f ImgMarkerCorners[4];

	CvMat* rotation_vector;
	CvMat* translation_vector;
	int CornerCount=4;
	int MarkerIDX;
	double outer_base;


	double tx,ty,tz;
	double rx,ry,rz;
	CvMat MatRealCorners;
	CvMat MatImgCorners;
	__android_log_print(ANDROID_LOG_DEBUG,"Estimate TR"," start");

	//sprintf(funcname,"EstimateMarkerRotation_TranlationVector");
	if(NULL==camera_matrix)
	{	//fprintf(stderr,"camera_matrix is NULL!!!!\n");
		__android_log_print(ANDROID_LOG_ERROR,"Estimate TR"," null cam matrix");
	return;
	}

	if(NULL==distortion_coeffs)
	{	//fprintf(stderr,"distortion_coeffs is NULL!!!!\n");
		__android_log_print(ANDROID_LOG_ERROR,"Estimate TR"," null distortion");
	return;
	}


	//まずは必要条件チェック
	if( camera_matrix->rows != 3 || camera_matrix->cols != 3 || CV_MAT_CN(camera_matrix->type) != 1 )
	{
		//fprintf(stderr,"Camera_matrix must be 3x3, single-channel floating point matrix.%s\n",funcname);
		//fprintf(stderr,"row:%d  col:%d   channel:%d\n", camera_matrix->rows, camera_matrix->cols,CV_MAT_CN(camera_matrix->type));
		__android_log_print(ANDROID_LOG_ERROR,"Estimate TR"," cam matrix invalid");
		return;
	}
	if( distortion_coeffs->rows != 1 || distortion_coeffs->cols != 4 || CV_MAT_CN(distortion_coeffs->type) != 1 )
	{
		//fprintf(stderr,"distortion_coeffs must be 1x4, single-channel floating point matrix.%s\n",funcname);
		__android_log_print(ANDROID_LOG_ERROR,"Estimate TR"," distortion invalid");
		return;
	}

	//__android_log_print(ANDROID_LOG_DEBUG,"Estimate TR"," chech finish");
	rotation_vector		= cvCreateMat (1, 3, CV_32FC1);
	translation_vector	= cvCreateMat (1, 3, CV_32FC1);


	//マーカーの外側、内側の現実上での座標を設定します。
	//マーカーに垂直な軸をY軸、マーカーの横をX軸、マーカーの縦をZ軸とします。
	//あと、マーカーの中央部分を(0,0)とします。
	//					->  X +
	//		X
	//			x	x	x
	//			x	x	x
	//			x	x
	//	|
	//	v
	//	Z+

	__android_log_print(ANDROID_LOG_DEBUG,"Estimate TR"," rectCount: %d",rectCount);
	for(MarkerIDX=0;MarkerIDX<rectCount;MarkerIDX++)
	{
		int cornerindex=0;

		for(cornerindex=0;cornerindex<4;cornerindex++)
		{
			//初期化
			RealMarkerCorners[cornerindex]=cvPoint3D32f(0,0,0);
		}

		outer_base=(double)(MARKER_OUTER_SIZE*0.5);

		//外側

		//RealMarkerCorners[0].x =-1*outer_base;
		//RealMarkerCorners[0].z =-1*outer_base;
		//RealMarkerCorners[1].x =-1*outer_base;
		//RealMarkerCorners[1].z = 1*outer_base;
		//RealMarkerCorners[2].x = 1*outer_base;
		//RealMarkerCorners[2].z = 1*outer_base;
		//RealMarkerCorners[3].x = 1*outer_base;
		//RealMarkerCorners[3].z =-1*outer_base;


		RealMarkerCorners[0].x =-1*outer_base;
		RealMarkerCorners[0].y =-1*outer_base;
		RealMarkerCorners[1].x =-1*outer_base;
		RealMarkerCorners[1].y = 1*outer_base;
		RealMarkerCorners[2].x = 1*outer_base;
		RealMarkerCorners[2].y = 1*outer_base;
		RealMarkerCorners[3].x = 1*outer_base;
		RealMarkerCorners[3].y =-1*outer_base;

		for(cornerindex=0;cornerindex<CornerCount;cornerindex++)
		{
			//初期化
			ImgMarkerCorners[cornerindex]=rectangle[MarkerIDX].outer_corners[cornerindex];
			//__android_log_print(ANDROID_LOG_INFO,"ImgMarker Corner","x: %f, y: %f", ImgMarkerCorners[cornerindex].x,ImgMarkerCorners[cornerindex].y);
		}


#ifdef VERBOSE
		//デバッグプリント
		//現実上の位置
		int i=0;
		for(i=0;i<MARKER_CORNER_COUNT;i++)
		{
			//fprintf(stdout,"Real corner:%d  X:%g\tY:%g\tZ:%g\n"	,i,(double)RealMarkerCorners[i].x,(double)RealMarkerCorners[i].y,(double)RealMarkerCorners[i].z);
			//fprintf(stdout,"Img  corner:%d  X:%g\tY:%g	\n"		,i,(double)ImgMarkerCorners[i].x,(double)ImgMarkerCorners[i].y);
		}
#endif

		//配列の集まりをCvMatで参照できるようにする
		cvInitMatHeader (&MatRealCorners,	MARKER_CORNER_COUNT, 1, CV_32FC3, RealMarkerCorners,CV_AUTOSTEP);
		cvInitMatHeader (&MatImgCorners,	MARKER_CORNER_COUNT, 1, CV_32FC2, ImgMarkerCorners,CV_AUTOSTEP);


		//Translation vectorとRotation vector を求める。
		cvFindExtrinsicCameraParams2 (&MatRealCorners, &MatImgCorners, camera_matrix, distortion_coeffs, rotation_vector, translation_vector,0);

		//移動ベクトルと回転ベクトルを構造体に格納
		tx=(float)cvmGet(translation_vector,0,0);
		ty=(float)cvmGet(translation_vector,0,1);
		tz=(float)cvmGet(translation_vector,0,2);
		rectangle[MarkerIDX].translation_vector=cvPoint3D32f(tx,ty,tz);

		rx=cvmGet(rotation_vector,0,0);
		ry=cvmGet(rotation_vector,0,1);
		rz=cvmGet(rotation_vector,0,2);
		rectangle[MarkerIDX].rotation_vector=cvPoint3D32f(rx,ry,rz);

	}

	//__android_log_print(ANDROID_LOG_DEBUG,"Estimate TR"," finish");
	//掃除
	cvReleaseMat(&rotation_vector);
	cvReleaseMat(&translation_vector);



}
*/
void EstimateMarkerRotation_TranlationVector(
	const CvMat* camera_matrix,
	const CvMat* distortion_coeffs,
	MarkerRectangle* rectangle,
	const int rectCount)
{

	char funcname[50];
	//CvPoint3D32f RealMarkerCorners[4];	//Outer 0,1,2,3
	//CvPoint2D32f ImgMarkerCorners[4];
	__android_log_print(ANDROID_LOG_INFO,"Estimate TR"," start");
	//CvMat* RealMarkerCorners;
	//CvMat* ImgMarkerCorners;



	//CvMat* EstimateRotation_vector;
	//CvMat* EstimateTranslation_vector;
	int CornerCount=4;
	int MarkerIDX;
	//double outer_base;


	double tx,ty,tz;
	double rx,ry,rz;
	//CvMat MatRealCorners;
	//CvMat MatImgCorners;


	//sprintf(funcname,"EstimateMarkerRotation_TranlationVector");
	if(NULL==camera_matrix)
	{	//fprintf(stderr,"camera_matrix is NULL!!!!\n");
		__android_log_print(ANDROID_LOG_ERROR,"Estimate TR"," null cam matrix");
	return;
	}

	if(NULL==distortion_coeffs)
	{	//fprintf(stderr,"distortion_coeffs is NULL!!!!\n");
		__android_log_print(ANDROID_LOG_ERROR,"Estimate TR"," null distortion");
	return;
	}


	//まずは必要条件チェック
	if( camera_matrix->rows != 3 || camera_matrix->cols != 3 || CV_MAT_CN(camera_matrix->type) != 1 )
	{
		//fprintf(stderr,"Camera_matrix must be 3x3, single-channel floating point matrix.%s\n",funcname);
		//fprintf(stderr,"row:%d  col:%d   channel:%d\n", camera_matrix->rows, camera_matrix->cols,CV_MAT_CN(camera_matrix->type));
		__android_log_print(ANDROID_LOG_ERROR,"Estimate TR"," cam matrix invalid");
		return;
	}
	if( distortion_coeffs->rows != 1 || distortion_coeffs->cols != 4 || CV_MAT_CN(distortion_coeffs->type) != 1 )
	{
		//fprintf(stderr,"distortion_coeffs must be 1x4, single-channel floating point matrix.%s\n",funcname);
		__android_log_print(ANDROID_LOG_ERROR,"Estimate TR"," distortion invalid");
		return;
	}

	//__android_log_print(ANDROID_LOG_DEBUG,"Estimate TR"," chech finish");
	//Estimaterotation_vector		= cvCreateMat (1, 3, CV_32FC1);
	//Estimatetranslation_vector	= cvCreateMat (1, 3, CV_32FC1);


	//マーカーの外側、内側の現実上での座標を設定します。
	//マーカーに垂直な軸をY軸、マーカーの横をX軸、マーカーの縦をZ軸とします。
	//あと、マーカーの中央部分を(0,0)とします。
	//					->  X +
	//		X
	//			x	x	x
	//			x	x	x
	//			x	x
	//	|
	//	v
	//	Z+

	__android_log_print(ANDROID_LOG_DEBUG,"Estimate TR"," rectCount: %d",rectCount);
	for(MarkerIDX=0;MarkerIDX<rectCount;MarkerIDX++)
	{
		int cornerindex=0;

		/*
		for(cornerindex=0;cornerindex<4;cornerindex++)
		{
			//初期化
			RealMarkerCorners[cornerindex]=cvPoint3D32f(0,0,0);
		}
		*/
		//outer_base=(double)(MARKER_OUTER_SIZE*0.5);

		//外側

		//RealMarkerCorners[0].x =-1*outer_base;
		//RealMarkerCorners[0].z =-1*outer_base;
		//RealMarkerCorners[1].x =-1*outer_base;
		//RealMarkerCorners[1].z = 1*outer_base;
		//RealMarkerCorners[2].x = 1*outer_base;
		//RealMarkerCorners[2].z = 1*outer_base;
		//RealMarkerCorners[3].x = 1*outer_base;
		//RealMarkerCorners[3].z =-1*outer_base;


		//RealMarkerCorners[0].x =-1*outer_base;
		//RealMarkerCorners[0].y =-1*outer_base;
		//RealMarkerCorners[1].x =-1*outer_base;
		//RealMarkerCorners[1].y = 1*outer_base;
		//RealMarkerCorners[2].x = 1*outer_base;
		//RealMarkerCorners[2].y = 1*outer_base;
		//RealMarkerCorners[3].x = 1*outer_base;
		//RealMarkerCorners[3].y =-1*outer_base;

		/*
			CV_MAT_ELEM(*RealMarkerCorners,float,0,0)=-outer_base;
			CV_MAT_ELEM(*RealMarkerCorners,float,0,2)=0.0;
			CV_MAT_ELEM(*RealMarkerCorners,float,0,1)=-outer_base;

			CV_MAT_ELEM(*RealMarkerCorners,float,1,0)=-outer_base;
			CV_MAT_ELEM(*RealMarkerCorners,float,1,2)=0.0;
			CV_MAT_ELEM(*RealMarkerCorners,float,1,1)=outer_base;


			CV_MAT_ELEM(*RealMarkerCorners,float,2,0)=outer_base;
			CV_MAT_ELEM(*RealMarkerCorners,float,2,2)=0.0;
			CV_MAT_ELEM(*RealMarkerCorners,float,2,1)=outer_base;

			CV_MAT_ELEM(*RealMarkerCorners,float,3,0)=outer_base;
			CV_MAT_ELEM(*RealMarkerCorners,float,3,2)=0.0;
			CV_MAT_ELEM(*RealMarkerCorners,float,3,1)=-outer_base;
		*/
		for(cornerindex=0;cornerindex<CornerCount;cornerindex++)
		{
			//初期化
			//ImgMarkerCorners[cornerindex]=rectangle[MarkerIDX].outer_corners[cornerindex];
			CV_MAT_ELEM(*ImgMarkerCorners,float,cornerindex,0)=rectangle[MarkerIDX].outer_corners[cornerindex].x;
			CV_MAT_ELEM(*ImgMarkerCorners,float,cornerindex,1)=rectangle[MarkerIDX].outer_corners[cornerindex].y;
			//__android_log_print(ANDROID_LOG_INFO,"ImgMarker Corner","x: %f, y: %f", ImgMarkerCorners[cornerindex].x,ImgMarkerCorners[cornerindex].y);
		}


#ifdef VERBOSE
		//デバッグプリント
		//現実上の位置
		int i=0;
		for(i=0;i<MARKER_CORNER_COUNT;i++)
		{
			//fprintf(stdout,"Real corner:%d  X:%g\tY:%g\tZ:%g\n"	,i,(double)RealMarkerCorners[i].x,(double)RealMarkerCorners[i].y,(double)RealMarkerCorners[i].z);
			//fprintf(stdout,"Img  corner:%d  X:%g\tY:%g	\n"		,i,(double)ImgMarkerCorners[i].x,(double)ImgMarkerCorners[i].y);
		}
#endif

		//配列の集まりをCvMatで参照できるようにする
		//cvInitMatHeader (&MatRealCorners,	MARKER_CORNER_COUNT, 1, CV_32FC3, RealMarkerCorners,CV_AUTOSTEP);
		//cvInitMatHeader (&MatImgCorners,	MARKER_CORNER_COUNT, 1, CV_32FC2, ImgMarkerCorners,CV_AUTOSTEP);


		//Translation vectorとRotation vector を求める。
		cvFindExtrinsicCameraParams2 (RealMarkerCorners, ImgMarkerCorners, camera_matrix, distortion_coeffs, EstimateRotation_vector, EstimateTranslation_vector,0);
		//cv::solvePnP (RealMarkerCorners, ImgMarkerCorners, camera_matrix, distortion_coeffs, EstimateRotation_vector, EstimateTranslation_vector,0);

		//移動ベクトルと回転ベクトルを構造体に格納
		tx=(float)cvmGet(EstimateTranslation_vector,0,0);
		ty=(float)cvmGet(EstimateTranslation_vector,0,1);
		tz=(float)cvmGet(EstimateTranslation_vector,0,2);
		rectangle[MarkerIDX].translation_vector=cvPoint3D32f(tx,ty,tz);

		rx=cvmGet(EstimateRotation_vector,0,0);
		ry=cvmGet(EstimateRotation_vector,0,1);
		rz=cvmGet(EstimateRotation_vector,0,2);
		rectangle[MarkerIDX].rotation_vector=cvPoint3D32f(rx,ry,rz);

	}

	//__android_log_print(ANDROID_LOG_DEBUG,"Estimate TR"," finish");
	//掃除
	//cvReleaseMat(&rotation_vector);
	//cvReleaseMat(&translation_vector);
	//cvReleaseMat(&RealMarkerCorners);
	//cvReleaseMat(&ImgMarkerCorners);



}
//------------------------------------------------------------
//座標軸を書きます。
//マーカーの位置、回転ベクトル、移動ベクトル、
//カメラの定数などはあらかじめ計算しておいてください
//------------------------------------------------------------
void drawAxis(
	IplImage* colorImg,
	const CvMat* camera_matrix,
	const CvMat* distortion_coeffs,
	MarkerRectangle* rectangle,
	const int rectCount)
{
	CvPoint axisPoints[4];//原点、x,y,z軸
	char funcname[50];
	CvMat* rotation_vector;
	CvMat* translation_vector;
	int i=0;
	CvMat* RealAxisPoints3D;
	CvMat* ImgAxisPoints2D;
	int rectIndex=0;

	//sprintf(funcname,"drawAxis");
	if(NULL==camera_matrix)
	{	//fprintf(stderr,"camera_matrix is NULL!!!!\n");
	return;
	}

	if(NULL==distortion_coeffs)
	{	//fprintf(stderr,"distortion_coeffs is NULL!!!!\n");
	return;
	}


	//まずは必要条件チェック
	if( camera_matrix->rows != 3 || camera_matrix->cols != 3 || CV_MAT_CN(camera_matrix->type) != 1 )
	{
		//fprintf(stderr,"Camera_matrix must be 3x3, single-channel floating point matrix.%s\n",funcname);
		//fprintf(stderr,"row:%d  col:%d   channel:%d\n", camera_matrix->rows, camera_matrix->cols,CV_MAT_CN(camera_matrix->type));
		return;
	}
	if( distortion_coeffs->rows != 1 || distortion_coeffs->cols != 4 || CV_MAT_CN(distortion_coeffs->type) != 1 )
	{
		//fprintf(stderr,"distortion_coeffs must be 1x4, single-channel floating point matrix.%s\n",funcname);
		return;
	}



	rotation_vector		= cvCreateMat (1, 3, CV_32FC1);
	translation_vector	= cvCreateMat (1, 3, CV_32FC1);


	RealAxisPoints3D=cvCreateMat (4,3, CV_32FC1);
	ImgAxisPoints2D=cvCreateMat (4,2, CV_32FC1);


	//軸座標初期化
	for(i=0;i<4;i++)
	{
		CV_MAT_ELEM(*RealAxisPoints3D,float,i,0)=0.0;
		CV_MAT_ELEM(*RealAxisPoints3D,float,i,1)=0.0;
		CV_MAT_ELEM(*RealAxisPoints3D,float,i,2)=0.0;
		CV_MAT_ELEM(*ImgAxisPoints2D,float,i,0)=0.0;
		CV_MAT_ELEM(*ImgAxisPoints2D,float,i,1)=0.0;
	}

	CV_MAT_ELEM(*RealAxisPoints3D,float,1,0)=MARKER_OUTER_SIZE/2;//X axis
	CV_MAT_ELEM(*RealAxisPoints3D,float,2,1)=MARKER_OUTER_SIZE/2;//Y axis
	CV_MAT_ELEM(*RealAxisPoints3D,float,3,2)=MARKER_OUTER_SIZE/2;//Z axis

	//used for draw corners
	CvMat* corners3D=cvCreateMat(4,3,CV_32FC1);
	CvMat* corners2D=cvCreateMat(4,2,CV_32FC1);
	CV_MAT_ELEM(*corners3D,float,0,0)=-MARKER_OUTER_SIZE/2;
	CV_MAT_ELEM(*corners3D,float,0,1)=-MARKER_OUTER_SIZE/2;
	CV_MAT_ELEM(*corners3D,float,0,2)=0.0;

	CV_MAT_ELEM(*corners3D,float,1,0)=-MARKER_OUTER_SIZE/2;
	CV_MAT_ELEM(*corners3D,float,1,1)=MARKER_OUTER_SIZE/2;
	CV_MAT_ELEM(*corners3D,float,1,2)=0.0;

	CV_MAT_ELEM(*corners3D,float,2,0)=MARKER_OUTER_SIZE/2;
	CV_MAT_ELEM(*corners3D,float,2,1)=MARKER_OUTER_SIZE/2;
	CV_MAT_ELEM(*corners3D,float,2,2)=0.0;

	CV_MAT_ELEM(*corners3D,float,3,0)=MARKER_OUTER_SIZE/2;
	CV_MAT_ELEM(*corners3D,float,3,1)=-MARKER_OUTER_SIZE/2;
	CV_MAT_ELEM(*corners3D,float,3,2)=0.0;

	CvPoint cir;

	for(rectIndex=0;rectIndex<rectCount;rectIndex++)
	{

		//Rotation vectorとTranslation vectorを取り込む

		//cvmSet(rotation_vector,0,0,rectangle[rectIndex].rotation_vector.x);
		//cvmSet(rotation_vector,0,1,rectangle[rectIndex].rotation_vector.y);
		//cvmSet(rotation_vector,0,2,rectangle[rectIndex].rotation_vector.z);
		CV_MAT_ELEM(*rotation_vector,float,0,0)=rectangle[rectIndex].rotation_vector.x;
		CV_MAT_ELEM(*rotation_vector,float,0,1)=rectangle[rectIndex].rotation_vector.y;
		CV_MAT_ELEM(*rotation_vector,float,0,2)=rectangle[rectIndex].rotation_vector.z;


		//cvmSet(translation_vector,0,0,rectangle[rectIndex].translation_vector.x);
		//cvmSet(translation_vector,0,1,rectangle[rectIndex].translation_vector.y);
		//cvmSet(translation_vector,0,2,rectangle[rectIndex].translation_vector.z);
		CV_MAT_ELEM(*translation_vector,float,0,0)=rectangle[rectIndex].translation_vector.x;
		CV_MAT_ELEM(*translation_vector,float,0,1)=rectangle[rectIndex].translation_vector.y;
		CV_MAT_ELEM(*translation_vector,float,0,2)=rectangle[rectIndex].translation_vector.z;


		cvProjectPoints2(RealAxisPoints3D,
			rotation_vector,translation_vector,
			camera_matrix,distortion_coeffs,ImgAxisPoints2D,
			NULL,NULL,NULL,NULL,NULL,0);



		cvProjectPoints2(corners3D,
			rotation_vector,translation_vector,
			camera_matrix,distortion_coeffs,corners2D,
			NULL,NULL,NULL,NULL,NULL,0);
		//てすと


		/*
		for(i=0;i<4;i++)
		{

		//fprintf(stdout,"AxisPoint %d:   (x,y)= (%g,%g)\n",i,cvmGet(ImgAxisPoints2D,0,i),cvmGet(ImgAxisPoints2D,1,i));

		}
		*/

		for(i=0;i<4;i++)
		{
			axisPoints[i]=cvPoint(
				ceil(cvmGet(ImgAxisPoints2D,i,0)),
				ceil(cvmGet(ImgAxisPoints2D,i,1))
				);
		}

		//x軸　赤　Y軸　G　Z軸　Bにする
		cvLine(colorImg,axisPoints[0], axisPoints[1],CV_RGB(255,0,0),2,8,0);	//x	axis
		cvLine(colorImg,axisPoints[0], axisPoints[2],CV_RGB(0,255,0),2,8,0);	//y axis
		cvLine(colorImg,axisPoints[0], axisPoints[3],CV_RGB(0,0,255),2,8,0);	//z axis

        cir.x=(int)CV_MAT_ELEM(*corners2D,float,0,0);
        cir.y=(int)CV_MAT_ELEM(*corners2D,float,0,1);
        cvCircle(colorImg,cir,7,CV_RGB(255,0,0),2,8,0);

        cir.x=(int)CV_MAT_ELEM(*corners2D,float,1,0);
        cir.y=(int)CV_MAT_ELEM(*corners2D,float,1,1);
        cvCircle(colorImg,cir,7,CV_RGB(0,255,0),2,8,0);

        cir.x=(int)CV_MAT_ELEM(*corners2D,float,2,0);
        cir.y=(int)CV_MAT_ELEM(*corners2D,float,2,1);
        cvCircle(colorImg,cir,7,CV_RGB(0,0,255),2,8,0);

        cir.x=(int)CV_MAT_ELEM(*corners2D,float,3,0);
        cir.y=(int)CV_MAT_ELEM(*corners2D,float,3,1);
        cvCircle(colorImg,cir,7,CV_RGB(255,255,0),2,8,0);
	}

	//掃除
	cvReleaseMat(&rotation_vector);
	cvReleaseMat(&translation_vector);
	cvReleaseMat(&RealAxisPoints3D);
	cvReleaseMat(&ImgAxisPoints2D);

	cvReleaseMat(&corners3D);
	cvReleaseMat(&corners2D);


}

void setMarkerDetectScreenResolution(int w,int h)
{
	mScreenWidth=w;
	mScreenHeight=h;
}
void initGlobalVars()
{
	RealMarkerCorners=cvCreateMat(4, 3, CV_32FC1);
	ImgMarkerCorners=cvCreateMat(4,2,CV_32FC1);

	double outer_base;
	outer_base=(double)(MARKER_OUTER_SIZE*0.5);
	EstimateRotation_vector		= cvCreateMat (1, 3, CV_32FC1);
	EstimateTranslation_vector	= cvCreateMat (1, 3, CV_32FC1);
	CV_MAT_ELEM(*RealMarkerCorners,float,0,0)=-outer_base;
	CV_MAT_ELEM(*RealMarkerCorners,float,0,1)=-outer_base;
	//CV_MAT_ELEM(*RealMarkerCorners,float,0,0)=outer_base;
	//CV_MAT_ELEM(*RealMarkerCorners,float,0,1)=outer_base;
	CV_MAT_ELEM(*RealMarkerCorners,float,0,2)=0.0;

	CV_MAT_ELEM(*RealMarkerCorners,float,1,0)=-outer_base;
	CV_MAT_ELEM(*RealMarkerCorners,float,1,1)=outer_base;
	CV_MAT_ELEM(*RealMarkerCorners,float,1,2)=0.0;

	CV_MAT_ELEM(*RealMarkerCorners,float,2,0)=outer_base;
	CV_MAT_ELEM(*RealMarkerCorners,float,2,1)=outer_base;
	//CV_MAT_ELEM(*RealMarkerCorners,float,2,0)=-outer_base;
	//CV_MAT_ELEM(*RealMarkerCorners,float,2,1)=-outer_base;
	CV_MAT_ELEM(*RealMarkerCorners,float,2,2)=0.0;

	CV_MAT_ELEM(*RealMarkerCorners,float,3,0)=outer_base;
	CV_MAT_ELEM(*RealMarkerCorners,float,3,1)=-outer_base;
	CV_MAT_ELEM(*RealMarkerCorners,float,3,2)=0.0;
}

void correctMarkerPoints(MarkerRectangle & MR)
{
    float x;
    float y;
    switch(MR.Direction)
    {
    case DIRECTION_BL:
        x=MR.outer_corners[0].x;
        y=MR.outer_corners[0].y;
        MR.outer_corners[0].x=MR.outer_corners[1].x;
        MR.outer_corners[0].y=MR.outer_corners[1].y;
        MR.outer_corners[1].x=MR.outer_corners[2].x;
        MR.outer_corners[1].y=MR.outer_corners[2].y;
        MR.outer_corners[2].x=MR.outer_corners[3].x;
        MR.outer_corners[2].y=MR.outer_corners[3].y;
        MR.outer_corners[3].x=x;
        MR.outer_corners[3].y=y;
        return;
    case DIRECTION_BR:
        x=MR.outer_corners[0].x;
        y=MR.outer_corners[0].y;
        MR.outer_corners[0].x=MR.outer_corners[2].x;
        MR.outer_corners[0].y=MR.outer_corners[2].y;
        MR.outer_corners[2].x=x;
        MR.outer_corners[2].y=y;
        x=MR.outer_corners[1].x;
        y=MR.outer_corners[1].y;
        MR.outer_corners[1].x=MR.outer_corners[3].x;
        MR.outer_corners[1].y=MR.outer_corners[3].y;
        MR.outer_corners[3].x=x;
        MR.outer_corners[3].y=y;
        return;
    case DIRECTION_TR:
        x=MR.outer_corners[0].x;
        y=MR.outer_corners[0].y;
        MR.outer_corners[0].x=MR.outer_corners[3].x;
        MR.outer_corners[0].y=MR.outer_corners[3].y;
        MR.outer_corners[3].x=MR.outer_corners[2].x;
        MR.outer_corners[3].y=MR.outer_corners[2].y;
        MR.outer_corners[2].x=MR.outer_corners[1].x;
        MR.outer_corners[2].y=MR.outer_corners[1].y;
        MR.outer_corners[1].x=x;
        MR.outer_corners[1].y=y;

        return;
    }
}

void correctMarkerPoints(MarkerRectangle & MR,int Direction)
{
    float x;
    float y;
    switch(Direction)
    {
    case DIRECTION_BL://0<-1, 1<-2, 2<-3, 3<-0
        x=MR.outer_corners[0].x;
        y=MR.outer_corners[0].y;
        MR.outer_corners[0].x=MR.outer_corners[1].x;
        MR.outer_corners[0].y=MR.outer_corners[1].y;
        MR.outer_corners[1].x=MR.outer_corners[2].x;
        MR.outer_corners[1].y=MR.outer_corners[2].y;
        MR.outer_corners[2].x=MR.outer_corners[3].x;
        MR.outer_corners[2].y=MR.outer_corners[3].y;
        MR.outer_corners[3].x=x;
        MR.outer_corners[3].y=y;
        return;
    case DIRECTION_BR://0<-2,2<-0,  1<-3,3<-1
        x=MR.outer_corners[0].x;
        y=MR.outer_corners[0].y;
        MR.outer_corners[0].x=MR.outer_corners[2].x;
        MR.outer_corners[0].y=MR.outer_corners[2].y;
        MR.outer_corners[2].x=x;
        MR.outer_corners[2].y=y;
        x=MR.outer_corners[1].x;
        y=MR.outer_corners[1].y;
        MR.outer_corners[1].x=MR.outer_corners[3].x;
        MR.outer_corners[1].y=MR.outer_corners[3].y;
        MR.outer_corners[3].x=x;
        MR.outer_corners[3].y=y;
        return;
    case DIRECTION_TR://0<-3,3<-2,2<-1,1<-0
        x=MR.outer_corners[0].x;
        y=MR.outer_corners[0].y;
        MR.outer_corners[0].x=MR.outer_corners[3].x;
        MR.outer_corners[0].y=MR.outer_corners[3].y;
        MR.outer_corners[3].x=MR.outer_corners[2].x;
        MR.outer_corners[3].y=MR.outer_corners[2].y;
        MR.outer_corners[2].x=MR.outer_corners[1].x;
        MR.outer_corners[2].y=MR.outer_corners[1].y;
        MR.outer_corners[1].x=x;
        MR.outer_corners[1].y=y;

        return;
    }
}

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
