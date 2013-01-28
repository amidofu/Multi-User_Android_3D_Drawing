#ifndef MARKERFINDER_H
#define MARKERFINDER_H

/*
*  rectFinder.h
*  
*
*  Created by lhaplus8888 on 2012/01/29.
*  四角形を探すクラス。
*/


#include <android/log.h>
#include <stdio.h>
#include <math.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

//#include "mCV.h"

#define GOOD_RECTANGLE (0)
#define GOOD_COLORIMAGE (0)

#define VECTOR_CALCRATED (0)

#define DIRECTION_TL (0)
#define DIRECTION_BL (1)
#define DIRECTION_BR (2)
#define DIRECTION_TR (3)
#define DIRECTION_UNKNOWN (-1)

#define MARKER_OUTER_SIZE (90.0)
#define MARKER_INNER_SIZE (50.0)

#define MARKER_DOTSIZE (10.0)
#define INNER_START (25.0)
#define INNER_END (65.0)
#define MARKER_CORNER_COUNT (4)

//extern int mScreenWidth;
//extern int mScreenHeight;


typedef struct _MarkerRectangle
{
	int MarkerID;//マーカーの番号　　identifier
	CvPoint2D32f outer_corners[4];//コーナーの座標
	double outer_degrees[4];	//コーナーの角度

	CvPoint2D32f inner_corners[4];
	double inner_degrees[4];

	CvPoint3D32f rotation_vector;	//移動ベクトル
	CvPoint3D32f translation_vector;	//回転ベクトル
	int Direction;


} MarkerRectangle;

//カラー画像のチェック。チャネル数、深度など
int CheckColorImage(IplImage* colorImg);


//３点でなす角度を計算します。
double dotDegree(CvPoint common,CvPoint a,CvPoint b);


//四角形かどうかをチェックします。
//チェック方法：内積をとって,60度0120度の間にいるかをチェック
//return 0:四角形と認定(good rectangle)　　-1:四角形ではない(not rectangle)
//正しい四角だったら、MarkerRectangleの情報を更新します。
int CheckRectangle(CvSeq* contour,CvSeq* inner_contour, MarkerRectangle* rectangle);

//--------------------------------------------------------------------------
//四角形を探します。
//	colorImg: 元画像
//	rectangles:結果格納先
//  rectlimitCount:最大検出数
//	戻り値： 見つけた四角形の数。
//--------------------------------------------------------------------------
int FindRectangle(IplImage* colorImg,MarkerRectangle* rectangles,const int rectLimitCount);

//マーカーの頂点を並び替えます。
void AlignMarkerCorners(MarkerRectangle* rectangles,const int rectCount);



//マーカーの移動ベクトルと回転ベクトルを求めます。
//rectangle: stores the translation and rotation
//rectCount: number of rectangles
//camera_matrix fx 0 cx
//              0 fy cy
//              0  0  1
// distortion_coeffs: 1x4, k1,k2,p1,p2  (guess)
void EstimateMarkerRotation_TranlationVector(const CvMat* camera_matrix,
	const CvMat* distortion_coeffs,
	MarkerRectangle* rectangle,
	const int rectCount);


//座標軸を書きます。
void drawAxis(IplImage* colorImg,
	const CvMat* camera_matrix,
	const CvMat* distortion_coeffs,
	MarkerRectangle* rectangle,
	const int rectCount);

//--------------------------------------------------------------------------
//マーカーをデコードして、番号を割りふります。
//この関数を使う前に、FindRectangleを使ってマーカーの頂点情報と角度情報を計算しておいてください。
//	colorImg: 元画像
//	rectangles:頂点／角度が既に求まっているMarkerRectangleの配列。
//  rectCount:検査個数
//	戻り値： 見つけた四角形の数。
//--------------------------------------------------------------------------
void DecodeMarker2DCode(IplImage* colorImg, MarkerRectangle* rectangles,const int rectCount);

int CheckDirection(IplImage* projectedImg);
//int CheckDirection(IplImage* colorImage, IplImage* projectedImg);
void setMarkerDetectScreenResolution(int w,int h);

void initGlobalVars();

void correctMarkerPoints(MarkerRectangle & MR);
void correctMarkerPoints(MarkerRectangle & MR,int Direction);
void drawCornersOneRect(IplImage* colorImg,
	const CvMat* camera_matrix,
	const CvMat* distortion_coeffs,
	MarkerRectangle & rectangle);
#endif
