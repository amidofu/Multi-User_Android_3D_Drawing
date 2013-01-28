#include "KeyFrame.h"
int KeyFrame::ImageWidth=0;
int KeyFrame::ImageHeight=0;
KeyFrame::KeyFrame()
{
	valid=false;
}
void KeyFrame::updateKeyFrame(std::vector<Point2f> & features,std::vector<Point3f>  & WorldPts,Mat & keyFrame)
{
	imgFeatures.clear();
	sWPts.clear();
	/*
	for(int i=0;i<features.size();i++)
	{
		imgFeatures.push_back(features.at(i));
		sWPts.push_back(WorldPts.at(i));
	}
	*/
	imgFeatures=features;
	sWPts=WorldPts;
	image=Mat(keyFrame);
}

void KeyFrame::updateFeatures(std::vector<Point2f> & select2f, std::vector<Point3f> & select3f)
{
	imgFeatures.clear();
	sWPts.clear();
	imgFeatures=select2f;
	sWPts=select3f;
}

void KeyFrame::clearPts()
{
	imgFeatures.clear();
	sWPts.clear();
	valid=false;
}
