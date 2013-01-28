#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <map>
#include <android/log.h>
#include "mCV.h"
using namespace cv;
typedef std::map<int,int> PtoPMap;
typedef std::pair<int,int> PtoPPair;
typedef PtoPMap::iterator PtoPiter;

class KeyFrame{
public:
	static int ImageWidth;
	static int ImageHeight;
	KeyFrame();
	Mat image;
	std::vector<Point2f> imgFeatures;
	std::vector<Point3f> sWPts;
	bool valid;
	void updateKeyFrame(std::vector<Point2f> & features,std::vector<Point3f>  & WorldPts,Mat & keyFrame);
	void updateFeatures(std::vector<Point2f> & select2f, std::vector<Point3f> & select3f);
	void clearPts();

};

