#ifndef SCENES_H
#define SCENES_H
#include <osg/Geometry>
#include <osg/Material>
#include <osg/Vec4>
#include <osg/Array>
#include <osg/Drawable>
#include <osg/Geode>
#include "findchildrenboundboxnodevisitor.h"
#include "transformedboundingbox.h"

class BoxGeometry:public osg::Geometry{
public:
    BoxGeometry();
    float length;
    float width;
    float height;
    osg::Vec3 color;
    osg::Geometry* getBoxGeometry(float length,float width,float height,osg::Vec4 color);
    osg::Geometry* getBoxGeometry(float length,float width,float height,osg::Vec3 translation,osg::Vec4 color);
    osg::Geometry* getBoxGeometry(float length,float width,float height,osg::Vec3 translation);
    void createBox(float length, float width, float height, osg::Vec4 color);
};

typedef std::map<int, osg::Geode*> IDGeodeMap;
typedef std::map<osg::Geode*,int> ReverseIDGeodeMap;
typedef std::pair<int, osg::Geode*> IDGeodePair;
typedef std::pair<osg::Geode*, int> GeodeIDPair;

void TestNumberedGeodeScene(osg::ref_ptr<osg::Group> & mainGroup,IDGeodeMap & IGM, ReverseIDGeodeMap & RIGM);
void mapIDAndGeode(int ID,osg::Geode* geode,IDGeodeMap & IGM, ReverseIDGeodeMap & RIGM);
void unMapIDAndGeode(int ID,osg::Geode* geode,IDGeodeMap & IGM, ReverseIDGeodeMap & RIGM);
osg::ref_ptr<osg::Geode> Axis();
#endif // SCENES_H
