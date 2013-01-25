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
//create cuboid
class BoxGeometry:public osg::Geometry{
public:
    BoxGeometry();
    float length;
    float width;
    float height;
    osg::Vec3 color;
	//create a box in the origin
    osg::Geometry* getBoxGeometry(float length,float width,float height,osg::Vec4 color);
    //create a box at a certain point (translation)
	osg::Geometry* getBoxGeometry(float length,float width,float height,osg::Vec3 translation,osg::Vec4 color);
    osg::Geometry* getBoxGeometry(float length,float width,float height,osg::Vec3 translation);
    void createBox(float length, float width, float height, osg::Vec4 color);
};
//map each Geode with an ID, ID is used to specify which Geode is selected and notice all users for the Geode by the ID
typedef std::map<int, osg::Geode*> IDGeodeMap;
//used to find out the ID of each Geode
typedef std::map<osg::Geode*,int> ReverseIDGeodeMap;
typedef std::pair<int, osg::Geode*> IDGeodePair;
typedef std::pair<osg::Geode*, int> GeodeIDPair;

//a test scene
void TestNumberedGeodeScene(osg::ref_ptr<osg::Group> & mainGroup,IDGeodeMap & IGM, ReverseIDGeodeMap & RIGM);
//build the ID and Geode mapping
void mapIDAndGeode(int ID,osg::Geode* geode,IDGeodeMap & IGM, ReverseIDGeodeMap & RIGM);
//delete the Geode-ID map entry 
void unMapIDAndGeode(int ID,osg::Geode* geode,IDGeodeMap & IGM, ReverseIDGeodeMap & RIGM);
//show the axis in OSG
osg::ref_ptr<osg::Geode> Axis();
#endif // SCENES_H
