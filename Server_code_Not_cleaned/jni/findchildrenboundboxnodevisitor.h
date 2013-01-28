#ifndef FINDCHILDRENBOUNDBOXNODEVISITOR_H
#define FINDCHILDRENBOUNDBOXNODEVISITOR_H
#include <osg/NodeVisitor>
#include <osg/BoundingBox>
#include <osg/Geode>
#include <osg/Group>
//find the bounding box of a geometry after world transformation
typedef std::vector<osg::Geode*> GeodeList;
class findChildrenBoundBoxNodeVisitor:public osg::NodeVisitor
{
public:
    findChildrenBoundBoxNodeVisitor();

    GeodeList geodeList;
    virtual void apply(osg::Group &node);
    virtual void apply(osg::Geode & node);


};

#endif // FINDCHILDRENBOUNDBOXNODEVISITOR_H
