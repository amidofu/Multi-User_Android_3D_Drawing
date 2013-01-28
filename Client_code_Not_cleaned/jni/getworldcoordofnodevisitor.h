#ifndef GETWORLDCOORDOFNODEVISITOR_H
#define GETWORLDCOORDOFNODEVISITOR_H
#include <osg/NodeVisitor>
#include <osgViewer/Renderer>
class getWorldCoordOfNodeVisitor:public osg::NodeVisitor
{
public:
    osg::Matrix* wcMatrix;
    bool done;
    getWorldCoordOfNodeVisitor();
    virtual void apply(osg::Node & node);
};

#endif // GETWORLDCOORDOFNODEVISITOR_H
