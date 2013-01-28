#ifndef GETWORLDCOORDOFNODEVISITOR_H
#define GETWORLDCOORDOFNODEVISITOR_H
//used to find the overall transformation from root node to a leaf
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
