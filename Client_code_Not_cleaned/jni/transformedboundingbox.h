#ifndef TRANSFORMEDBOUNDINGBOX_H
#define TRANSFORMEDBOUNDINGBOX_H
#include <osg/BoundingBox>
#include <osg/Geometry>
#include "getworldcoordofnodevisitor.h"
#include <osg/Matrix>
#include <iostream>
class TransformedBoundingBox: public osg::BoundingBox
{
public:
    TransformedBoundingBox();
    void expandByWorldCood(osg::Drawable &  drawable);
    static void computeWorldCoordBoundingBox(osg::BoundingBox & bb, osg::Drawable & drawable);
    static void computeBoundingBoxByMatrix(osg::BoundingBox &bb, osg::Matrix mat);
    static osg::Matrixd* getWorldCoords(osg::Node* node);
};

#endif // TRANSFORMEDBOUNDINGBOX_H
