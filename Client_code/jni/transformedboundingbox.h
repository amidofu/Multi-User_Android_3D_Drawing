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

	//expand the BBox of a drawable by the drawable's world coordinates
	void expandByWorldCood(osg::Drawable &  drawable);
	//compute the BBox of a drawable by the drawable's world coordinates (not object coordinates)
    static void computeWorldCoordBoundingBox(osg::BoundingBox & bb, osg::Drawable & drawable);
	//transform a BBox by a matrix
    static void computeBoundingBoxByMatrix(osg::BoundingBox &bb, osg::Matrix mat);
	//get the transformation matrix to world coordinates (traverse the scene graph to the root to find the transformation matrix)
    static osg::Matrixd* getWorldCoords(osg::Node* node);
};

#endif // TRANSFORMEDBOUNDINGBOX_H
