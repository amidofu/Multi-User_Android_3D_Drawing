#include "transformedboundingbox.h"

TransformedBoundingBox::TransformedBoundingBox():osg::BoundingBox()
{
}
void TransformedBoundingBox::expandByWorldCood(osg::Drawable &drawable)
{
    osg::BoundingBox bb=drawable.getBound();
    osg::Matrixd* wcMat=TransformedBoundingBox::getWorldCoords(drawable.getParent(0));
    osg::BoundingBox transformedBB;
    for(unsigned int i=0;i<8;i++)
    {
        transformedBB.expandBy(wcMat->preMult(bb.corner(i)));
    }
    this->set(transformedBB.xMin(),transformedBB.yMin(),transformedBB.zMin(),transformedBB.xMax(),transformedBB.yMax(),transformedBB.zMax());

}
osg::Matrixd* TransformedBoundingBox::getWorldCoords(osg::Node* node)
{
    getWorldCoordOfNodeVisitor* ncv=new getWorldCoordOfNodeVisitor();
    if(node&&ncv)
    {
        node->accept(*ncv);
        return ncv->wcMatrix;
    }
    else
        return NULL;
}
void TransformedBoundingBox::computeWorldCoordBoundingBox(osg::BoundingBox & bb, osg::Drawable & drawable)
{
    osg::BoundingBox bb2=drawable.getBound();
    osg::Matrixd* wcMat=TransformedBoundingBox::getWorldCoords(drawable.getParent(0));
    if (wcMat==0)
        return;
    osg::BoundingBox transformedBB;
    for(unsigned int i=0;i<8;i++)
    {
        transformedBB.expandBy(wcMat->preMult(bb2.corner(i)));
    }
    bb.set(transformedBB.xMin(),transformedBB.yMin(),transformedBB.zMin(),transformedBB.xMax(),transformedBB.yMax(),transformedBB.zMax());
}
void TransformedBoundingBox::computeBoundingBoxByMatrix(osg::BoundingBox &bb, osg::Matrix mat)
{
    osg::BoundingBox temp;
    for(unsigned int i=0;i<8;i++)
    {
        temp.expandBy(mat.preMult(bb.corner(i)));
    }
    bb.set(temp.xMin(),bb.yMin(),bb.zMin(),bb.xMax(),bb.yMax(),bb.zMax());
}
