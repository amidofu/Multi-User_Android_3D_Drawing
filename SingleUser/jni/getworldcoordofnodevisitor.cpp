#include "getworldcoordofnodevisitor.h"

getWorldCoordOfNodeVisitor::getWorldCoordOfNodeVisitor():osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_PARENTS)
{

    done=false;
    wcMatrix=new osg::Matrix;
}
void getWorldCoordOfNodeVisitor::apply(osg::Node &node)
 {
     if(!done)
     {
         if(0==node.getNumParents())//got root
         {
             wcMatrix->set(osg::computeLocalToWorld(this->getNodePath()));
            done=true;
         }
         traverse(node);
     }
 }
