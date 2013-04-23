#include "findchildrenboundboxnodevisitor.h"

findChildrenBoundBoxNodeVisitor::findChildrenBoundBoxNodeVisitor():osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
{

}
void findChildrenBoundBoxNodeVisitor::apply(osg::Group &node)
{
        traverse(node);
}
void findChildrenBoundBoxNodeVisitor::apply(osg::Geode &node)
{
    geodeList.push_back(&node);
}
