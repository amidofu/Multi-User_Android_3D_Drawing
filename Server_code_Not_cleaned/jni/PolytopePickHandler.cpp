#include "PolytopePickHandler.h"

PolytopePickHandler::PolytopePickHandler()
{
    _mx=0.0;
    _my=0.0;
    NewAction=false;
    selected=false;
}

bool PolytopePickHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
    osgViewer::Viewer* viewer=dynamic_cast<osgViewer::Viewer*>(&aa);
    if(!viewer)
        return false;

    switch(ea.getEventType())
    {
    case(osgGA::GUIEventAdapter::PUSH):
    case(osgGA::GUIEventAdapter::MOVE):
    {
        _mx=ea.getX();
        _my=ea.getY();
        return false;
    }

    case(osgGA::GUIEventAdapter::RELEASE):
    {
        if(_mx==ea.getX() && _my==ea.getY())
        {
            pick(ea,viewer);
        }
        return true;
    }
    default:
        return false;
    }
}
bool PolytopePickHandler::performMovementLeftMouseButton( const double eventTimeDelta, const double dx, const double dy )
{
	return false;
}
void PolytopePickHandler::pick(const osgGA::GUIEventAdapter &ea, osgViewer::Viewer *viewer)
{
    osg::Node* scene=viewer->getSceneData();
    if(!scene)
        return;

    osg::Node* node=0;
    osg::Group* parent=0;

    osgUtil::PolytopeIntersector* picker;
    osg::Viewport* viewport=viewer->getCamera()->getViewport();
    double mx=viewport->x()+(int)((double)viewport->width()*(ea.getXnormalized()*0.5+0.5));
     double my=viewport->y()+(int)((double)viewport->height()*(ea.getYnormalized()*0.5+0.5));
     double w=5.0f;
     double h=5.0f;
     picker=new osgUtil::PolytopeIntersector(osgUtil::Intersector::WINDOW,mx-w,my-h,mx+w,my+h);


     osgUtil::IntersectionVisitor iv(picker);
      iv.setTraversalMask(~0x1);
     viewer->getCamera()->accept(iv);

     if(picker->containsIntersections())
     {
    	 __android_log_write(ANDROID_LOG_DEBUG,"Client","picked something");
         osgUtil::PolytopeIntersector::Intersection result=*(picker->getIntersections().begin());//picker->getFirstIntersection();
         currentInsersection=result;
         osg::BoundingBox bb=result.drawable->getBound();
         osg::Vec3 WorldCenter=bb.center()*osg::computeLocalToWorld(result.nodePath);
         osg::Matrix m=osg::computeLocalToWorld(result.nodePath);
         currentSelectedDrawableWroldCenter=m.postMult(bb.center());



         //compute the bounding box of the selected object
         //osg::Matrixd scale=osg::Matrix::scale(bb.xMax()-bb.xMin(),bb.yMax()-bb.yMin(),bb.zMax()-bb.zMin());
         //osg::Matrixd translate=osg::Matrix::translate(WorldCenter);
         //_selectionBox->setMatrix(scale*translate);
         currentSelectedDrawable=result.drawable;
         currentSelectedNode=result.drawable->getParent(0);
         NodeForSendSelected=true;
         //ActiveGeode* ActGeode=dynamic_cast<ActiveGeode*>(result.drawable->getParent(0));
         /*
         if(ActGeode!=0)
         {
        	 //do the action
        	 ActGeode->action();
        	 currentActArea=ActGeode->_area;
        	 //record ActGeode ID
        	 currentActGeode=ActGeode->_GeodeNum;
        	 NewAction=true;
         }
         */
         selected=true;
     }
     else
    	 selected=false;
}

osg::Node* PolytopePickHandler::getOrCreateSelectionBox()
{
    //new a selection box
    if(!_selectionBox)
    {
        osg::ref_ptr<osg::Geode> geode=new osg::Geode;
        geode->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3(),1.0f)));
        _selectionBox=new osg::MatrixTransform;
        _selectionBox->setNodeMask(0x1);
        _selectionBox->addChild(geode.get());

        osg::StateSet* ss= _selectionBox->getOrCreateStateSet();
        ss->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
        ss->setAttributeAndModes(new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK,osg::PolygonMode::LINE));
    }
    return _selectionBox.get();
}
bool PolytopePickHandler::NodeSelected()
{
	return selected;
	/*
	if(currentSelectedNode==NULL)
		return false;
	else
		return true;
		*/
}
