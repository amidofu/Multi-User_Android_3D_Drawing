#ifndef POLYTOPEPICKHANDLER_H
#define POLYTOPEPICKHANDLER_H
#include <osgUtil/PolytopeIntersector>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/PolygonMode>
#include <osgDB/ReadFile>
#include <android/log.h>
class PolytopePickHandler: public osgGA::GUIEventHandler
{
public:
    PolytopePickHandler();
    float _mx,_my;

    virtual bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa);
    virtual bool performMovementLeftMouseButton( const double eventTimeDelta, const double dx, const double dy );
    void pick(const osgGA::GUIEventAdapter& ea,osgViewer::Viewer* viewer);
    osg::Node* getOrCreateSelectionBox();
    osg::ref_ptr<osg::MatrixTransform> _selectionBox;
    int currentActArea;
    int currentActGeode;
    bool NewAction;

    osg::ref_ptr<osg::Node> currentSelectedNode;
    bool NodeForSendSelected;

    osg::ref_ptr<osg::Drawable> currentSelectedDrawable;
    osgUtil::PolytopeIntersector::Intersection currentInsersection;
    osg::Vec3 currentSelectedDrawableWroldCenter;

    bool NodeSelected();
    bool selected;
};

#endif // POLYTOPEPICKHANDLER_H
