#include <osg/NodeCallback>
#include <osg/MatrixTransform>
#include <osg/Vec3>
#include <osg/Quat>
class UpdateOtherPlayerCamCallback:public osg::NodeCallback
{
public:
	osg::Vec3 _pos;
	osg::Quat _rot;
	void setPosRot(osg::Vec3 pos,osg::Quat rot);
	UpdateOtherPlayerCamCallback();
	virtual void operator()(osg::Node* node,osg::NodeVisitor* nv);
	osg::Quat FixInitialRot;
};
