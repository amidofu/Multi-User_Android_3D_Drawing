#include <osg/NodeCallback>
#include <osg/MatrixTransform>
#include <osg/Vec3>
#include <osg/Quat>
class UpdateOtherPlayerCamCallback:public osg::NodeCallback
{
public:
	UpdateOtherPlayerCamCallback();
	//another user's position
	osg::Vec3 _pos;
	//another user's orientation
	osg::Quat _rot;
	//update another user's pose
	void setPosRot(osg::Vec3 pos,osg::Quat rot);
	virtual void operator()(osg::Node* node,osg::NodeVisitor* nv);
	//reset orientation
	osg::Quat FixInitialRot;
};
