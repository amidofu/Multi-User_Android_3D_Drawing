#include <osg/NodeCallback>
#include <osg/MatrixTransform>
#include <osg/Vec3>
#include <osg/Quat>
//used to update other pplayer's camera position and orientation
class UpdateOtherPlayerCamCallback:public osg::NodeCallback
{
public:
	osg::Vec3 _pos;
	osg::Quat _rot;
	void setPosRot(osg::Vec3 pos,osg::Quat rot);
	UpdateOtherPlayerCamCallback();
	virtual void operator()(osg::Node* node,osg::NodeVisitor* nv);
	//used to cancel out initial bias of camera manipulator
	osg::Quat FixInitialRot;
};
