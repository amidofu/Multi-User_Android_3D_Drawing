#include "UpdateOtherPlayerCamCallback.h"

void UpdateOtherPlayerCamCallback::setPosRot(osg::Vec3 pos,osg::Quat rot)
{
	_pos=pos;
	_rot=rot;
}
UpdateOtherPlayerCamCallback::UpdateOtherPlayerCamCallback()
{
	_pos.set(0.0,0.0,0.0);
	_rot.makeRotate(0.0,1.0,0.0,0.0);
	FixInitialRot.makeRotate(-3.1415926535/2,1.0,0.0,0.0);
}
void UpdateOtherPlayerCamCallback::operator()(osg::Node* node,osg::NodeVisitor* nv)
{
	osg::ref_ptr<osg::MatrixTransform> mt=dynamic_cast<osg::MatrixTransform*>(node);
	mt->setMatrix(osg::Matrixd::rotate( FixInitialRot*_rot )*osg::Matrixd::translate( _pos ));
	traverse(node,nv);
}
