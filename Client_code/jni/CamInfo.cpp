#include "CamInfo.h"

CamInfo::CamInfo()
{
	CamPos.set(0.0,0.0,0.0);
	CamRot.zeroRotation();
	CamRotValue=CamRot.asVec4();

}
void CamInfo::setCamInfo(float posx,float posy, float posz,float rox,float roy,float roz,float angle)
{
	CamPos.set(posx,posy,posz);
	CamRotValue.set(angle,rox,roy,roz);
	CamRot.makeRotate(angle,rox,roy,roz);
}
void CamInfo::setCamInfo(osg::Vec3 eye,osg::Vec4 rot)
{
	CamPos=eye;
	CamRot.set(rot);
}
void CamInfo::setCamRot(float rox,float roy,float roz,float angle)
{
	CamRotValue.set(angle,rox,roy,roz);
	CamRot.makeRotate(angle,rox,roy,roz);
}
void CamInfo::setCamPos(float posx,float posy, float posz)
{
	CamPos.set(posx,posy,posz);
}
void CamInfo::setCamRot(osg::Quat rot)
{
	CamRot.set(rot.x(),rot.y(),rot.z(),rot.w());
}
/*
void CamInfo::setActGeodeInfo(int pickedArea,int pickedActGeode,bool NewAGeode)
{

}
*/
