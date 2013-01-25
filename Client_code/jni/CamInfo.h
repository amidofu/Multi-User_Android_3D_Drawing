#include <osg/Vec3>
#include <osg/Quat>
#include <osg/Vec4>
class CamInfo
{
public:
	osg::Vec3 CamPos;
	osg::Quat CamRot;
	osg::Vec4 CamRotValue;//angle, rox,roy,roz

	CamInfo();
	void setCamInfo(float posx,float posy, float posz,float rox,float roy,float roz,float angle);
	void setCamInfo(osg::Vec3 eye,osg::Vec4 rot);
	void setCamRot(float rox,float roy,float roz,float angle);
	void setCamRot(osg::Quat rot);
	void setCamPos(float posx,float posy, float posz);
	//void setActGeodeInfo(int pickedArea,int pickedActGeode,bool NewAGeode);

};
