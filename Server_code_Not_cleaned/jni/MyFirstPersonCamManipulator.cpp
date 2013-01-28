#include "MyFirstPersonCamManipulator.h"
MyFirstPersonCamManipulator::MyFirstPersonCamManipulator(float MoveSpeed)
{
	this->MoveSpeed=1.5f;
	this->RotationAngleScalar=0.9f;
	setVerticalAxisFixed( false );
}
MyFirstPersonCamManipulator::MyFirstPersonCamManipulator()
{

}
void MyFirstPersonCamManipulator::moveCameraByAcceleration(float x, float y, float z)
{
	_eye+=osg::Vec3d(x*MoveSpeed,y*MoveSpeed,z*MoveSpeed);
}

void MyFirstPersonCamManipulator::rotateCameraByRotation(float axisX, float axisY, float axisZ, float angle)
{
	osg::Quat rotate;
	rotate.makeRotate(angle*RotationAngleScalar,axisX,axisY,axisZ);
	_rotation=_rotation*rotate;
}
void MyFirstPersonCamManipulator::home(double d)
{
	StandardManipulator::home(0.0);
}
osg::Vec3 MyFirstPersonCamManipulator::getEye()
{
	return _eye;
}

osg::Quat MyFirstPersonCamManipulator::getRotation()
{
	return _rotation;
}
bool MyFirstPersonCamManipulator::handleKeyDown(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &us)
{
	switch (ea.getKey())
	{
		case osgGA::GUIEventAdapter::KEY_KP_Space:
			home(0.0);
			break;

		case 'r':
			//_rotation.makeRotate(3.1415926535/2,1.0,0.0,0.0);
		    {
		        osg::Vec3 eye;
		        osg::Vec3 dir;
		        osg::Vec3 up;
		        getInverseMatrix().getLookAt(eye,dir,up);
		        osg::Vec3 Look=dir-eye;
		        float AngleBetweenAxisAndZ;
		        float dot=osg::Vec3(0.0,0.0,1.0)*up;

		        AngleBetweenAxisAndZ=acos(dot);
		        osg::Quat restore;
		        if(Look.z()>0.0)
		             restore.makeRotate(AngleBetweenAxisAndZ,-1.0,0.0,0.0);
		        else if(Look.z()<0.0)
		             restore.makeRotate(AngleBetweenAxisAndZ,1.0,0.0,0.0);
		        else
		        {}
		        _rotation=restore*_rotation;

		        break;
		    }
		case 'w':
	        if(!move&&(currentKey=='w'))//step back, used to avoid penetrating objects in the scene
	        {
	             _eye+=_rotation*osg::Vec3d(0.0,0.0,2.0);
	             move=true;
	            break;
	        }
	        else if(!move&&(currentKey=='s'))//step back, used to avoid penetrating objects in the scene
	        {
	             _eye+=_rotation*osg::Vec3d(0.0,0.0,-2.0);
	             move=true;
	            //break;
	        }
	        currentKey='w';
            _eye+=_rotation*osg::Vec3d(0.0,0.0,-1.0);
            break;
		case 's':
	        if(!move&&(currentKey=='s'))//step back, used to avoid penetrating objects in the scene
	        {
	             _eye+=_rotation*osg::Vec3d(0.0,0.0,-2.0);
	             move=true;
	            break;
	        }
	        else if(!move&&(currentKey=='w'))//step back, used to avoid penetrating objects in the scene
	        {
	             _eye+=_rotation*osg::Vec3d(0.0,0.0,2.0);
	             move=true;
	            //break;
	        }
	        currentKey='s';
            _eye+=_rotation*osg::Vec3d(0.0,0.0,1.0);
            break;
		default:
			break;
	}
}

void MyFirstPersonCamManipulator::moveEye(osg::Vec3 movement)
{
    _eye+=_rotation*osg::Vec3d(movement.x(),movement.z(),-movement.y());
}
void MyFirstPersonCamManipulator::printEye()
{
   // std::cout<<"Eye: "<<_eye.x()<<" "<<_eye.y()<<" "<<_eye.z()<<std::endl;
}
void MyFirstPersonCamManipulator::setEye(osg::Vec3 newEye)
{
    _eye=newEye;
}
void MyFirstPersonCamManipulator::setEye(float x,float y,float z)
{
	_eye.set(x,y,z);
}
void MyFirstPersonCamManipulator::setRot(const osg::Matrixd mat)
{
	_rotation.set(mat);
}
osg::Matrixd MyFirstPersonCamManipulator::getMatrix() const
{
   return osg::Matrixd::rotate( _rotation ) * osg::Matrixd::translate( _eye );

}
osg::Matrix MyFirstPersonCamManipulator::getInverseMatrix()const
{
    MoveDir=_eye-OldEye;
    OldEye=_eye;
       return osg::Matrixd::translate( -_eye ) * osg::Matrixd::rotate( _rotation.inverse() );

}
void MyFirstPersonCamManipulator::printMoveDir()
{
    //std::cout<<"MoveDir: "<<MoveDir.x()<<" "<<MoveDir.y()<<" "<<MoveDir.z()<<std::endl;
}

void MyFirstPersonCamManipulator::moveCameraByDisplacement(float x,float y, float z)
{
	_eye+=osg::Vec3(x,y,z);
}
osg::Matrixd MyFirstPersonCamManipulator::getRotationMatrix()
{
	osg::Quat FixInitialRot;//used to cancel out the initial rotation of _rotation
	FixInitialRot.makeRotate(-3.1415926535/2,1.0,0.0,0.0);
	return osg::Matrixd::rotate( FixInitialRot*_rotation);
}
