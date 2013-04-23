//customized first person view manipulator
#include <osgGA/FirstPersonManipulator>
#include <android/log.h>
class MyFirstPersonCamManipulator : public osgGA::FirstPersonManipulator
{
public:
	//scalar for camera moving speed, currently not used
	float MoveSpeed;
	//scalar for camera rotation angle, currently not used
	float RotationAngleScalar;
    //the last pressed key
	int currentKey;
    //flag to indicate if the camera is allowed to move;
    bool move;
	MyFirstPersonCamManipulator(float MoveSpeed);
	MyFirstPersonCamManipulator();

	//move camera position by camera acceleration, currently not used
	void moveCameraByAcceleration(float x, float y, float z);
	//rotate camera orientation by gyroscope
	void rotateCameraByRotation(float axisX, float axisY, float axisZ, float angle);
	//move camera position by camera translation, currently not used
	void moveCameraByDisplacement(float x,float y, float z);
	virtual void home(double d);
	virtual bool handleKeyDown(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &us);
	//get camera's position
	osg::Vec3 getEye();
	//get camera's orientation
	osg::Quat getRotation();

    void moveEye(osg::Vec3 movement);
    //print camera's position, can't be used in android
    void printEye();
    //set camera's position
    void setEye(osg::Vec3 newEye);
    void setEye(float x,float y,float z);
    void setRot(const osg::Matrixd mat);
    //get camera matrix
    osg::Matrixd getMatrix() const;
    //moving direction of camera
    mutable osg::Vec3 MoveDir;
    //camera postion of previous frame
    mutable osg::Vec3 OldEye;
    //get inverse camera matrix (as model view matrix)
    osg::Matrix getInverseMatrix()const;
    //print moving direction, can't be used in android
    void printMoveDir();
    //get the camera orientation as rotation matrix
    osg::Matrixd getRotationMatrix();
};
