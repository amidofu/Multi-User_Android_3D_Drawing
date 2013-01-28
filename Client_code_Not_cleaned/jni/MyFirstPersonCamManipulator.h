#include <osgGA/FirstPersonManipulator>
#include <android/log.h>
class MyFirstPersonCamManipulator : public osgGA::FirstPersonManipulator
{
public:
	float MoveSpeed;
	float RotationAngleScalar;
    int currentKey;
    bool move;
	MyFirstPersonCamManipulator(float MoveSpeed);
	MyFirstPersonCamManipulator();

	void moveCameraByAcceleration(float x, float y, float z);
	void rotateCameraByRotation(float axisX, float axisY, float axisZ, float angle);
	void moveCameraByDisplacement(float x,float y, float z);
	virtual void home(double d);
	virtual bool handleKeyDown(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &us);
	osg::Vec3 getEye();
	osg::Quat getRotation();
    void moveEye(osg::Vec3 movement);
    void printEye();
    void setEye(osg::Vec3 newEye);
    void setEye(float x,float y,float z);
    void setRot(const osg::Matrixd mat);

    osg::Matrixd getMatrix() const;
    mutable osg::Vec3 MoveDir;
    mutable osg::Vec3 OldEye;
    osg::Matrix getInverseMatrix()const;
    void printMoveDir();




    osg::Matrixd getRotationMatrix();
};
