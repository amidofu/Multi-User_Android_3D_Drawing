#include <iostream>
#include <osg/GL>
#include <osg/GLExtensions>
#include <osg/Depth>
#include <osg/Program>
#include <osg/Shader>
#include <osg/Node>
#include <osg/Notify>

//osgDB
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

//osg_viewer
#include <osgViewer/Viewer>
#include <osgViewer/Renderer>
#include <osgViewer/ViewerEventHandlers>

//osgGA
#include <osgGA/GUIEventAdapter>
#include <osgUtil/Optimizer>
#include <osgUtil/SmoothingVisitor>


#include "CamInfo.h"
#include "MyFirstPersonCamManipulator.h"
#include "PolytopePickHandler.h"
#include "AuxFunc.h"
#include "scenes.h"
#include "UpdateOtherPlayerCamCallback.h"
#include "AuxFile.h"

#include <android/log.h>
#include <map>
#include <math.h>
#include <stdbool.h>
#include <string>


//Static plugins Macro
USE_OSGPLUGIN(ive)
USE_OSGPLUGIN(osg)
USE_OSGPLUGIN(osg2)
//USE_OSGPLUGIN(terrain)
USE_OSGPLUGIN(rgb)
USE_OSGPLUGIN(OpenFlight)
USE_OSGPLUGIN(dds)

//Static DOTOSG
USE_DOTOSGWRAPPER_LIBRARY(osg)
USE_DOTOSGWRAPPER_LIBRARY(osgFX)
USE_DOTOSGWRAPPER_LIBRARY(osgViewer)
USE_DOTOSGWRAPPER_LIBRARY(osgVolume)

//Static serializer
USE_SERIALIZER_WRAPPER_LIBRARY(osg)
USE_SERIALIZER_WRAPPER_LIBRARY(osgFX)
USE_SERIALIZER_WRAPPER_LIBRARY(osgVolume)

#define Rotation 0
#define Translation 1
#define Scale 2
#define TaxisX 0
#define TaxisY 1
#define TaxisZ 2
class osgMain{
public:

	//main osg viewer
    osg::ref_ptr<osgViewer::Viewer> viewer;
    //main osg root
    osg::ref_ptr<osg::Group> root;
    //currently not used
    osg::ref_ptr<osg::StateSet> state;
    //used for showing the position and orientation of the other player
    osg::ref_ptr<osg::MatrixTransform> mt;
    //customized first person camera manipulator, control how camera moves or rotates
    osg::ref_ptr<MyFirstPersonCamManipulator> MFPCM;

    //used to select an object in the scene by clicking the object
    osg::ref_ptr<PolytopePickHandler> picker;
    //node callback for updating the motion of another player
    osg::ref_ptr<UpdateOtherPlayerCamCallback> UOPC;


    //indicate if OSG stuffs are initialized
    bool initialized;

    //record self camera position
    osg::Vec3 FPeye;
    //record self camera moving direction
    osg::Vec3 moveDir;


    //main draw function, called by every frame
    void draw();
    //intialize the scene and other stuff
    void initOSG(int width,int height);
    //action when got mouse movement events
    void mouseEventMotion(float x,float y);
    //action when got mouse button press events
    void mouseEventPress(float x, float y,int button);
    //action when got mouse button release events
    void mouseEventRelease(float x, float y,int button);
    //action when got key down from key board
    void keyboardEventKeyDown(int key);
    //move self camera by the acceleration which is record from the accelerometer, currently not used
    void moveCameraByAcceleration(float x,float y,float z,float dT);
    //rotate self camera by the data from gyroscope
    void rotateCameraByRotation(float axisX,float axisY,float axisZ,float angle);
    //read other player's camera position, orientation and other events
    void readOtherPlayerCamInfo(CamInfo cam);
    //send self camera position, orientation and other events
    void sendPlayerCamInfo();

    void setCameraPosition(float x,float y,float z);
    void setCameraOrientation(const osg::Matrixd mat);
    //store self camera position, orientation and other events
    CamInfo SelfCamInfo;
    //store other camera position, orientation and other events
    CamInfo OtherPlayerCamInfo;

    ScamInfo selfC;
    ScamInfo otherC;
    //store info about local camera and get ready to send
    void prepareSendCamInfo();
    //read other user's pose info
    void readCamInfo();


    bool startSendNode;
    bool recvNode;
    //serialize a node to send via wifi
    void prepareSendNode();
    void prepareSendNode(osg::Node * node,int & numBytes);
    char* sendNodeBuffer;
    //read node from wifi
    void readRecvNode(char* readNode);
    void readRecvNode(char* readNode,osg::Node* & output, std::stringstream & stst,osg::ref_ptr<osgDB::ReaderWriter> & rw);
    osg::ref_ptr<osg::Group> RecvNodes;
    bool SendingGeometry;
    bool RecvingGeometry;

    osg::ref_ptr<osg::Group> mainSceneGeodes;
    bool startSendMainSceneGeodes;
    bool sendingMainSceneGeodes;
    bool recvingMainSceneGeodes;

    IDGeodeMap mNumGeodeMap;
    ReverseIDGeodeMap mRNumGeodeMap;

    //members for drawline
    void toggleDrawLine(int ID);
    void toggleDrawLine(int ID,osg::Vec4 color);
    bool toggleStartDrawLine;
    bool DrawLine;
    bool LineStart;
    void drawLine(osg::Vec3 NewPt);
    osg::Vec3 newPt;
    bool getNewPt;
    osg::ref_ptr<osg::Geode> currentLineGeode;
    osg::ref_ptr<osg::Vec3Array>  mshape;
    osg::ref_ptr<osg::Vec3Array> line3Pts;
    void createPipe(osg::Vec3 & pt1, osg::Vec3 & pt2, osg::Vec3 & pt3, osg::Vec3Array* shape,bool startPt,osg::ref_ptr<osg::Geode>  & currentLine);


    //members for editing geometry
    osg::ref_ptr<osg::MatrixTransform> editTransform;
    osg::ref_ptr<osg::Geode> editGeode;
    osg::ref_ptr<osg::Geometry> editGeometry;
    osg::Vec3 editCenter;
    osg::Matrixd editRotation;
    osg::Matrixd editTranslation;
    osg::Matrixd editScale;
    float scaleX,scaleY,scaleZ;
    bool toggleEditing;
    bool editMode;
    bool editHost;
    void updateEditTransform();
    void finishEditing();
    osg::ref_ptr<osg::Drawable> editing;
    void swapDrawbles(osg::Drawable* drawable1, osg::Drawable* drawable2);
    void ToggleEditing();
    void ToggleEditing(int ID);
    int editingID;
    void edit(int type,int axis,float amount);
    void storeMatrixIntoBuf(char* & buf, osg::Matrixd mat);
    void loadMatrixFromBuf(char  buf[], osg::Matrixd & mat);
    bool justFinishEdit;

    //members for adding box
    void addBox(int & ID,osg::Vec3 & pos);
    void addBox(osg::Vec3 pos,int ID);
    void addGeode(osg::Geode* geode,int ID);
    void addBox(int & ID,osg::Vec3 & pos,osg::Vec4 color);
    void addBox(osg::Vec3 pos,int ID,osg::Vec4 color);

    //delete geode in the scene
    bool getCurrentSelectGeodeID(int & ID);
    void deleteCurrentSelectedGeometry();
    void deleteMainSceneGeode(int ID);
    bool deleteGeode;

    //scene graph control
    int lastNewID;
    int findNewID();
    osgMain();
    ~osgMain();

    //change color of geometry
    osg::Vec4 currentColor;
    bool changeColor;
    void changeColorForCurrentSelectGeode();
    void changeColorForOneGeode(int ID,osg::Vec4 color);

    bool NodeSelected();

    bool sending;
};

void itoa(int val,std::string& buf,int base);
//extern osgMain* osgmain;
