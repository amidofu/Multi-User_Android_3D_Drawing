#include "osgMain.h"

void itoa(int val,std::string& buf, int base)
{
	int i=30;
	buf="";
	for(;val&&i;--i,val/=base)
		buf="0123456789abcdef"[val%base]+buf;
}
osgMain::osgMain()
{
	initialized=false;
    startSendNode=false;
    SendingGeometry=false;
    RecvingGeometry=false;
    startSendMainSceneGeodes=false;
    sendingMainSceneGeodes=false;
    recvingMainSceneGeodes=false;
    DrawLine=false;


    LineStart=true;
    getNewPt=false;

    toggleStartDrawLine=false;
    mshape=new osg::Vec3Array();
    mshape->push_back(osg::Vec3(1.0,0.0,0.0));
    mshape->push_back(osg::Vec3(0.0,1.0,0.0));
    mshape->push_back(osg::Vec3(-1.0,0.0,0.0));
    mshape->push_back(osg::Vec3(0.0,-1.0,0.0));

    line3Pts=new osg::Vec3Array();

    toggleEditing=false;
    editMode=false;
    editHost=false;
    justFinishEdit=false;


    deleteGeode=false;

    lastNewID=0;
    sending=false;

}
void osgMain::draw()
{
	//store self camera's orientation to the storage class
	SelfCamInfo.setCamRot(MFPCM->getRotation());

	//update other player's motion
	UOPC->setPosRot(this->OtherPlayerCamInfo.CamPos,this->OtherPlayerCamInfo.CamRot);

	if(OtherPlayerCamInfo.NewActGeode)//got some events from other player, do corresponding actions in self scene
	{
		int area=OtherPlayerCamInfo.pickedAreaNum;
		int GeodeNum=OtherPlayerCamInfo.pickedActGeodeNum;

		OtherPlayerCamInfo.NewActGeode=false;
	}

	viewer->frame();
	//store self camera's position to the storage class
	this->SelfCamInfo.CamPos=MFPCM->getEye();
}
void osgMain::initOSG(int width, int height)
{

	__android_log_write(ANDROID_LOG_ERROR,"OSGANDROID","in initOSG");


	root=new osg::Group();


	mainSceneGeodes=new osg::Group;
	__android_log_write(ANDROID_LOG_ERROR,"OSGANDROID","create test scene start");
	//TestNumberedGeodeScene(mainSceneGeodes, mNumGeodeMap, mRNumGeodeMap);
	__android_log_write(ANDROID_LOG_ERROR,"OSGANDROID","create test scene ok");

	root->addChild(mainSceneGeodes);

    editTransform=new osg::MatrixTransform;
    editGeode=new osg::Geode;
    editGeometry=new osg::Geometry;
    root->addChild(editTransform);
    editTransform->addChild(editGeode);
    osg::Vec4 white(1.0,1.0,1.0,1.0);
    osg::ref_ptr<osg::Material> mat=new osg::Material;
    mat->setDiffuse(osg::Material::FRONT_AND_BACK,white);
    editGeode->getOrCreateStateSet()->setAttributeAndModes(mat.get());
    editGeode->getOrCreateStateSet()->setMode(GL_CULL_FACE,osg::StateAttribute::OFF);
    editGeode->addDrawable(editGeometry);


	//lights in the scene
	osg::ref_ptr<osg::Light> light=new osg::Light;
    light->setPosition(osg::Vec4(0.0,0.0,10000.0,1.0));
    osg::ref_ptr<osg::LightSource> lightsrc=new osg::LightSource;
    lightsrc->setLight(light);
    root->addChild(lightsrc);
    root->getOrCreateStateSet()->setMode(GL_LIGHT0,osg::StateAttribute::ON);

    RecvNodes=new osg::Group();
    root->addChild(RecvNodes);
	MFPCM=new MyFirstPersonCamManipulator(0.5f);
	//__android_log_write(ANDROID_LOG_ERROR,"OSGANDROID","MFPCM created");
    picker=new PolytopePickHandler();
    //root->addChild(picker->getOrCreateSelectionBox());//visualize the selection box


	mt=new osg::MatrixTransform;
	UOPC=new UpdateOtherPlayerCamCallback();
	mt->setUpdateCallback(UOPC);
	root->addChild(mt);

	//model that represent the other player
	osg::Vec4 yellow;
	yellow.set(1.0,1.0,0.0,1.0);
	mt->addChild(Triangle(yellow,0.0));
	osg::Vec4 red;
	red.set(1.0,0.0,0.0,1.0);
	mt->addChild(Triangle(red,2.0));

    //__android_log_write(ANDROID_LOG_ERROR,"OSGANDROID","Before create viewer");
	viewer = new osgViewer::Viewer();
	viewer->setUpViewerAsEmbeddedInWindow(0, 0, width, height);
	viewer->realize();

    state = root->getOrCreateStateSet();
    state->setMode(GL_LIGHTING, osg::StateAttribute::ON);
    state->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
    state->setMode(GL_CULL_FACE, osg::StateAttribute::ON);

	viewer->setSceneData(root.get());
	viewer->setCameraManipulator(MFPCM);
	viewer->addEventHandler(picker.get());
	viewer->home();
	//__android_log_write(ANDROID_LOG_DEBUG,"OK","OK");


	viewer->getCamera()->setClearColor(osg::Vec4(0.0f,1.0f,0.0f,1.0f));

	viewer->frame();
	//set initial position of self camera
	MFPCM->setEye(osg::Vec3(0.0,0.0,200.0));
    osg::Vec3 eye=MFPCM->getEye();

    osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFile("/sdcard/osg_data/cessna.osg");
        //osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFile("/mnt/sdcard/osg_data/Octahedron.osg");
        //osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFile("/mnt/sdcard/osg_data/sphere.osg");
        //osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFile("/mnt/sdcard/osg_data/venus.osg");


        //osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFile("/mnt/sdcard/osg_data/Giant.osg");


        osg::ref_ptr<osg::MatrixTransform> mmm=new osg::MatrixTransform;
        osg::Matrixd mmat;
         mmat.makeTranslate(0,0,50.0);
         mmm->setMatrix(mmat);
         root->addChild(mmm);
        if(loadedModel==0)
    	    __android_log_write(ANDROID_LOG_ERROR,"OSGANDROID","Model not loaded");
    	else
    		mmm->addChild(loadedModel);

        osg::ref_ptr<osg::Group> axis=new osg::Group;
        axis->addChild(Axis());
        root->addChild(axis);


    initialized=true;
	__android_log_write(ANDROID_LOG_ERROR,"OSGANDROID","finish initOSG");

}

void osgMain::setCameraPosition(float x,float y,float z)
{
    MFPCM->setEye(x,y,z);
}
void osgMain::setCameraOrientation(const osg::Matrixd mat)
{
    MFPCM->setRot(mat);
}
void osgMain::mouseEventMotion(float x,float y)
{
	//send mouse action
	viewer->getEventQueue()->mouseMotion(x,y);
}
void osgMain::mouseEventPress(float x,float y,int button)
{
	viewer->getEventQueue()->mouseButtonPress(x,y,button);
}
void osgMain::mouseEventRelease(float x,float y,int button)
{
	viewer->getEventQueue()->mouseButtonRelease(x,y,button);
}
void osgMain::keyboardEventKeyDown(int key)
{
	viewer->getEventQueue()->keyPress(key);
}
void osgMain::moveCameraByAcceleration(float x,float y,float z,float dT)
{
	//currently not used
}
void osgMain::rotateCameraByRotation(float axisX,float axisY,float axisZ,float angle)
{
	//rotate self camera
	MFPCM->rotateCameraByRotation(axisX,axisY,axisZ,angle);
}

osgMain::~osgMain()
{
}

void osgMain::moveCameraByDisplacement(float x,float y,float z)
{
	//move self camera
	MFPCM->moveCameraByDisplacement(x,y,z);
}
void osgMain::prepareSendCamInfo()
{

	selfC.posx=SelfCamInfo.CamPos.x();
	selfC.posy=SelfCamInfo.CamPos.y();
	selfC.posz=SelfCamInfo.CamPos.z();
	//__android_log_print(ANDROID_LOG_INFO,"jni server","prepare Campos: %f,%f,%f",SelfCamInfo.CamPos.x(),SelfCamInfo.CamPos.y(),SelfCamInfo.CamPos.z());
	//__android_log_print(ANDROID_LOG_INFO,"jni server","prepare selfC: %f,%f,%f",selfC.posx,selfC.posy,selfC.posz);
	selfC.rotx=SelfCamInfo.CamRot.x();
	selfC.roty=SelfCamInfo.CamRot.y();
	selfC.rotz=SelfCamInfo.CamRot.z();
	selfC.rotw=SelfCamInfo.CamRot.w();

	selfC.pickedActGeodeNum=picker->currentActGeode;
	selfC.pickedAreaNum=picker->currentActArea;
	selfC.NewActGeode=picker->NewAction;

	picker->NewAction=false;

    selfC.startSendNode=startSendNode;
    selfC.startSendMainScene=startSendMainSceneGeodes;
    selfC.drawLine=DrawLine;
    selfC.toggleStartDrawLine=toggleStartDrawLine;



	if(getNewPt)
	{
		selfC.newPt=true;
		selfC.newPtx=newPt.x();
		selfC.newPty=newPt.y();
		selfC.newPtz=newPt.z();
	}
	else
		selfC.newPt=false;

	selfC.toggleEditing=toggleEditing;
	selfC.editing=editMode;

	selfC.deleteGeode=deleteGeode;

	selfC.changeColor=changeColor;

}

void osgMain::readCamInfo()
{
		osg::Vec3 eye;
		osg::Vec4 rot;
		osg::Quat quat;

		eye.set(otherC.posx,otherC.posy,otherC.posz);

		rot.set(otherC.rotx,otherC.roty,otherC.rotz,otherC.rotw);
		quat.set(otherC.rotx,otherC.roty,otherC.rotz,otherC.rotw);

		OtherPlayerCamInfo.setCamInfo(eye,rot);
		OtherPlayerCamInfo.setCamRot(quat);

		OtherPlayerCamInfo.setActGeodeInfo(otherC.pickedAreaNum,otherC.pickedActGeodeNum,otherC.NewActGeode);
		recvNode=otherC.startSendNode;


}
void osgMain::prepareSendNode()
{
	std::stringstream stst;
	osg::ref_ptr<osgDB::ReaderWriter> rw=osgDB::Registry::instance()->getReaderWriterForExtension("osg");
	rw->writeNode(*picker->currentSelectedNode.get(),stst);
	unsigned long arrayLength=stst.str().length();
	sendNodeBuffer=new char[arrayLength];
	strcpy(sendNodeBuffer,stst.str().c_str());
	selfC.numBytesofNode=arrayLength;
}
void osgMain::prepareSendNode(osg::Node * node,int & numBytes)
{
	std::stringstream stst;
	osg::ref_ptr<osgDB::ReaderWriter> rw=osgDB::Registry::instance()->getReaderWriterForExtension("osg");
	rw->writeNode(*node,stst);
	unsigned long arrayLength=stst.str().length();
	sendNodeBuffer=new char[arrayLength];
	strcpy(sendNodeBuffer,stst.str().c_str());
	numBytes=arrayLength;
}
void osgMain::readRecvNode(char* readNode)
{
	std::stringstream stst;
	stst<<readNode;
	osg::ref_ptr<osgDB::ReaderWriter> rw=osgDB::Registry::instance()->getReaderWriterForExtension("osg");
	RecvNodes->addChild( rw->readNode(stst).getNode());
}
void osgMain::readRecvNode(char* readNode,osg::Node* & output, std::stringstream & stst,osg::ref_ptr<osgDB::ReaderWriter> & rw)
{
	stst<<readNode;
	rw=osgDB::Registry::instance()->getReaderWriterForExtension("osg");
	output=rw->readNode(stst).getNode();
}
void osgMain::toggleDrawLine(int ID)
{

	DrawLine=!DrawLine;
	__android_log_print(ANDROID_LOG_ERROR,"jni server","toggel draw line, DrawLine:%d",DrawLine);
	if(DrawLine)
	{
		__android_log_print(ANDROID_LOG_ERROR,"jni server","draw line initialize start");
		toggleStartDrawLine=true;
		//osg::ref_ptr<osg::Geode> geode=new osg::Geode;
		//int ID=mNumGeodeMap.size;//??
		currentLineGeode=new osg::Geode;
		mapIDAndGeode(ID,currentLineGeode,mNumGeodeMap,mRNumGeodeMap);
		mainSceneGeodes->addChild(currentLineGeode);
		LineStart=true;
		__android_log_print(ANDROID_LOG_ERROR,"jni server","draw line initialized");
	}
	else
	{
		toggleStartDrawLine=true;
		//currentLineGeode=new osg::Geode;
		line3Pts->clear();
		getNewPt=false;
		LineStart=true;
	}
}
void osgMain::toggleDrawLine(int ID,osg::Vec4 color)
{
	DrawLine=!DrawLine;
	if(DrawLine)
	{
		toggleStartDrawLine=true;
		currentLineGeode=new osg::Geode;


	    osg::ref_ptr<osg::Material> mat=new osg::Material;
	    mat->setDiffuse(osg::Material::FRONT_AND_BACK,color);
	    currentLineGeode->getOrCreateStateSet()->setAttributeAndModes(mat.get());
	    currentLineGeode->getOrCreateStateSet()->setMode(GL_CULL_FACE,osg::StateAttribute::OFF);
		//int ID=mNumGeodeMap.size;//??
		mapIDAndGeode(ID,currentLineGeode,mNumGeodeMap,mRNumGeodeMap);
		mainSceneGeodes->addChild(currentLineGeode);

		LineStart=true;
	}
	else
	{
		toggleStartDrawLine=true;
		//currentLineGeode=new osg::Geode;
		line3Pts->clear();
		getNewPt=false;
		LineStart=true;
	}
}
void osgMain::drawLine(osg::Vec3 NewPt)
{
	__android_log_print(ANDROID_LOG_ERROR,"jni server","in new line point");
	if(!DrawLine)
		return;
	else
		__android_log_print(ANDROID_LOG_ERROR,"jni server","in new line point");
	newPt=NewPt;
    getNewPt=true;
    __android_log_print(ANDROID_LOG_ERROR,"jni server","create pipe");
    __android_log_print(ANDROID_LOG_ERROR,"jni server","newPt: x:%f, y:%f, z:%f",newPt.x(),newPt.y(),newPt.z());
	if(line3Pts->size()==3)
	{
		osg::Vec3 pt1=line3Pts->at(0);
		osg::Vec3 pt2=line3Pts->at(1);
		osg::Vec3 pt3=line3Pts->at(2);
		createPipe(pt1,pt2,pt3,mshape,LineStart,currentLineGeode);
		line3Pts->at(0)=line3Pts->at(1);
		line3Pts->at(1)=line3Pts->at(2);
		line3Pts->at(2)=newPt;
		if(LineStart)
			LineStart=false;
	}
	else
		line3Pts->push_back(newPt);
	__android_log_print(ANDROID_LOG_ERROR,"jni server","get new line point");
}
void osgMain::createPipe(osg::Vec3 & pt1, osg::Vec3 & pt2, osg::Vec3 & pt3, osg::Vec3Array* shape,bool startPt,osg::ref_ptr<osg::Geode>  & currentLine)
{
	__android_log_print(ANDROID_LOG_ERROR,"jni server","create pipe");
    //shape must be at xy plane
    if(shape->size()<3)
    {
    	//__android_log_print(ANDROID_LOG_ERROR,"jni server","create pipe NOOOO");
    	return ;
    }

    //currentLine->addDrawable(new osg::ShapeDrawable(new osg::Sphere(pt3,1.0f)));
    //return;
    osg::Vec3 v12=(pt1-pt2);
    //__android_log_print(ANDROID_LOG_ERROR,"jni server","create pipe OK0-1");
    v12.normalize();
    osg::Vec3 v32=(pt3-pt2);
    //__android_log_print(ANDROID_LOG_ERROR,"jni server","create pipe OK0-2");
    v32.normalize();
    osg::ref_ptr<osg::Vec3Array> startShape;
    osg::Vec3 center(0,0,0);
    //__android_log_print(ANDROID_LOG_ERROR,"jni server","create pipe OK0-3");
    //__android_log_print(ANDROID_LOG_ERROR,"jni server","shape: x:%f, y%f, z%f",shape->at(0).x(),shape->at(0).y(),shape->at(0).z());
    for(int i=0;i<shape->size();i++)
        center=center+shape->at(i);
    //__android_log_print(ANDROID_LOG_ERROR,"jni server","create pipe OK0-4");
    center=center/(float)shape->size();
    //__android_log_print(ANDROID_LOG_ERROR,"jni server","create pipe OK1");
    if(startPt)
    {
    	// __android_log_print(ANDROID_LOG_ERROR,"jni server","create pipe OK2");
        startShape=new osg::Vec3Array;

        osg::Vec3 posZ(0.0,0.0,1.0);
        osg::Vec3 rotAxis=posZ^(-v12);
        if(rotAxis.length2()<0.01f)
            return ;
        float rotAngle=(float)acos((double)(posZ*(-v12)));
        osg::Matrixd mat;
        mat.makeRotate(rotAngle,rotAxis);
        for(int i=0;i<shape->size();i++)
            startShape->push_back(mat.preMult(shape->at(i)-center)+pt1);
        //__android_log_print(ANDROID_LOG_ERROR,"jni server","create pipe OK3");
    }
    else
        startShape=shape;

    osg::Vec3 p2Dir;
    osg::Vec3 half=(v12+v32)/2.0f;
    half.normalize();
    osg::Vec3 p2DirOrtho=v12^half;
    p2DirOrtho.normalize();
    p2Dir=p2DirOrtho^half;
    p2Dir.normalize();

    osg::Vec3 shapeDir=(startShape->at(2)-startShape->at(1))^(startShape->at(0)-startShape->at(1));
    shapeDir.normalize();
    if(shapeDir*(-v12)<0.0)
        shapeDir=-shapeDir;
    //__android_log_print(ANDROID_LOG_ERROR,"jni server","create pipe OK3");
    osg::ref_ptr<osg::Vec3Array> p2Shape=new osg::Vec3Array;
    osg::Vec3 rotAxis=shapeDir^p2Dir;
    float rotAngle=(float)acos((double)(shapeDir*p2Dir));
    osg::Matrixd mat;
    mat.makeRotate(rotAngle,rotAxis);
    osg::Vec3 trans=pt2-pt1;
    center.set(0,0,0);
    for(int i=0;i<startShape->size();i++)
        center=center+startShape->at(i);
    center=center/(float)startShape->size();
    for(int i=0;i<startShape->size();i++)
        p2Shape->push_back(mat.preMult(startShape->at(i)-center)+center+trans);

    //__android_log_print(ANDROID_LOG_ERROR,"jni server","create pipe OK4");
    osg::ref_ptr<osg::Geometry> pipe=new osg::Geometry;

    osg::ref_ptr<osg::Vec3Array> allVerts=new osg::Vec3Array;
    int numVerts=p2Shape->size();
    for(unsigned int i=0;i<numVerts;i++)
    {
        allVerts->push_back(startShape->at(i));
        allVerts->push_back(startShape->at((i+1)%numVerts));
        allVerts->push_back(p2Shape->at(i));
        allVerts->push_back(startShape->at((i+1)%numVerts));
        allVerts->push_back(p2Shape->at((i+1)%numVerts));
        allVerts->push_back(p2Shape->at(i));
    }
    //__android_log_print(ANDROID_LOG_ERROR,"jni server","create pipe OK5");
    pipe->setVertexArray(allVerts.get());

    osg::ref_ptr<osg::DrawArrays> array=new osg::DrawArrays(GL_TRIANGLES,0,allVerts->size());
    pipe->addPrimitiveSet(array);
    osgUtil::SmoothingVisitor::smooth(*pipe);

    //__android_log_print(ANDROID_LOG_ERROR,"jni server","create pipe OK6");
    if(startPt)
    {
        //currentLine=new oseditGeodeg::Geode();
        currentLine->addDrawable(pipe);
    }
    else
    {
         osgUtil::Optimizer::MergeGeometryVisitor::mergeGeometry(*currentLine->getDrawable(0)->asGeometry(),*pipe);
    }
    for(int i=0;i<numVerts;i++)
            shape->at(i)=p2Shape->at(i);
       // __android_log_print(ANDROID_LOG_ERROR,"jni server","shape2: x:%f, y:%f, z:%f, numVerts:%d",shape->at(0).x(),shape->at(0).y(),shape->at(0).z(),numVerts);
    //return pipe.release();
    __android_log_print(ANDROID_LOG_ERROR,"jni client","create pipe finish");

}
void osgMain::updateEditTransform()
{
    osg::Matrixd mat;
    mat.identity();

    mat.postMult(osg::Matrixd::translate(-editCenter));
    mat.postMult(editScale);
    mat.postMult(editRotation);
    mat.postMult(editTranslation);
    mat.postMult(osg::Matrixd::translate(editCenter));

    editTransform->setMatrix(mat);
    osg::Matrixd mat2=editTransform->getMatrix();
    //__android_log_print(ANDROID_LOG_ERROR,"jni server","edit mat %f, %f, %f, %f",mat2(0,0),mat2(0,1),mat2(0,2),mat2(0,3));
    //__android_log_print(ANDROID_LOG_ERROR,"jni server","edit mat %f, %f, %f, %f",mat2(1,0),mat2(1,1),mat2(1,2),mat2(1,3));
    //__android_log_print(ANDROID_LOG_ERROR,"jni server","edit mat %f, %f, %f, %f",mat2(2,0),mat2(2,1),mat2(2,2),mat2(2,3));
    //__android_log_print(ANDROID_LOG_ERROR,"jni server","edit mat %f, %f, %f, %f",mat2(3,0),mat2(3,1),mat2(3,2),mat2(3,3));
}
void osgMain::finishEditing()
{
    osg::ref_ptr<osg::Geometry> temp=editing->asGeometry();

    temp->setUseDisplayList(false);
    temp->dirtyBound();

    osg::Vec3Array* verts=dynamic_cast<osg::Vec3Array*>(temp->getVertexArray());
    unsigned int length=verts->size();
    osg::Matrixd mat=editTransform->getMatrix();
    for(unsigned int i=0;i<length;i++)
    {
        verts->at(i)=mat.preMult(verts->at(i));
    }
    temp->setUseDisplayList(true);
	editRotation.identity();
	editTranslation.identity();
	editScale.identity();
    editTransform->setMatrix(osg::Matrixd::identity());
    swapDrawbles(editing,editGeometry);
}
void osgMain::swapDrawbles(osg::Drawable* drawable1, osg::Drawable* drawable2)
{
    osg::Drawable* temp;
    temp=drawable1;
    osg::Geode* geode2=drawable2->getParent(0)->asGeode();
    drawable1->getParent(0)->asGeode()->replaceDrawable(drawable1,drawable2);
    geode2->replaceDrawable(drawable2,temp);
}
void osgMain::ToggleEditing()
{
	editMode=!editMode;
	toggleEditing=true;

	if(editMode)
	{
		editHost=true;
		editing=picker->currentSelectedDrawable;
		osg::Geode* geode=editing->getParent(0)->asGeode();
		editingID=mRNumGeodeMap.find(geode)->second;
		osg::BoundingBox bb=editing->getBound();

		editTransform->setMatrix(osg::Matrixd::identity());
		swapDrawbles(editing,editGeometry);
		editCenter=picker->currentSelectedDrawableWroldCenter;

		editRotation.set(osg::Matrixd::identity());
		editTranslation.set(osg::Matrixd::identity());
		editScale.set(osg::Matrixd::identity());
		updateEditTransform();
	}
	else
	{
		justFinishEdit=true;
		finishEditing();
	}


}
void osgMain::ToggleEditing(int ID)
{
	editMode=!editMode;
	///toggleEditing=true;?
	if(editMode)
	{
		editingID=ID;
		osg::Geode* geode=mNumGeodeMap.find(ID)->second;
		editing=geode->getDrawable(0);

		swapDrawbles(editing,editGeometry);
		//editTransform->setMatrix(transMat);
	}
	else
	{
		editHost=false;
		finishEditing();
	}
}
void osgMain::edit(int type,int axis,float amount)
{
	if(!editMode)
		return;
	switch(type)
	{
	case Rotation:
		switch(axis)
		{
		case TaxisX:
			editRotation.preMult(osg::Matrixd::rotate(amount,1.0,0.0,0.0));
			updateEditTransform();
			break;
		case TaxisY:
			editRotation.preMult(osg::Matrixd::rotate(amount,0.0,1.0,0.0));
			updateEditTransform();
			break;
		case TaxisZ:
			editRotation.preMult(osg::Matrixd::rotate(amount,0.0,0.0,1.0));
			updateEditTransform();
			break;
		}
		break;
	case Translation:
		switch(axis)
		{
		case TaxisX:
			editTranslation.preMult(osg::Matrixd::translate((double)amount,0.0,0.0));
			updateEditTransform();
			break;
		case TaxisY:
			editTranslation.preMult(osg::Matrixd::translate(0.0,(double)amount,0.0));
			updateEditTransform();
			break;
		case TaxisZ:
			editTranslation.preMult(osg::Matrixd::translate(0.0,0.0,(double)amount));
			updateEditTransform();
			break;
		}
		break;
	case Scale:
		float nAmount;
		if(amount>0.0)
			nAmount=1.0+amount;
		else if(amount<0.0)
			nAmount=1.0/(1.0-amount);
		else
			break;
		switch(axis)
		{
		case TaxisX:
			editScale.preMult(osg::Matrixd::scale((double)nAmount,1.0,1.0));
			updateEditTransform();
			break;
		case TaxisY:
			editScale.preMult(osg::Matrixd::scale(1.0,(double)nAmount,1.0));
			updateEditTransform();
			break;
		case TaxisZ:
			editScale.preMult(osg::Matrixd::scale(1.0,1.0,(double)nAmount));
			updateEditTransform();
			break;
		}
		break;
	}
}
void osgMain::storeMatrixIntoBuf(char* & buf, osg::Matrixd & mat)
{
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            float a=mat(i,j);
            memcpy(&buf[4*(i*4+j)],&a,4);
        }
    }
}
void osgMain::loadMatrixFromBuf(char  buf[4*4*4], osg::Matrixd & mat)
{
    float a;
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            memcpy(&a,&buf[4*(i*4+j)],4);
            mat(i,j)=a;
        }
    }
}

bool osgMain::getCurrentSelectGeodeID(int & ID)
{

	osg::Geode* geode=picker->currentSelectedNode->asGeode();
	if(geode!=NULL)
	{
		ID=mRNumGeodeMap.find(geode)->second;
		return true;
	}
	else
		return false;
}

void osgMain::deleteCurrentSelectedGeometry()
{
	int ID;
	ReverseIDGeodeMap::iterator it;
	osg::Geode* geodeToDelete=picker->currentSelectedNode->asGeode();
	if(geodeToDelete!=NULL)
		it=mRNumGeodeMap.find(geodeToDelete);
	else
	{
		deleteGeode=false;
		return;
	}
	if(it!=mRNumGeodeMap.end())
			ID=it->second;
	else
	{
		deleteGeode=false;
		__android_log_print(ANDROID_LOG_ERROR,"jni server","no such geode");
		return;
	}

	mainSceneGeodes->removeChild(geodeToDelete);
	unMapIDAndGeode(ID,geodeToDelete,mNumGeodeMap,mRNumGeodeMap);
}

void osgMain::deleteMainSceneGeode(int ID)
{
	IDGeodeMap::iterator it;
	it=mNumGeodeMap.find(ID);
	osg::Geode* geodeToDelete;
	if(it!=mNumGeodeMap.end())
		geodeToDelete=it->second;
	else
	{
		deleteGeode=false;
		return;
	}
	mainSceneGeodes->removeChild(geodeToDelete);
	unMapIDAndGeode(ID,geodeToDelete,mNumGeodeMap,mRNumGeodeMap);
}

int osgMain::findNewID()
{
	int temp=mNumGeodeMap.size();
	if(lastNewID>mNumGeodeMap.size())
		temp=lastNewID;
	else
		temp=mNumGeodeMap.size();
	IDGeodeMap::iterator it;

	it=mNumGeodeMap.find(temp);
	while(it!=mNumGeodeMap.end())
	{
		temp++;
		it=mNumGeodeMap.find(temp);
	}
	lastNewID=temp;
	return lastNewID;

}

void osgMain::addBox(int & ID,osg::Vec3 & pos)
{
	osg::ref_ptr<osg::Geode> geode=new osg::Geode;
	osg::ref_ptr<BoxGeometry> box=new BoxGeometry();
	float boxSize=15.0;
	pos=MFPCM->getEye();
	geode->addDrawable(box->getBoxGeometry(boxSize,boxSize,boxSize,pos,osg::Vec4(1.0,1.0,1.0,1.0)));

	ID=findNewID();
	mapIDAndGeode(ID,geode.get(),mNumGeodeMap, mRNumGeodeMap);
	mainSceneGeodes->addChild(geode);
}
void osgMain::addBox(osg::Vec3 pos,int ID)
{
	osg::ref_ptr<osg::Geode> geode=new osg::Geode;
	osg::ref_ptr<BoxGeometry> box=new BoxGeometry();
	float boxSize=15.0;
	geode->addDrawable(box->getBoxGeometry(boxSize,boxSize,boxSize,pos,osg::Vec4(1.0,1.0,1.0,1.0)));
	mapIDAndGeode(ID,geode.get(),mNumGeodeMap, mRNumGeodeMap);
	mainSceneGeodes->addChild(geode);
}
void osgMain::addGeode(osg::Geode* geode,int ID)
{
	mapIDAndGeode(ID,geode,mNumGeodeMap, mRNumGeodeMap);
	mainSceneGeodes->addChild(geode);
}

void osgMain::addBox(int & ID,osg::Vec3 & pos,osg::Vec4 color)
{
	osg::ref_ptr<osg::Geode> geode=new osg::Geode;
	osg::ref_ptr<BoxGeometry> box=new BoxGeometry();
	float boxSize=15.0;
	pos=MFPCM->getEye();
    osg::ref_ptr<osg::Material> mat=new osg::Material;
    mat->setDiffuse(osg::Material::FRONT_AND_BACK,color);
    __android_log_print(ANDROID_LOG_ERROR,"jni server","add box r: %f, g: %f, b: %f",color.x(),color.y(),color.z());
    geode->getOrCreateStateSet()->setAttributeAndModes(mat.get());
    geode->getOrCreateStateSet()->setMode(GL_CULL_FACE,osg::StateAttribute::OFF);
	geode->addDrawable(box->getBoxGeometry(boxSize,boxSize,boxSize,pos));

	ID=findNewID();
	mapIDAndGeode(ID,geode.get(),mNumGeodeMap, mRNumGeodeMap);
	mainSceneGeodes->addChild(geode);
}
void osgMain::addBox(osg::Vec3 pos,int ID,osg::Vec4 color)
{
	osg::ref_ptr<osg::Geode> geode=new osg::Geode;
	osg::ref_ptr<BoxGeometry> box=new BoxGeometry();
	float boxSize=15.0;
    osg::ref_ptr<osg::Material> mat=new osg::Material;
    mat->setDiffuse(osg::Material::FRONT_AND_BACK,color);
    geode->getOrCreateStateSet()->setAttributeAndModes(mat.get());
    geode->getOrCreateStateSet()->setMode(GL_CULL_FACE,osg::StateAttribute::OFF);
	geode->addDrawable(box->getBoxGeometry(boxSize,boxSize,boxSize,pos));
	mapIDAndGeode(ID,geode.get(),mNumGeodeMap, mRNumGeodeMap);
	mainSceneGeodes->addChild(geode);
}
void osgMain::changeColorForCurrentSelectGeode()
{
	int ID;
	ReverseIDGeodeMap::iterator it;
	osg::Geode* geodeToChange=picker->currentSelectedNode->asGeode();
	if(geodeToChange!=NULL)
		it=mRNumGeodeMap.find(geodeToChange);
	else
	{
		changeColor=false;
		return;
	}
	if(it!=mRNumGeodeMap.end())
			ID=it->second;
	else
	{
		changeColor=false;
		__android_log_print(ANDROID_LOG_ERROR,"jni client chagne color","no such geode");
		return;
	}

    osg::ref_ptr<osg::Material> mat=new osg::Material;
    mat->setDiffuse(osg::Material::FRONT_AND_BACK,currentColor);
    geodeToChange->getOrCreateStateSet()->setAttributeAndModes(mat);
}

void osgMain::changeColorForOneGeode(int ID,osg::Vec4 color)
{
	IDGeodeMap::iterator it;
	it=mNumGeodeMap.find(ID);
	osg::Geode* geodeToChangeColor;
	if(it!=mNumGeodeMap.end())
		geodeToChangeColor=it->second;
	else
	{
		changeColor=false;
		return;
	}
    osg::ref_ptr<osg::Material> mat=new osg::Material;
    mat->setDiffuse(osg::Material::FRONT_AND_BACK,color);
    geodeToChangeColor->getOrCreateStateSet()->setAttributeAndModes(mat);
}
bool osgMain:: NodeSelected()
{
	return picker->NodeSelected();
}
