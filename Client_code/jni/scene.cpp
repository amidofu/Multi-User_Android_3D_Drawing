#include "scenes.h"
BoxGeometry::BoxGeometry()
{
    length=0.0;
    width=0.0;
    height=0.0;

}

osg::Geometry* BoxGeometry::getBoxGeometry(float length, float width, float height, osg::Vec4 color)
{
    this->createBox(length,width,height,color);
    return this;
}

void BoxGeometry::createBox(float length, float width, float height, osg::Vec4 color)
{
    osg::ref_ptr<osg::Vec3Array> v=new osg::Vec3Array;

    v->push_back(osg::Vec3(-length/2,-width/2,-height/2));
    v->push_back(osg::Vec3(length/2,-width/2,-height/2));
    v->push_back(osg::Vec3(length/2,-width/2,height/2));
    v->push_back(osg::Vec3(-length/2,-width/2,height/2));

    v->push_back(osg::Vec3(-length/2,width/2,height/2));
    v->push_back(osg::Vec3(length/2,width/2,height/2));
    v->push_back(osg::Vec3(length/2,width/2,-height/2));
    v->push_back(osg::Vec3(-length/2,width/2,-height/2));


    v->push_back(osg::Vec3(-length/2,width/2,-height/2));
    v->push_back(osg::Vec3(-length/2,-width/2,-height/2));
    v->push_back(osg::Vec3(-length/2,-width/2,height/2));
    v->push_back(osg::Vec3(-length/2,width/2,height/2));

    v->push_back(osg::Vec3(length/2,width/2,height/2));
    v->push_back(osg::Vec3(length/2,-width/2,height/2));
    v->push_back(osg::Vec3(length/2,-width/2,-height/2));
    v->push_back(osg::Vec3(length/2,width/2,-height/2));


    v->push_back(osg::Vec3(-length/2,-width/2,-height/2));
    v->push_back(osg::Vec3(length/2,-width/2,-height/2));
    v->push_back(osg::Vec3(length/2,width/2,-height/2));
    v->push_back(osg::Vec3(-length/2,width/2,-height/2));

    v->push_back(osg::Vec3(-length/2,width/2,height/2));
    v->push_back(osg::Vec3(length/2,width/2,height/2));
    v->push_back(osg::Vec3(length/2,-width/2,height/2));
    v->push_back(osg::Vec3(-length/2,-width/2,height/2));

    this->setVertexArray(v);
    osg::ref_ptr<osg::DrawArrays> array=new osg::DrawArrays(GL_QUADS,0,24);
    this->addPrimitiveSet(array);
    osg::ref_ptr<osg::Material> mat=new osg::Material;
    mat->setDiffuse(osg::Material::FRONT_AND_BACK,color);

    this->getOrCreateStateSet()->setAttributeAndModes(mat.get());
    this->getOrCreateStateSet()->setAttributeAndModes(mat.get());
    this->getOrCreateStateSet()->setMode(GL_CULL_FACE,osg::StateAttribute::OFF);
}
osg::Geometry* BoxGeometry::getBoxGeometry(float length,float width,float height,osg::Vec3 translation,osg::Vec4 color)
{
	osg::ref_ptr<osg::Vec3Array> v=new osg::Vec3Array;

	    v->push_back(osg::Vec3(-length/2,-width/2,-height/2)+translation);
	    v->push_back(osg::Vec3(length/2,-width/2,-height/2)+translation);
	    v->push_back(osg::Vec3(length/2,-width/2,height/2)+translation);
	    v->push_back(osg::Vec3(-length/2,-width/2,height/2)+translation);

	    v->push_back(osg::Vec3(-length/2,width/2,height/2)+translation);
	    v->push_back(osg::Vec3(length/2,width/2,height/2)+translation);
	    v->push_back(osg::Vec3(length/2,width/2,-height/2)+translation);
	    v->push_back(osg::Vec3(-length/2,width/2,-height/2)+translation);


	    v->push_back(osg::Vec3(-length/2,width/2,-height/2)+translation);
	    v->push_back(osg::Vec3(-length/2,-width/2,-height/2)+translation);
	    v->push_back(osg::Vec3(-length/2,-width/2,height/2)+translation);
	    v->push_back(osg::Vec3(-length/2,width/2,height/2)+translation);

	    v->push_back(osg::Vec3(length/2,width/2,height/2)+translation);
	    v->push_back(osg::Vec3(length/2,-width/2,height/2)+translation);
	    v->push_back(osg::Vec3(length/2,-width/2,-height/2)+translation);
	    v->push_back(osg::Vec3(length/2,width/2,-height/2)+translation);


	    v->push_back(osg::Vec3(-length/2,-width/2,-height/2)+translation);
	    v->push_back(osg::Vec3(length/2,-width/2,-height/2)+translation);
	    v->push_back(osg::Vec3(length/2,width/2,-height/2)+translation);
	    v->push_back(osg::Vec3(-length/2,width/2,-height/2)+translation);

	    v->push_back(osg::Vec3(-length/2,width/2,height/2)+translation);
	    v->push_back(osg::Vec3(length/2,width/2,height/2)+translation);
	    v->push_back(osg::Vec3(length/2,-width/2,height/2)+translation);
	    v->push_back(osg::Vec3(-length/2,-width/2,height/2)+translation);

	    this->setVertexArray(v);
	    osg::ref_ptr<osg::DrawArrays> array=new osg::DrawArrays(GL_QUADS,0,24);
	    this->addPrimitiveSet(array);
	    osg::ref_ptr<osg::Material> mat=new osg::Material;
	    mat->setDiffuse(osg::Material::FRONT_AND_BACK,color);

	    this->getOrCreateStateSet()->setAttributeAndModes(mat.get());
	    this->getOrCreateStateSet()->setAttributeAndModes(mat.get());
	    this->getOrCreateStateSet()->setMode(GL_CULL_FACE,osg::StateAttribute::OFF);
	    return this;
}

osg::Geometry* BoxGeometry::getBoxGeometry(float length,float width,float height,osg::Vec3 translation)
{
	osg::ref_ptr<osg::Vec3Array> v=new osg::Vec3Array;

	    v->push_back(osg::Vec3(-length/2,-width/2,-height/2)+translation);
	    v->push_back(osg::Vec3(length/2,-width/2,-height/2)+translation);
	    v->push_back(osg::Vec3(length/2,-width/2,height/2)+translation);
	    v->push_back(osg::Vec3(-length/2,-width/2,height/2)+translation);

	    v->push_back(osg::Vec3(-length/2,width/2,height/2)+translation);
	    v->push_back(osg::Vec3(length/2,width/2,height/2)+translation);
	    v->push_back(osg::Vec3(length/2,width/2,-height/2)+translation);
	    v->push_back(osg::Vec3(-length/2,width/2,-height/2)+translation);


	    v->push_back(osg::Vec3(-length/2,width/2,-height/2)+translation);
	    v->push_back(osg::Vec3(-length/2,-width/2,-height/2)+translation);
	    v->push_back(osg::Vec3(-length/2,-width/2,height/2)+translation);
	    v->push_back(osg::Vec3(-length/2,width/2,height/2)+translation);

	    v->push_back(osg::Vec3(length/2,width/2,height/2)+translation);
	    v->push_back(osg::Vec3(length/2,-width/2,height/2)+translation);
	    v->push_back(osg::Vec3(length/2,-width/2,-height/2)+translation);
	    v->push_back(osg::Vec3(length/2,width/2,-height/2)+translation);


	    v->push_back(osg::Vec3(-length/2,-width/2,-height/2)+translation);
	    v->push_back(osg::Vec3(length/2,-width/2,-height/2)+translation);
	    v->push_back(osg::Vec3(length/2,width/2,-height/2)+translation);
	    v->push_back(osg::Vec3(-length/2,width/2,-height/2)+translation);

	    v->push_back(osg::Vec3(-length/2,width/2,height/2)+translation);
	    v->push_back(osg::Vec3(length/2,width/2,height/2)+translation);
	    v->push_back(osg::Vec3(length/2,-width/2,height/2)+translation);
	    v->push_back(osg::Vec3(-length/2,-width/2,height/2)+translation);

	    this->setVertexArray(v);
	    osg::ref_ptr<osg::DrawArrays> array=new osg::DrawArrays(GL_QUADS,0,24);
	    this->addPrimitiveSet(array);
	    return this;
}

void TestNumberedGeodeScene(osg::ref_ptr<osg::Group> & mainGroup,IDGeodeMap & IGM, ReverseIDGeodeMap & RIGM)
{
    osg::Vec4 white(1.0,1.0,1.0,1.0);
    osg::Vec4 red(1.0,0.0,0.0,1.0);
    osg::Vec4 blue(0.0,0.0,1.0,1.0);
    osg::Vec4 yellow(1.0,1.0,0.0,1.0);

    float size=20.0f;
    osg::ref_ptr<osg::Geode> box1Geode=new osg::Geode;


    osg::ref_ptr<BoxGeometry> box1=new BoxGeometry();
    box1Geode->addDrawable(box1->getBoxGeometry(2.0*size,2.0*size,4.0*size,osg::Vec3(50.0,0.0,0.0),white));
    mainGroup->addChild(box1Geode);

    mapIDAndGeode(0,box1Geode,IGM,RIGM);


    osg::ref_ptr<osg::Geode> box2Geode=new osg::Geode;

    osg::ref_ptr<BoxGeometry> box2=new BoxGeometry();
    box2Geode->addDrawable(box2->getBoxGeometry(2.0*size,2.0*size,4.0*size,osg::Vec3(-50.0,0.0,0.0),red));
    mainGroup->addChild(box2Geode);

    mapIDAndGeode(1,box2Geode,IGM,RIGM);

    osg::ref_ptr<osg::Geode> box3Geode=new osg::Geode;

    osg::ref_ptr<BoxGeometry> box3=new BoxGeometry();
    box3Geode->addDrawable(box3->getBoxGeometry(2.0*size,2.0*size,4.0*size,osg::Vec3(0.0,50.0,0.0),yellow));
    mainGroup->addChild(box3Geode);
    mapIDAndGeode(2,box3Geode,IGM,RIGM);
}
void mapIDAndGeode(int ID,osg::Geode* geode,IDGeodeMap & IGM, ReverseIDGeodeMap & RIGM)
{
	IGM.insert(IDGeodePair(ID,geode));
	RIGM.insert(GeodeIDPair(geode,ID));
}
void unMapIDAndGeode(int ID,osg::Geode* geode,IDGeodeMap & IGM, ReverseIDGeodeMap & RIGM)
{
	IGM.erase(ID);
	RIGM.erase(geode);
}
osg::ref_ptr<osg::Geode> Axis()
{

    osg::Vec4 red(1.0,0.0,0.0,1.0);
    osg::Vec4 blue(0.0,0.0,1.0,1.0);
    osg::Vec4 yellow(1.0,1.0,0.0,1.0);

    //osg::Vec4 mred(0.5,0.0,0.0,1.0);
    //osg::Vec4 mblue(0.0,0.0,0.5,1.0);
    //osg::Vec4 myellow(0.5,0.5,0.0,1.0);
    osg::Vec4 mred(1.0,0.0,1.0,1.0);
    osg::Vec4 mblue(0.0,1.0,1.0,1.0);
    osg::Vec4 myellow(0.5,0.5,0.5,1.0);
    float size=10;
	float axisLength=100.0;
    osg::ref_ptr<osg::Geode> geode=new osg::Geode;
	osg::ref_ptr<BoxGeometry> xaxis=new BoxGeometry();
	geode->addDrawable(xaxis->getBoxGeometry(axisLength*size,1.0*size,1.0*size,osg::Vec3(axisLength/2.0*size,0.0,0.0),red));
	osg::ref_ptr<BoxGeometry> mxaxis=new BoxGeometry();
	geode->addDrawable(mxaxis->getBoxGeometry(axisLength*size,1.0*size,1.0*size,osg::Vec3(-axisLength/2.0*size,0.0,0.0),mred));

	osg::ref_ptr<BoxGeometry> yaxis=new BoxGeometry();
	geode->addDrawable(yaxis->getBoxGeometry(1.0*size,axisLength*size,1.0*size,osg::Vec3(0.0,axisLength/2.0*size,0.0),blue));
	osg::ref_ptr<BoxGeometry> myaxis=new BoxGeometry();
	geode->addDrawable(myaxis->getBoxGeometry(1.0*size,axisLength*size,1.0*size,osg::Vec3(0.0,-axisLength/2.0*size,0.0),mblue));

	osg::ref_ptr<BoxGeometry> zaxis=new BoxGeometry();
	geode->addDrawable(zaxis->getBoxGeometry(1.0*size,1.0*size,axisLength*size,osg::Vec3(0.0,0.0,axisLength/2.0*size),yellow));
	osg::ref_ptr<BoxGeometry> mzaxis=new BoxGeometry();
	geode->addDrawable(mzaxis->getBoxGeometry(1.0*size,1.0*size,axisLength*size,osg::Vec3(0.0,0.0,-axisLength/2.0*size),myellow));

	return geode;
}
