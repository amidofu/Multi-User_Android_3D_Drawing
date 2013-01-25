#include "AuxFunc.h"

osg::ref_ptr<osg::Geode> Trapzoid(osg::Vec4 & color)
{
	osg::ref_ptr<osg::Geode> geode=new osg::Geode;


	    osg::ref_ptr<osg::Vec3Array> v=new osg::Vec3Array;

	    v->push_back(osg::Vec3(-10,0,-10));
	    v->push_back(osg::Vec3(10,0,-10));
	    v->push_back(osg::Vec3(5,0,10));
	    v->push_back(osg::Vec3(-5,0,10));

	    osg::ref_ptr<osg::Vec4Array> colors=new osg::Vec4Array;

	    colors->push_back(color);
	    colors->push_back(color);
	    colors->push_back(color);
	    colors->push_back(color);

	    //triangle drew by triangle strip for test
	    osg::ref_ptr<osg::Geometry> geometry2=new osg::Geometry();
	    geometry2->setVertexArray(v);
	    geometry2->setColorArray(colors);
	    geometry2->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	    osg::ref_ptr<osg::DrawArrays> array=new osg::DrawArrays(GL_QUADS,0,4);
	    geometry2->addPrimitiveSet(array);
	    geode->addDrawable(geometry2);
	    geode->setDataVariance(osg::Object::STATIC);
	    osg::ref_ptr<osg::Material> mat=new osg::Material;
	    mat->setDiffuse(osg::Material::FRONT_AND_BACK,color);
	    geode->getOrCreateStateSet()->setAttributeAndModes(mat.get());
	    geode->getOrCreateStateSet()->setMode(GL_CULL_FACE,osg::StateAttribute::OFF);
	    return geode;
}

osg::ref_ptr<osg::Geode> Triangle(osg::Vec4 & color,float offset)
{
	osg::ref_ptr<osg::Geode> geode=new osg::Geode;


	    osg::ref_ptr<osg::Vec3Array> v=new osg::Vec3Array;

	    v->push_back(osg::Vec3(-10,offset,-10));
	    v->push_back(osg::Vec3(10,offset,-10));
	    v->push_back(osg::Vec3(0,offset,50));



	    osg::ref_ptr<osg::Vec4Array> colors=new osg::Vec4Array;

	    colors->push_back(color);
	    colors->push_back(color);
	    colors->push_back(color);
	    colors->push_back(color);

	    //triangle drew by triangle strip for test
	    osg::ref_ptr<osg::Geometry> geometry2=new osg::Geometry();
	    geometry2->setVertexArray(v);
	    geometry2->setColorArray(colors);
	    geometry2->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	    osg::ref_ptr<osg::DrawArrays> array=new osg::DrawArrays(GL_TRIANGLES,0,3);
	    geometry2->addPrimitiveSet(array);
	    geode->addDrawable(geometry2);
	    geode->setDataVariance(osg::Object::STATIC);
	    osg::ref_ptr<osg::Material> mat=new osg::Material;
	    mat->setDiffuse(osg::Material::FRONT_AND_BACK,color);
	    geode->getOrCreateStateSet()->setAttributeAndModes(mat.get());
	    geode->getOrCreateStateSet()->setMode(GL_CULL_FACE,osg::StateAttribute::OFF);
	    return geode;
}
void ComputeBoundingBoxByRotation(osg::BoundingBox & InOut,osg::Vec3 BBCenter,float BBSize,osg::Matrixd Rotation)
{
	osg::BoundingBox bb;
	bb.set(BBCenter.x()-BBSize,BBCenter.y()-BBSize,BBCenter.z()-BBSize,BBCenter.x()+BBSize,BBCenter.y()+BBSize,BBCenter.z()+BBSize);
    osg::BoundingBox Tbb;
	for(unsigned int i=0;i<8;i++)
    {
        Tbb.expandBy(Rotation.preMult(bb.corner(i)));
    }
	InOut.set(Tbb.xMin(),Tbb.yMin(),Tbb.zMin(),Tbb.xMax(),Tbb.yMax(),Tbb.zMax());
}
