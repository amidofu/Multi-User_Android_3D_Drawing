#ifndef AUXFUNC_H
#define AUXFUNC_H
#include <osg/Geometry>
#include <osg/Vec4>
#include <osg/Material>
#include <osg/Geode>
osg::ref_ptr<osg::Geode> Trapzoid(osg::Vec4 & color);
osg::ref_ptr<osg::Geode> Triangle(osg::Vec4 & color,float offset);
void ComputeBoundingBoxByRotation(osg::BoundingBox & InOut,osg::Vec3 BBCenter,float BBSize,osg::Matrixd Rotation);
#endif
