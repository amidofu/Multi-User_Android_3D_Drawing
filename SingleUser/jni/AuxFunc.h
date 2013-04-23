#ifndef AUXFUNC_H
#define AUXFUNC_H
#include <osg/Geometry>
#include <osg/Vec4>
#include <osg/Material>
#include <osg/Geode>
osg::ref_ptr<osg::Geode> Trapzoid(osg::Vec4 & color);
osg::ref_ptr<osg::Geode> Triangle(osg::Vec4 & color,float offset);
void ComputeBoundingBoxByRotation(osg::BoundingBox & InOut,osg::Vec3 BBCenter,float BBSize,osg::Matrixd Rotation);
void extendVertexArray(osg::ref_ptr<osg::Vec3Array> & src,osg::ref_ptr<osg::Vec3Array> & dest);
void extendVec3xArray(osg::Vec3Array* src,osg::Vec3Array* dest);
void extendVec3xArray(osg::Array * src, osg::Vec3Array* dest);
#endif
