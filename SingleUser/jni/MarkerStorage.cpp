#include "MarkerStorage.h"
float showAngle(Point3f center, Point3f p1, Point3f p2)
{
    Point3f v1=p1-center;
    Point3f v2=p2-center;
    normalizeP3f(v1);
    normalizeP3f(v2);
    return acos(v1.dot(v2));

}
float P3fLength(Point3f p)
{
    float x=p.x;
    float y=p.y;
    float z=p.z;
    return sqrt(x*x+y*y+z*z);
}
void normalizeP3f(Point3f & p)
{
    float length=P3fLength(p);
    p.x=p.x/length;
    p.y=p.y/length;
    p.z=p.z/length;
}
float P3fDist(Point3f p1,Point3f p2)
{
    return P3fLength(p2-p1);
}
