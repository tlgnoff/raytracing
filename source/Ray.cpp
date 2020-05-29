#include "Ray.h"
#include <Eigen/Dense>

using namespace Eigen;

Ray::Ray()
{
}

Ray::Ray(const Vector3f& origin, const Vector3f& direction)
    : origin(origin), direction(direction)
{
}

/* Takes a parameter t and returns the point accoring to t. t is the parametric variable in the ray equation o+t*d.*/
Vector3f Ray::getPoint(float t) const
{
	return origin + t*direction;
}

/* Takes a point p and returns the parameter t according to p such that p = o+t*d. */
float Ray::gett(const Vector3f & p) const
{
  Vector3f po = p - origin;
  float t0 = po[0]/direction[0];
  float t1 = po[1]/direction[1];
  float t2 = po[2]/direction[2];
  return std::max(std::max(t0, t1), t2);
}
