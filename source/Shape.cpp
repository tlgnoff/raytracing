#include "Shape.h"
#include "Scene.h"
#include <cstdio>
#include <Eigen/Dense>

using namespace Eigen;

Shape::Shape(void)
{
}

Shape::Shape(int id, int matIndex)
    : id(id), matIndex(matIndex)
{
}

Sphere::Sphere(void)
{}

/* Constructor for sphere. You will implement this. */
Sphere::Sphere(int id, int matIndex, int cIndex, float R)
    : Shape(id, matIndex)
{
	centerIndex = cIndex;
  radius = R;
}

/* Sphere-ray intersection routine. You will implement this.
Note that ReturnVal structure should hold the information related to the intersection point, e.g., coordinate of that point, normal at that point etc.
You should to declare the variables in ReturnVal structure you think you will need. It is in defs.h file. */
ReturnVal Sphere::intersect(const Ray & ray) const
{
  ReturnVal result;
	float t, t1, t2, a, b, c, det, sqrtdet;
  Vector3f o, d, oc, center;
  center = pScene->vertices[centerIndex-1];
  o = ray.origin;
  d = ray.direction ;
  oc = o - center;
  a = d.dot(d);
  b = 2*d.dot(oc);
  c = (oc.dot(oc)) - radius*radius;
  det = b*b - 4*a*c;
  if(det < 0.f) {
    result.isIntersected = false;
    return result;
  }
  sqrtdet = sqrt(det);
  t1 = (-1*b - sqrtdet)/(2*a);
  t2 = (-1*b + sqrtdet)/(2*a);
  t = (t1 < t2) ? t1 : t2;
  if(t >= 0){
    result.isIntersected = true;
    result.coordinate = ray.origin + t*ray.direction;
    result.t = t;
    result.normal = (result.coordinate - center)/radius;
    if(result.normal[0] == 0) result.normal[0] = 0;
    if(result.normal[1] == 0) result.normal[1] = 0;
    if(result.normal[2] == 0) result.normal[2] = 0;
    return result;
  }
  result.isIntersected = false;
  return result;
}

Triangle::Triangle(void)
{}

/* Constructor for triangle. You will implement this. */
Triangle::Triangle(int id, int matIndex, int p1Index, int p2Index, int p3Index)
    : Shape(id, matIndex)
{
	this->p1Index = p1Index;
  this->p2Index = p2Index;
  this->p3Index = p3Index;
}

/* Triangle-ray intersection routine. You will implement this.
Note that ReturnVal structure should hold the information related to the intersection point, e.g., coordinate of that point, normal at that point etc.
You should to declare the variables in ReturnVal structure you think you will need. It is in defs.h file. */
ReturnVal Triangle::intersect(const Ray & ray) const
{
  ReturnVal result;
	float t, beta, gamma, abx, aby, abz, acx, acy, acz, aox, aoy, aoz, a_det;
  Matrix3f a_matrix, t_matrix, beta_matrix, gamma_matrix;
  Vector3f o, d, p1, p2, p3;
  p1 = pScene->vertices[p1Index-1];
  p2 = pScene->vertices[p2Index-1];
  p3 = pScene->vertices[p3Index-1];
  o = ray.origin;
  d = ray.direction;
  abx = p1(0) - p2(0);
  aby = p1(1) - p2(1);
  abz = p1(2) - p2(2);
  acx = p1(0) - p3(0);
  acy = p1(1) - p3(1);
  acz = p1(2) - p3(2);
  aox = p1(0) - o(0);
  aoy = p1(1) - o(1);
  aoz = p1(2) - o(2);
  a_matrix << abx, acx, d(0),
              aby, acy, d(1),
              abz, acz, d(2);
  t_matrix << abx, acx, aox,
              aby, acy, aoy,
              abz, acz, aoz;
  beta_matrix << aox, acx, d(0),
                 aoy, acy, d(1),
                 aoz, acz, d(2);
  gamma_matrix << abx, aox, d(0),
                  aby, aoy, d(1),
                  abz, aoz, d(2);
  a_det = a_matrix.determinant();
  t = t_matrix.determinant()/a_det;
  beta = beta_matrix.determinant()/a_det;
  gamma = gamma_matrix.determinant()/a_det;
  if(beta + gamma <= 1+pScene->intTestEps && beta >= 0 - pScene->intTestEps && gamma >= 0 - pScene->intTestEps && t >= pScene->intTestEps) {
    result.isIntersected = true;
    result.coordinate = o + t*d;
    result.t = t;
    result.normal = (p3 - p2).cross(p1 - p2).normalized();
    return result;
  }
  result.isIntersected = false;
  return result;
}

Mesh::Mesh()
{}

/* Constructor for mesh. You will implement this. */
Mesh::Mesh(int id, int matIndex, const vector<Triangle>& faces)
    : Shape(id, matIndex)
{
	this->faces = faces;
}

/* Mesh-ray intersection routine. You will implement this.
Note that ReturnVal structure should hold the information related to the intersection point, e.g., coordinate of that point, normal at that point etc.
You should to declare the variables in ReturnVal structure you think you will need. It is in defs.h file. */
ReturnVal Mesh::intersect(const Ray & ray) const
{
	ReturnVal result, tempresult;
  float tmin = std::numeric_limits<float>::max();
  size_t facesSize = faces.size();

  result.isIntersected = false;
  for(int i = 0; i < facesSize; i++){
    Triangle triangle = faces[i];
    tempresult = triangle.intersect(ray);
    if(tempresult.isIntersected && tempresult.t < tmin-pScene->shadowRayEps && tempresult.t >= 0) {
      result = tempresult;
      tmin = result.t;
    }
  }
  return result;
}
