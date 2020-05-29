#include "Camera.h"
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;

Camera::Camera(int id,                      // Id of the camera
               const char* imageName,       // Name of the output PPM file
               const Vector3f& pos,         // Camera position
               const Vector3f& gaze,        // Camera gaze direction
               const Vector3f& up,          // Camera up direction
               const ImagePlane& imgPlane)  // Image plane parameters
          : id(id), imgPlane(imgPlane)
{
  strcpy(this->imageName, imageName);
  this->pos = pos;
  this->gaze = gaze;
  this->up = up;
}

/* Takes coordinate of an image pixel as row and col, and
 * returns the ray going through that pixel.
 */
Ray Camera::getPrimaryRay(int col, int row) const
{
	Ray ray;
  Vector3f m, q, s, u;
  float su, sv;
  u = (this->up).cross(-1*this->gaze);
  m = this->pos + this->gaze*(this->imgPlane).distance;
  q = m + (this->imgPlane).left*u + (this->imgPlane).top*this->up;
  su = (col + 0.5) * ((this->imgPlane).right - (this->imgPlane).left)/(this->imgPlane).nx;
  sv = (row + 0.5) * ((this->imgPlane).top - (this->imgPlane).bottom)/(this->imgPlane).ny;
  s = q + su*u - sv*this->up;
  ray.origin = this->pos;
  ray.direction = (s - this->pos).normalized();
  return ray;
}
