// Copyright @2018 Pony AI Inc. All rights reserved.

#include "homework1/protobuf/canvas.h"

#include <iostream>
#include <glog/logging.h>

namespace homework1 {

using homework1::geometry::Point3D;

void Canvas::Draw() const {
  for (const auto& p : polygon_.point()) {
    std::cout << "Point:" << p.x() << ", " << p.y() << ", " << p.z() << std::endl;
  }
}

void Canvas::AddPoint(double x, double y, double z) {
  Point3D point;
  point.set_x(x);
  point.set_y(y);
  point.set_z(z);
  AddPoint(point);
}

void Canvas::AddPoint(const Point3D& p) {
  auto* point = polygon_.add_point();
  point->CopyFrom(p);
}

const Point3D& Canvas::GetPoint(int index) const {
  return polygon_.point(index);
}

void Canvas::ParseFromString(const std::string& serialzation) {
  polygon_.ParseFromString(serialzation);
}

const std::string Canvas::SerializeToString() const {
  std::string serialzation;
  CHECK(polygon_.SerializeToString(&serialzation)) << "Canvas serialization failed.";
  return serialzation;
}

double sqr(double x){
  return x * x;
}

const double Canvas::GetLength() const{
  double polyLen = .0;
  bool _ = false;
  Point3D lastPoint, firstPoint;
  for (const auto& point:polygon_.point()){
    if (_)
      polyLen += sqrt(sqr(point.x() - lastPoint.x()) + sqr(point.y() - lastPoint.y()) + sqr(point.z() - lastPoint.z()));
    else
      firstPoint = point;
    _ = true;
    lastPoint = point;
  }
  polyLen += sqrt(sqr(lastPoint.x() - firstPoint.x()) + sqr(lastPoint.y() - firstPoint.y()) + sqr(lastPoint.z() - firstPoint.z()));
  return polyLen;
}

}  // namespace homework1
