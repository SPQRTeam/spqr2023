/**
 * @file LibMiscProvider.cpp
 * 
 * See LibMisc
 *
 * @author Francesco Petri
 */

#include "LibMiscProvider.h"

MAKE_MODULE(LibMiscProvider, behaviorControl);

void LibMiscProvider::update(LibMisc& libMisc)
{
  libMisc.localDirectionToField = [this](const Vector2f& localDirection) {
    return localDirectionToField(localDirection);
  };
  libMisc.distance = [this](const Pose2f& p1, const Pose2f& p2) {
    return distance(p1, p2);
  };
  libMisc.distanceVec = [this](const Vector2f& p1, const Vector2f& p2) {
    return distanceVec(p1, p2);
  };
  libMisc.mapToInterval = [this](float value, float fromIntervalMin, float fromIntervalMax, float toIntervalMin, float toIntervalMax) -> float {
    return mapToInterval(value, fromIntervalMin, fromIntervalMax, toIntervalMin, toIntervalMax);
  };
  libMisc.isValueBalanced = [this](float currentValue, float target, float bound) -> bool {
    return isValueBalanced(currentValue, target, bound);
  };
  libMisc.angleToTarget = [this](float x, float y) -> float {
    return angleToTarget(x, y);
  };
  libMisc.glob2Rel = [this](float x, float y) -> Pose2f {
    return glob2Rel(x, y);
  };
  libMisc.rel2Glob = [this](float x, float y) -> Pose2f {
    return rel2Glob(x, y);
  };
  libMisc.norm = [this](float x, float y) -> float {
    return norm(x, y);
  };
  libMisc.radiansToDegree = [this](float x) -> float {
    return radiansToDegree(x);
  };
  libMisc.angleBetweenPoints = [this](const Vector2f& p1, const Vector2f& p2) -> float {
    return angleBetweenPoints(p1, p2);
  };
  libMisc.angleBetweenGlobalVectors = [this](const Vector2f& start, const Vector2f& end1, const Vector2f& end2) -> float {
    return angleBetweenGlobalVectors(start, end1, end2);
  };
  libMisc.isInsideGlobalSector = [this](const Vector2f startBounds, const Vector2f endBound1, const Vector2f endBound2, float startRadius, float endRadius, const Vector2f point) -> bool {
    return isInsideGlobalSector(startBounds, endBound1, endBound2, startRadius, endRadius, point);
  };
  libMisc.isInsideGlobalRectangle = [this](const Vector2f point, const Vector2f bottom, const Vector2f left, const Vector2f right, const Vector2f up) -> bool {
    return isInsideGlobalRectangle(point, bottom, left, right, up);
  };


  libMisc.angleToGoal = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundLine, 0.f)).angle();
  libMisc.angleToBall = (theRobotPose.inversePose * theFieldBall.positionOnField).angle();
}


int LibMiscProvider::localDirectionToField(const Vector2f& localDirection) const {
  Vector2f globalDirection = localDirection.rotated(theRobotPose.rotation);
  if (globalDirection.x() >= 0) return 1;
  else return -1;
}

float LibMiscProvider::distance(const Pose2f& p1, const Pose2f& p2) const {
  return static_cast<float>( std::sqrt( std::pow(p2.translation.x() - p1.translation.x(), 2) +
    std::pow(p2.translation.y() - p1.translation.y(), 2) ) );
}

float LibMiscProvider::distanceVec(const Vector2f& p1, const Vector2f& p2) const {     //TODO: Flavio&Valerio
  return static_cast<float>( std::sqrt( std::pow(p2.x() - p1.x(), 2) +
    std::pow(p2.y() - p1.y(), 2) ) );
}

float LibMiscProvider::mapToInterval(float value, float fromIntervalMin, float fromIntervalMax, float toIntervalMin, float toIntervalMax) const {
  float fromIntervalSize = fromIntervalMax - fromIntervalMin;
  float toIntervalSize = toIntervalMax - toIntervalMin;
  if(value > fromIntervalMax) return toIntervalMax;
  else if (value<fromIntervalMin) return toIntervalMin;
  else return toIntervalMin + (value - fromIntervalMin) * toIntervalSize / fromIntervalSize;
}

bool LibMiscProvider::isValueBalanced(float currentValue, float target, float bound) const {
  float minErr = currentValue - (target - bound);
  float maxErr = currentValue - (target + bound);

  if( std::abs(minErr) < bound*1.2 && std::abs(maxErr) < bound*1.2 )
    return true;
  else
    return false;
}

float LibMiscProvider::angleToTarget(float x, float y) const
{
  Pose2f relativePosition = glob2Rel(x,y);
  return (atan2f(relativePosition.translation.y(), relativePosition.translation.x()));
}

Pose2f LibMiscProvider::glob2Rel(float x, float y) const
{
  Vector2f result;
  float theta = 0;
  float tempX = x - theRobotPose.translation.x();
  float tempY = y - theRobotPose.translation.y();

  result.x() = (float)(tempX * cos(theRobotPose.rotation) + tempY * sin(theRobotPose.rotation));
  result.y() = (float)(-tempX * sin(theRobotPose.rotation) + tempY * cos(theRobotPose.rotation));

  return Pose2f(theta /*deg*/, result.x(),result.y());
}

Pose2f LibMiscProvider::rel2Glob(float x, float y) const
{
  Vector2f result;
  float rho = (float)(sqrt((x * x) + (y * y)));

  result.x() = (float)(theRobotPose.translation.x() + (rho * cos(theRobotPose.rotation + atan2(y,x))));
  result.y() = (float)(theRobotPose.translation.y() + (rho * sin(theRobotPose.rotation + atan2(y,x))));

  return Pose2f(result.x(),result.y());
}

float LibMiscProvider::norm(float x, float y) const
{
  return sqrtf((x*x) + (y*y));
};

float LibMiscProvider::radiansToDegree(float x) const
{
  return (float)((x*180)/3.14159265358979323846);
}

// [torch, 2022] I do not understand why deltaY is noy p2-p1 like it *feels* like it should be.
//               Am I missing something obvious?
//               Did the author forcefully enforce some nonstandard convention?
//               Anyway, this is used in a couple modules, so I can't change this at the moment.
//               Future users, make sure this is really what you want.
float LibMiscProvider::angleBetweenPoints(const Vector2f& p1, const Vector2f& p2) const {
  float deltaY = p1.y() - p2.y();
  float deltaX = p2.x() - p1.x();
  return atan2f(deltaY, deltaX);
}

float LibMiscProvider::angleBetweenVectors(const Vector2f& v1, const Vector2f& v2) const {
  return atan2f( v2.y()*v1.x() - v2.x()*v1.y(), v2.x()*v1.x() + v2.y()*v1.y());
}

float LibMiscProvider::angleBetweenGlobalVectors(const Vector2f& start, const Vector2f& end1, const Vector2f& end2) const {
  Vector2f v1 = Vector2f(end1.x() - start.x(), end1.y()-start.y());
  Vector2f v2 = Vector2f(end2.x() - start.x(), end2.y()-start.y());
  float cos_theta = ((v1.x()*v2.x())+(v1.y()*v2.y()))/(v1.norm()*v2.norm());
  float theta = acos(cos_theta);
  return theta;
}

bool LibMiscProvider::isInsideGlobalSector(const Vector2f startBounds, const Vector2f endBound1, const Vector2f endBound2, float startRadius, float endRadius, const Vector2f point) const{
  float radius_point = distanceVec(startBounds, point);
  if(radius_point < startRadius || radius_point > endRadius)
    return false;
  float angle_btw_bounds = angleBetweenGlobalVectors(startBounds, endBound1, endBound2);
  float angle_btw_b1_pnt = angleBetweenGlobalVectors(startBounds, endBound1, point);

  //std::cout << "angle_btw_bounds: " << angle_btw_bounds <<std::endl;
  //std::cout << "angle_btw_b1_pnt: " << angle_btw_b1_pnt <<std::endl;
  //std::cout << "--------------" <<std::endl;
  if (angle_btw_b1_pnt > angle_btw_bounds)
    return false; 
  return true;
}  

bool LibMiscProvider::isInsideGlobalRectangle(const Vector2f point, const Vector2f bottom, const Vector2f left, const Vector2f right, const Vector2f up) const{
  if(point.x() >= bottom.x() && point.x()<=up.x() && point.y() <= left.y() && point.y() >= right.y()){
    return true;
  }
  return false;
}