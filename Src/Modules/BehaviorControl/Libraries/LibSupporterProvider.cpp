/**
 * @file LibSupporterProvider.cpp
 * 
 * See LibSupporter
 *
 * @author Francesco Petri
 */

#include "LibSupporterProvider.h"
#include <iostream>
#include "Tools/Debugging/DebugDrawings3D.h"

MAKE_MODULE(LibSupporterProvider, behaviorControl);

void LibSupporterProvider::update(LibSupporter& libSupporter)
{
  DECLARE_DEBUG_DRAWING3D("module:LibSupporterProvider:supporterPosition", "field");

  libSupporter.getSupporterPosition = [this]() -> Vector2f {
    return getSupporterPosition();
  };

  libSupporter.getSupporterOrientation = [this](bool ballSeen, Vector2f robotPosition) -> Angle {
    return getSupporterOrientation(ballSeen, robotPosition);
  };
}

// Unused
float LibSupporterProvider::distanceToLine(Vector2f objectToCheck, Vector2f linePoint1, Vector2f linePoint2) const {
  return std::abs(((linePoint2.y()-linePoint1.y())*objectToCheck.x()) - ((linePoint2.x() - linePoint1.x()) * objectToCheck.y())
    + (linePoint2.x() * linePoint1.y()) - (linePoint2.y() * linePoint1.x())) / ((linePoint1-linePoint2).norm());
}


Vector2f LibSupporterProvider::getSupporterPosition() const {
  Vector2f target;
  if(theFieldBall.recentBallPositionOnField().x() > theFieldDimensions.xPosOwnGoalArea){
    target = Vector2f(theFieldBall.recentBallPositionOnField().x() - 1000.f, theFieldBall.recentBallPositionOnField().y());     
  }else{
    if(theFieldBall.recentBallPositionOnField().y() > 0){
      target = Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosLeftGoal);
    }else{
      target = Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosRightGoal);
    }       
  }
  if(theRobotInfo.number==4)
    CYLINDER3D("module:LibSupporterProvider:supporterPosition", target.x(), target.y(), 0.0f, 0.0f, 0.0f, 0.0f, 50.0f, 60.0f, ColorRGBA::blue);
  
  return target.hasNaN() ? Vector2f(1000.f,1000.f) : target;
}


Angle LibSupporterProvider::getSupporterOrientation(bool ballSeen, Vector2f robotPosition) const{
  Vector2f bestBall = ballSeen ? theFieldBall.recentBallPositionOnField() : theTeamBallModel.position;
  Angle recoveryAngle = (Vector2f(theFieldDimensions.xPosOpponentGoal, 0.f) - robotPosition).angle();
  Angle targetAngle = (bestBall - robotPosition).angle();
  return std::isnan(targetAngle) ? recoveryAngle : targetAngle;
}