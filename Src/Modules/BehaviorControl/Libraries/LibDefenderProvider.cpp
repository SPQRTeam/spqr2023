/**
 * @file LibDefenderProvider.cpp
 * 
 * See LibDefender
 *
 * @author Francesco Petri
 */

#include "LibDefenderProvider.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <iostream>


MAKE_MODULE(LibDefenderProvider, behaviorControl);

void LibDefenderProvider::update(LibDefender& libDefender)
{
  DECLARE_DEBUG_DRAWING("module:LibDefenderProvider:defenderPosition", "drawingOnField");

  libDefender.getDefenderonePosition = [this]() -> Vector2f {
    return getDefenderonePosition();
  };
  libDefender.getDefendertwoPosition = [this]() -> Vector2f {
    return getDefendertwoPosition();
  };
  libDefender.getDefenderoneOrientation = [this](bool ballSeen, Vector2f robotPosition) -> Angle {
    return getDefenderoneOrientation(ballSeen, robotPosition);
  };
  libDefender.getDefendertwoOrientation = [this](bool ballSeen, Vector2f robotPosition) -> Angle {
    return getDefendertwoOrientation(ballSeen, robotPosition);
  };
  libDefender.targetOnSemiCircle =[this](Vector2f segmentStart, Vector2f segmentEnd, float minRadius, float maxRadius) -> Vector2f {
    return targetOnSemiCircle(segmentStart, segmentEnd, minRadius, maxRadius);
  };
}


Vector2f LibDefenderProvider::getDefenderonePosition() const {
  Vector2f target;
  const Vector2f leftGoalPost{theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosLeftGoal - 100.f};
  const Vector2f goalCenter{theFieldDimensions.xPosOwnGroundLine, 0.f};

  bool ballSeen = theFieldBall.ballWasSeen(500);
  const Vector2f ballPos = ballSeen ? theFieldBall.recentBallPositionOnField() : theTeamBallModel.position; //global

  //OPPONENT CORNER
  if(theGameState.state == GameState::State::opponentCorner)
  {
    if(theTeamBallModel.position.y()>0) //left corner
      target = Vector2f(theFieldDimensions.xPosOwnGroundLine + 200, theFieldDimensions.yPosLeftPenaltyArea - 200);
    else
      target = Vector2f(theFieldDimensions.xPosOwnGroundLine + 200, theFieldDimensions.yPosRightPenaltyArea + 200);
  }
  //ORDINARY SITUATION
  else{
    target = targetOnSemiCircle(ballPos, leftGoalPost, 800, 1300);
  }

  LINE("module:LibDefenderProvider:defenderPosition", ballPos.x(), ballPos.y(), leftGoalPost.x(), leftGoalPost.y(), 80, Drawings::PenStyle::solidPen, ColorRGBA::blue);
  CIRCLE("module:LibDefenderProvider:defenderPosition", target.x(), target.y(), 80, 50, Drawings::PenStyle::solidPen, ColorRGBA::blue, Drawings::BrushStyle::solidBrush, ColorRGBA::blue);
  CYLINDER3D("module:LibDefenderProvider:defenderPosition", target.x(), target.y(), 0.0f, 0.0f, 0.0f, 0.0f, 100.0f, 0.0f, ColorRGBA::blue);

  return (target.hasNaN() || !theFieldDimensions.isInsideField(target)) ? Vector2f(-3000.f, -1000.f) : target;
}

Vector2f LibDefenderProvider::getDefendertwoPosition() const {
  Vector2f target;
  const Vector2f rightGoalPost{theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosRightGoal + 100.f};
  const Vector2f goalCenter{theFieldDimensions.xPosOwnGroundLine, 0.f};

  bool ballSeen = theFieldBall.ballWasSeen(500);
  const Vector2f ballPos = ballSeen ? theFieldBall.recentBallPositionOnField() : theTeamBallModel.position; //global


  //OPPONENT CORNER
  if(theGameState.state == GameState::State::opponentCorner)
  {
    target = theLibSpec.targetDefenseCornerPoint();
  }
  //ORDINARY SITUATION
  else
  {
    target = targetOnSemiCircle(ballPos, rightGoalPost, 1100, 1600);
  }

  LINE("module:LibDefenderProvider:defenderPosition", ballPos.x(), ballPos.y(), rightGoalPost.x(), rightGoalPost.y(), 80, Drawings::PenStyle::solidPen, ColorRGBA::cyan);
  CIRCLE("module:LibDefenderProvider:defenderPosition", target.x(), target.y(), 80, 50, Drawings::PenStyle::solidPen, ColorRGBA::cyan, Drawings::BrushStyle::solidBrush, ColorRGBA::cyan);
  CYLINDER3D("module:LibDefenderProvider:defenderPosition", target.x(), target.y(), 0.0f, 0.0f, 0.0f, 0.0f, 100.0f, 0.0f, ColorRGBA::cyan);

  return (target.hasNaN() || !theFieldDimensions.isInsideField(target))  ? Vector2f(-3000.f, 1000.f) : target;
}

Angle LibDefenderProvider::getDefenderoneOrientation(bool ballSeen, Vector2f robotPosition) const {
  Vector2f bestBall = ballSeen ? theFieldBall.recentBallPositionOnField() : theTeamBallModel.position;
  Angle recoveryAngle = (Vector2f(theFieldDimensions.xPosOpponentGoal, 0.f) - robotPosition).angle();
  Angle targetAngle = (bestBall - robotPosition).angle();
  return std::isnan(targetAngle) ? recoveryAngle : targetAngle;
}

Angle LibDefenderProvider::getDefendertwoOrientation(bool ballSeen, Vector2f robotPosition) const {
  Vector2f bestBall = ballSeen ? theFieldBall.recentBallPositionOnField() : theTeamBallModel.position;
  Angle recoveryAngle = (Vector2f(theFieldDimensions.xPosOpponentGoal, 0.f) - robotPosition).angle();
  Angle targetAngle = (bestBall - robotPosition).angle();
  return std::isnan(targetAngle) ? recoveryAngle : targetAngle;
}


Vector2f LibDefenderProvider::targetOnSemiCircle(Vector2f segmentStart, Vector2f segmentEnd, float minRadius, float maxRadius) const {
//global target
  
  // Line from segment start (ball pos or an opponent) to segment end (for now right or left goal post)
  const Geometry::Line line(segmentStart, segmentEnd - segmentStart);

  //Radius computation
  const float startDistance = theFieldBall.distanceToOwnPenaltyArea;
  float alpha = startDistance/theFieldDimensions.xPosOpponentPenaltyArea;
  if (alpha > 1) alpha = 1; //clipping
  float radius{};
  if (startDistance == -1.f){
    radius = minRadius;
  } else{
    radius = minRadius + alpha*(maxRadius - minRadius);
  }

  // Circle with center in goal 
  Vector2f center{theFieldDimensions.xPosOwnGroundLine + 200.f, 0.f}; //global
  const Geometry::Circle circle(center, radius); 
  CIRCLE("module:LibDefenderProvider:defenderPosition", center.x(), center.y(), radius,  40, Drawings::solidPen, ColorRGBA::black, Drawings::noPen, ColorRGBA::black);
  

  Vector2f target{};
  //Data preparation
  const Vector2f d = line.direction;
  const Vector2f q = line.base - circle.center; 
  const float a = std::pow(d.x(),2) + std::pow(d.y(),2);
  const float b = 2*(d.x()*q.x() + d.y()*q.y());
  const float c = std::pow(q.x(),2) + std::pow(q.y(),2) - std::pow(radius,2);

  //Solving snd order equation
  const float delta = std::pow(b,2) - 4*a*c;
  if (delta >= 0) {
    const float first = (-b + std::sqrt(delta)) / (2*a);
    const float second =(-b - std::sqrt(delta)) / (2*a);

    const Vector2f firstIntercept = line.base + first * d;
    const Vector2f secondIntercept = line.base + second * d;  

    target = firstIntercept.x() > theFieldDimensions.xPosOwnGroundLine ? firstIntercept : secondIntercept; //still global
  } else {
    target = Vector2f(-3000.f, 0.f); //still still global
  }
  /// Intersection of line and circle
  return target;
}







