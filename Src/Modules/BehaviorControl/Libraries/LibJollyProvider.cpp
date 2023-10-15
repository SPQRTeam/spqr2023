/**
 * @file LibJollyProvider.cpp
 * 
 * See LibJolly
 *
 * @author Francesco Petri
 */

#include "LibJollyProvider.h"
#include "Tools/Debugging/DebugDrawings3D.h"

MAKE_MODULE(LibJollyProvider, behaviorControl);

void LibJollyProvider::update(LibJolly& libJolly)
{
  DECLARE_DEBUG_DRAWING3D("module:LibJollyProvider:jollyPosition", "field");
  libJolly.getJollyPosition = [this]() -> Vector2f {
    return getJollyPosition();
  };

  libJolly.getJollyOrientation = [this](bool ballSeen, Vector2f robotPosition) -> Angle {
    return getJollyOrientation(ballSeen, robotPosition);
  };
}


Vector2f LibJollyProvider::getJollyPosition() const {
  Vector2f target;
  if(theGameInfo.state==STATE_PLAYING && theGameInfo.setPlay == SET_PLAY_CORNER_KICK && theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber){
    std::tuple<float, float> angles = theLibSpec.calcAngleCorner();
    float angle_to_kick = std::get<0>(angles);
    
    float radius = 2500;
    Vector2f targetPoint = theLibSpec.targetCornerPoint(angle_to_kick, radius);
    target = Vector2f(targetPoint.x(), targetPoint.y());
  }
  else{
    if(theFieldBall.recentBallPositionOnField().y() > 0){
      //follow ball right
      if(theFieldBall.recentBallPositionOnField().x() > theFieldDimensions.xPosOwnGoalArea){
        target = Vector2f(clip(theFieldBall.recentBallPositionOnField().x() + 700.f, theFieldDimensions.xPosOwnPenaltyMark,
        theFieldDimensions.xPosOpponentPenaltyMark), theFieldBall.recentBallPositionOnField().y() -800.f);        
      }else{
        target = Vector2f(theFieldDimensions.xPosOwnGoalArea/1.5, theFieldDimensions.yPosLeftGoal);
      }
    }
    else{
      //follow ball left
      if(theFieldBall.recentBallPositionOnField().x() > theFieldDimensions.xPosOwnGoalArea){
        target = Vector2f(clip(theFieldBall.recentBallPositionOnField().x() + 700.f, theFieldDimensions.xPosOwnPenaltyMark,
        theFieldDimensions.xPosOpponentPenaltyMark), theFieldBall.recentBallPositionOnField().y() +800.f);
      }else{
        target = Vector2f(theFieldDimensions.xPosOwnGoalArea/1.5, theFieldDimensions.yPosRightGoal);
      }
    }
  }
  if(theRobotInfo.number==4)
    CYLINDER3D("module:LibJollyProvider:jollyPosition", target.x(), target.y(), 0.0f, 0.0f, 0.0f, 0.0f, 50.0f, 60.0f, ColorRGBA::green);
  
  return target.hasNaN() ? Vector2f(1000.f, -1000.f) : target;
}

Angle LibJollyProvider::getJollyOrientation(bool ballSeen, Vector2f robotPosition) const{
  float alpha = 0.8;
  ASSERT(alpha <= 1);
  Vector2f bestBall = ballSeen ? theFieldBall.recentBallPositionOnField() : theTeamBallModel.position;
  Angle targetAngleGoal = (Vector2f(theFieldDimensions.xPosOpponentGoal, 0.f) - robotPosition).angle();
  Angle targetAngleBall = (bestBall - robotPosition).angle();
  Angle targetAngle = alpha * targetAngleBall + (1-alpha)*targetAngleGoal;
  return std::isnan(targetAngle) ? targetAngleGoal : targetAngle;
}


