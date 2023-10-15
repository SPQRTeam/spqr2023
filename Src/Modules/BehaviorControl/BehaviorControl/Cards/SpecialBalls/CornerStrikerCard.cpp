/**
 * @file CornerStrikerCard.cpp
 *
 * This file implements the striker behavior for the attack and defense corner.
 *
 * @author Flavio Volpi, Valerio Spagnoli
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/BehaviorControl/Libraries/LibStriker.h"
#include "Representations/BehaviorControl/Libraries/LibObstacles.h"
#include "Representations/BehaviorControl/PathPlanner.h"
#include "Tools/Math/BHMath.h"

//CORNER
#include "Representations/Communication/GameInfo.h"   
#include "Representations/BehaviorControl/Libraries/LibSpec.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/Modeling/ObstacleModel.h"
#include <list>
#include "Tools/Debugging/DebugDrawings3D.h"


#include <iostream>

CARD(CornerStrikerCard,
{,
  CALLS(Activity),
  CALLS(GoToBallAndKick),
  CALLS(GoToBallAndDribble),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(Dribble),
  CALLS(LookActive),
  CALLS(LookAtBall),
  CALLS(WalkToBallAndKick),
  CALLS(WalkPotentialField), // opponent corner
  CALLS(LookAtGlobalBall), // opponent corner
  CALLS(WalkToPoint),


  REQUIRES(ObstacleModel), // opponent corner
  REQUIRES(LibMisc), // opponent corner

  REQUIRES(GameInfo), // own corner
  REQUIRES(LibSpec), // own corenr
  REQUIRES(TeamBallModel), // own corner
  REQUIRES(OwnTeamInfo), // own corner

  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(LibStriker),
  REQUIRES(LibObstacles),
  REQUIRES(PathPlanner),

  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(100) initialWaitTime,
    (int)(7000) ballNotSeenTimeout,
    (int)(4500) time_when_last_seen,
  }),
});

class CornerStrikerCard : public CornerStrikerCardBase
{
  bool preconditions() const override
  {
    return theGameInfo.state == STATE_PLAYING && theGameInfo.setPlay == SET_PLAY_CORNER_KICK;
  }

  bool postconditions() const override
  {
    return theGameInfo.setPlay != SET_PLAY_CORNER_KICK;
  }

  option
  {
    theActivitySkill(BehaviorStatus::Striker);

    initial_state(start)
    {
      transition
      {
        if(state_time > initialWaitTime)
          goto searchForBall;
      }

      action
      {
        theLookForwardSkill();
        theStandSkill();
      }
    }


    state(searchForBall)
    {
      transition
      {
        if(theFieldBall.ballWasSeen() && theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber)
          goto ownCorner;

        else if(theFieldBall.ballWasSeen() && theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber){
          goto opponentCorner;
        }
      }

      action
      {
        theLookForwardSkill();
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
      }
    }


    state(ownCorner){
      transition
      {
        if(theGameInfo.setPlay != SET_PLAY_CORNER_KICK)
          goto searchForBall;
      }
      action
      {
        float radius = 2500;
        std::tuple<float, float> angles = theLibSpec.calcAngleCorner();
        float angle_to_kick = std::get<0>(angles);
        float best_angle = std::get<1>(angles);
      
        if(best_angle<=0.34){ // 20° ToTest 
          //std::cout << best_angle << " --> BASE" << std::endl;
          (theTeamBallModel.position[1] > 0) ? 
            theGoToBallAndKickSkill(calcAngleToTarget(Vector2f(theFieldDimensions.xPosOpponentGroundLine-1000, theFieldDimensions.yPosLeftSideline-1000)), KickInfo::walkForwardsRight, true, 600.0f, true, false): //ToTest the Length
            theGoToBallAndKickSkill(calcAngleToTarget(Vector2f(theFieldDimensions.xPosOpponentGroundLine-1000, theFieldDimensions.yPosRightSideline+1000)), KickInfo::walkForwardsRight, true, 600.0f, true, false); //ToTest the Length
        }
        else{
          //std::cout << best_angle << " --> STEP" << std::endl;
          (theTeamBallModel.position[1] > 0) ? 
            theGoToBallAndKickSkill(calcAngleToTarget(theLibSpec.targetCornerPoint(angle_to_kick, radius)), KickInfo::walkForwardsLeftLong, true, 3000.0f, true, false): //ToTest the length
            theGoToBallAndKickSkill(calcAngleToTarget(theLibSpec.targetCornerPoint(angle_to_kick, radius)), KickInfo::walkForwardsRightLong, true, 3000.0f, true, false); //ToTest the length
        } 
      }
    }


    state(opponentCorner){
      transition
      {
        if(theGameInfo.setPlay != SET_PLAY_CORNER_KICK)
          goto searchForBall;
      }
      action
      {
        bool ballSeen = theFieldBall.ballWasSeen(time_when_last_seen);
        Vector2f target_point = theLibMisc.glob2Rel(theLibStriker.getStrikerPosition(ballSeen).x(), theLibStriker.getStrikerPosition(ballSeen).y()).translation;
        theWalkToPointSkill(target_point);
        theLookAtBallSkill();
        
        /* ToDo
          rotazione striker
        */

      }

    }
  }

  Angle calcAngleToTarget(Vector2f target) const
  {
    return (theRobotPose.inversePose * target).angle();
  }
};

MAKE_CARD(CornerStrikerCard);
