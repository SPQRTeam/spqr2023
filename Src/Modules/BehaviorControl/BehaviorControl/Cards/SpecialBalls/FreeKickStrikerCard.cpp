/**
* @file FreeKickStrikerCard.cpp
* @author Flavio Maiorana, Elisa Santini, Flavio Volpi, Valerio Spagnoli
*/

#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/BehaviorControl/Skills.h"

#include "Representations/Communication/GameInfo.h"
#include "Representations/spqr_representations/GameState.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamBallModel.h"

#include "Representations/BehaviorControl/Libraries/LibStriker.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibSpec.h"

#include <iostream>
#include <cmath>

#include "Tools/Settings.h"


CARD(FreeKickStrikerCard,
{,
  CALLS(Activity),
  CALLS(GoToBallAndKick),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToPoint),
  CALLS(LookAtBall),

  REQUIRES(GameInfo),
  REQUIRES(GameState),
  REQUIRES(FieldBall),  
  REQUIRES(TeamBallModel),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(LibStriker),
  REQUIRES(LibMisc), 
  REQUIRES(LibSpec),  

  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(100) initialWaitTime,
    (int)(7000) ballNotSeenTimeout,
    (int)(4500) time_when_last_seen,
  }),
});

class FreeKickStrikerCard : public FreeKickStrikerCardBase
{
  bool preconditions() const override
  {
     return theGameInfo.state == STATE_PLAYING && 
            theGameInfo.setPlay != SET_PLAY_NONE;
  }

  bool postconditions() const override
  {
     return true;
  }

  option
  {
    theActivitySkill(BehaviorStatus::Striker);

    initial_state(start)
    {
      transition
      {
          switch(theGameState.state)
          {
            case GameState::State::ownCorner:
              goto ownCorner; return;
            case GameState::State::opponentCorner:
              goto opponentCorner; return;
            // case GameState::State::ownKickIn:
            //   goto ownKickIn; return;
            case GameState::State::opponentKickIn:
              goto opponentKickIn; return;
            // case GameState::State::ownPenaltyKick:
            //   goto ownPenaltyKick; return;
            // case GameState::State::opponentPenaltyKick:
            //   goto opponentPenaltyKick; return;
            // case GameState::State::ownPushingKick:
            //   goto ownPushingKick; return;
            // case GameState::State::opponentPushingKick:
            //   goto opponentPushingKick; return;
          }
      }

      action
      {
        theLookForwardSkill();
        theStandSkill();
      }
    }

    state(opponentKickIn)
    {
      transition
      {

      }

      action
      {
        //Returns a position for the striker in a defensive kick in configuration
        Vector2f strikerPos = theLibSpec.getPositionCoveringDangerousOpponent(true);
        Pose2f position = theLibMisc.glob2Rel(strikerPos.x(),strikerPos.y());

        //Robot needs a pose, in order to orient the body towards the ball
        Pose2f targetPose = Pose2f(theLibMisc.angleToBall,position.translation);

        theLookForwardSkill();
        theWalkToPointSkill(targetPose,1.0f);
      }  
    }

    state(ownKickIn)
    {
      transition{

      }

      action{

      }
    }

    state(opponentCorner)
    {
       transition
      {
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

    state(ownCorner)
    {
      transition
      {
        if(theGameInfo.setPlay != SET_PLAY_CORNER_KICK)
          goto start;
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

    state(opponentPenaltyKick)
    {
      transition{

      }

      action{
        
      }
    }

    state(ownPenaltyKick)
    {
      transition{

      }

      action{

      }
    }

    state(opponentPushingKick)
    {
      transition{

      }

      action{
        
      }
    }

    state(ownPushingKick)
    {
      transition{

      }

      action{

      }
    }
  }

  Angle calcAngleToTarget(Vector2f target) const
  {
    return (theRobotPose.inversePose * target).angle();
  }

};

MAKE_CARD(FreeKickStrikerCard);