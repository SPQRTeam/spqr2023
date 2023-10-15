/**
* @file FreeKickDefendertwoCard.cpp
* @author @neverorfrog
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


CARD(FreeKickDefendertwoCard,
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

class FreeKickDefendertwoCard : public FreeKickDefendertwoCardBase
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
            case GameState::State::opponentCorner:
              goto opponentCorner; return;
            case GameState::State::ownKickIn:
              goto ownKickIn; return;
            case GameState::State::opponentKickIn:
              goto opponentKickIn; return;
            case GameState::State::opponentPushingKick:
              goto opponentPushingKick; return;
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
        Vector2f defendertwoPos = theLibSpec.getPositionCoveringDangerousOpponent(false);
        Pose2f position = theLibMisc.glob2Rel(defendertwoPos.x(), defendertwoPos.y());

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

MAKE_CARD(FreeKickDefendertwoCard);