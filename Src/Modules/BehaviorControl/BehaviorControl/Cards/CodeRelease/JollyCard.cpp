/**
 * @file JollyCard.cpp
 *
 * This file implements a basic behavior for the Jolly.
 *
 * @author Emanuele Antonioni
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibJolly.h"

//CORNER
#include "Representations/Communication/GameInfo.h"   

CARD(JollyCard,
{,
  CALLS(Activity),
  //CALLS(GoToBallAndKick),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(WalkToPoint),
  CALLS(WalkAtRelativeSpeed),
  CALLS(LookAtGlobalBall),
  CALLS(TurnToPoint),

  REQUIRES(GameInfo), 
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(LibMisc),
  REQUIRES(LibJolly),
  REQUIRES(BallModel),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(1000) initialWaitTime,
    (int)(7000) ballNotSeenTimeout,
  }),
});

class JollyCard : public JollyCardBase
{
  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
    return true;
  }

  option
  {
    theActivitySkill(BehaviorStatus::Jolly);

    initial_state(start)
    {
      transition
      {
        if(state_time > initialWaitTime)
          goto followBall;
      }

      action
      {
        theLookForwardSkill();
        theStandSkill();
      }
    }

    state(followBall)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
      }

      action
      {
        Vector2f target = theLibMisc.glob2Rel(theLibJolly.getJollyPosition().x(), theLibJolly.getJollyPosition().y()).translation;
        theLookAtGlobalBallSkill();
        theWalkToPointSkill(target);
        //theTurnToPointSkill(theBallModel.estimate.position);
      }
    }

    state(searchForBall)
    {
      transition
      {
        if(theFieldBall.ballWasSeen())
          goto followBall;
      }

      action
      {
        theLookForwardSkill();
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
      }
    }
  }

};

MAKE_CARD(JollyCard);
