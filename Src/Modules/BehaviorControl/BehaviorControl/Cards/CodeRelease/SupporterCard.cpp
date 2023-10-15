/**
 * @file SupporterCard.cpp
 *
 * This file implements a basic behavior for the supporter.
 *
 * @author Emanuele Antonioni
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibSupporter.h"

CARD(SupporterCard,
{,
  CALLS(Activity),
  CALLS(GoToBallAndKick),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(WalkToPoint),
  CALLS(WalkAtRelativeSpeed),
  CALLS(LookAtGlobalBall),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(LibMisc),
  REQUIRES(LibSupporter),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(1000) initialWaitTime,
    (int)(7000) ballNotSeenTimeout,
  }),
});

class SupporterCard : public SupporterCardBase
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
    theActivitySkill(BehaviorStatus::Supporter);

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
        //if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
        //  goto searchForBall;
      }

      action
      {
        theLookAtGlobalBallSkill();
        Vector2f target = theLibMisc.glob2Rel(theLibSupporter.getSupporterPosition().x(), theLibSupporter.getSupporterPosition().y()).translation;
        theWalkToPointSkill(target);
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

MAKE_CARD(SupporterCard);
