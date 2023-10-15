/**
 * @file LiberoCard.cpp
 *
 * This file implements a basic behavior for the Libero.
 *
 * @author Flavio Volpi
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibLibero.h"

CARD(LiberoCard,
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
  REQUIRES(LibLibero),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(1000) initialWaitTime,
    (int)(7000) ballNotSeenTimeout,
  }),
});

class LiberoCard : public LiberoCardBase
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
    theActivitySkill(BehaviorStatus::Libero);

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
        theLookAtGlobalBallSkill();
        Vector2f target = theLibMisc.glob2Rel(theLibLibero.getLiberoPosition().x(), theLibLibero.getLiberoPosition().y()).translation;
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

MAKE_CARD(LiberoCard);
