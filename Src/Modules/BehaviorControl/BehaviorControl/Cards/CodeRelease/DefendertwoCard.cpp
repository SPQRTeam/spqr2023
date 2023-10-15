/**
 * @file DefendertwoCard.cpp
 * This file implements a basic behavior for the Defendertwo.
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"
#include "Representations/BehaviorControl/Libraries/LibDefender.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibStriker.h"
#include "Representations/BehaviorControl/Libraries/LibObstacles.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include <Eigen/Geometry>
#include <cmath>

using Line2 = Eigen::Hyperplane<float, 2>;

CARD(DefendertwoCard,
{,
  CALLS(Activity),
  CALLS(GoToBallAndKick),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkPotentialField),
  CALLS(LookAtGlobalBall),
  CALLS(TurnToPoint),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(LibDefender),
  REQUIRES(LibMisc),
  REQUIRES(TeamBallModel),
  REQUIRES(FrameInfo),
  USES(LibStriker),
  USES(LibObstacles),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(1000) initialWaitTime,
    (int)(7000) ballNotSeenTimeout,
  }),
});

class DefendertwoCard : public DefendertwoCardBase
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
    theActivitySkill(BehaviorStatus::Defender);

    initial_state(start)
    {
      transition
      {
        if (state_time > initialWaitTime)
          goto followBall;

        if(theFieldBall.ballWasSeen(ballNotSeenTimeout) || theFrameInfo.getTimeSince(theTeamBallModel.timeWhenLastSeen) < ballNotSeenTimeout)
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
   
      }

      action
      {
        Vector2f target = theLibDefender.getDefendertwoPosition();
        theLookAtGlobalBallSkill();
        theWalkPotentialFieldSkill(target, -1);
        const Vector2f error = target - theRobotPose.translation;
        if (std::abs(error.x()) < 10 && std::abs(error.y()) < 10)
          theStandSkill();
      }
    }
  }
};

MAKE_CARD(DefendertwoCard);