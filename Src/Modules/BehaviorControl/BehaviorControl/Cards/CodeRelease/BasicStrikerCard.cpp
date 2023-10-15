/**
 * @file BasicStrikerCard.cpp
 *
 * This file implements a basic striker behavior for the striker.
 *
 * @author Emanuele Antonioni
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/BehaviorControl/Libraries/LibStriker.h"
#include "Representations/BehaviorControl/Libraries/LibObstacles.h"
#include "Representations/BehaviorControl/Libraries/LibDefender.h"
#include "Representations/BehaviorControl/PathPlanner.h"
#include "Representations/Modeling/OpponentGoalModel.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Tools/Math/BHMath.h"

#include <iostream>

CARD(BasicStrikerCard,
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
  CALLS(WalkPotentialField),
  CALLS(LookAtGlobalBall),
  CALLS(Say),
  CALLS(WalkToPoint),

  REQUIRES(OpponentGoalModel),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(LibStriker),
  REQUIRES(LibObstacles),
  REQUIRES(ObstacleModel),
  REQUIRES(PathPlanner),
  REQUIRES(BallModel),
  REQUIRES(TeamBallModel),
  USES(LibDefender),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(100) initialWaitTime,
    (int)(7000) ballNotSeenTimeout,
    (float)(80) kickIntervalThreshold,
  }),
});

Vector2f MAGIC_VECTOR = Vector2f(-9999, -9999);

class BasicStrikerCard : public BasicStrikerCardBase
{
  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
    return true;
  }

  Vector2f chosenTarget;
  KickInfo::KickType chosenKickType;
  bool chosenDoItAsap;
  Vector2f ballPositionAtLastTargetChoice;

  bool shouldKick(bool currentlyKicking) {
    float interval = theLibStriker.goalTargetWithArea(false, false).second.interval;
    OUTPUT_TEXT("INTERVAL " << interval);
    return 
      (
        interval > (currentlyKicking ? -1 : kickIntervalThreshold) &&
        theRobotPose.translation.x() > (currentlyKicking ? 0.f : 500.f) &&
        theBallModel.seenPercentage > (currentlyKicking ? 0 : 5)
      ) || (     // always kick if at goal and there are too many obstacles to strategize
        theRobotPose.translation.x() > theFieldDimensions.xPosOpponentPenaltyArea - (currentlyKicking ? 100.f : 0.f) &&
        theObstacleModel.obstacles.size() >= (currentlyKicking ? 2 : 3)
      ) || (    // clear if playing defense
        theRobotPose.translation.x() < theFieldDimensions.xPosOwnGroundLine / 2 + (currentlyKicking ? 400.f : 0.f)
      );
  }

  option
  {
    theActivitySkill(BehaviorStatus::Striker);

    initial_state(start)
    {
      transition
      {
        if(state_time > initialWaitTime)
          goto goToBallAndDribble;
      }

      action
      {
        ballPositionAtLastTargetChoice = MAGIC_VECTOR;
        theLookForwardSkill();
        theStandSkill();
      }
    }

    state(goToBallAndDribble)
    {
      transition
      {
        if(shouldKick(false)){
          goto goToBallAndKick;
        }

        if(!theFieldBall.ballWasSeen(2000))
          goto searchForBall;
      }

      action
      {
        ballPositionAtLastTargetChoice = MAGIC_VECTOR;
        theGoToBallAndDribbleSkill(calcAngleToTarget(theLibStriker.strikerMovementPoint()));
        
      }
    }

    state(goToBallAndKick)
    {
      transition
      {
        if(!shouldKick(true)){
          goto goToBallAndDribble;
        }
      }

      action
      {
        if (!state_time) {
          OUTPUT_TEXT("Kicking");
          theSaySkill("Kicking");
        }

        Vector2f ballRobotv = theFieldBall.positionOnFieldClipped - theRobotPose.translation;

        OUTPUT_TEXT("state_time " << state_time);
        OUTPUT_TEXT("ballRobotv.norm() " << ballRobotv.norm());
        OUTPUT_TEXT("ball disp " << (theFieldBall.positionOnField - ballPositionAtLastTargetChoice).norm());

        if (state_time < 200 || ballRobotv.norm() > 300 || (theFieldBall.positionOnField - ballPositionAtLastTargetChoice).norm() > 500) {
          chosenDoItAsap = theLibObstacles.nearestOpponent().translation.norm() < 750.f;

          Vector2f ballGoalv = theFieldBall.positionOnFieldClipped - Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0);
          ballGoalv = ballGoalv.rotate(-pi_2);

          bool kickRight = ballRobotv.dot(ballGoalv) < 0;

          chosenKickType = theLibStriker.getKick(chosenDoItAsap,kickRight);

          chosenTarget = theLibStriker.goalTarget(chosenDoItAsap, true);

          ballPositionAtLastTargetChoice = theFieldBall.positionOnField;
        }
        else {
          OUTPUT_TEXT("Target locked");
        }

        Angle angle = calcAngleToTarget(chosenTarget);

        theGoToBallAndKickSkill(angle, chosenKickType, !chosenDoItAsap);
      }
    }

    state(searchForBall)
    {
      transition
      {
        if(theFieldBall.ballWasSeen())
          goto goToBallAndDribble;
      }

      action
      {
        theLookForwardSkill();
        theWalkToPointSkill(theFieldBall.recentBallEndPositionRelative(3000));
      }
    }
  }


  

  Angle calcAngleToTarget(Vector2f target) const
  {
    return (theRobotPose.inversePose * target).angle();
  }


};

MAKE_CARD(BasicStrikerCard);
