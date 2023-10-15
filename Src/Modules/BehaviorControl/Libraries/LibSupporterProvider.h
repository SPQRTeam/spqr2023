/**
 * @file LibSupporterProvider.h
 * 
 * See LibSupporter
 *
 * @author Francesco Petri
 */

#pragma once

#include "Representations/Modeling/RobotPose.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Libraries/LibObstacles.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibSupporter.h"
#include "Tools/Module/Module.h"

MODULE(LibSupporterProvider,
{,
  REQUIRES(RobotPose),
  REQUIRES(FieldDimensions),
  REQUIRES(FieldBall),
  REQUIRES(TeamBallModel),
  REQUIRES(TeamData),
  REQUIRES(TeamPlayersModel),
  REQUIRES(RobotInfo),
  REQUIRES(LibObstacles),
  REQUIRES(LibMisc),
  PROVIDES(LibSupporter),
  // LOADS_PARAMETERS(
  // {,
  //   // nothing for now!
  // }),
});

class LibSupporterProvider : public LibSupporterProviderBase
{
private:
  
  /**
   * Updates LibSupporter
   * @param libSupporter The representation provided
   */
  void update(LibSupporter& libSupporter) override;

  /** 
   * Provides the supporter reference position
   */
  Vector2f getSupporterPosition() const;
  
  /** 
   * Holds the angle between robot's current position and reference libero position.
   */
  Angle getSupporterOrientation(bool ballSeen, Vector2f robotPosition) const;

  /**
   * <no doc, guessing>
   * Computes what it says.
   * Unused
   */
  float distanceToLine(Vector2f objectToCheck, Vector2f linePoint1, Vector2f linePoint2) const;

};
