/**
 * @file LibJollyProvider.h
 * 
 * See LibJolly
 *
 * @author Francesco Petri
 */

#pragma once

#include "Representations/Modeling/RobotPose.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibJolly.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Libraries/LibSpec.h"
#include "Representations/Communication/GameInfo.h"   
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Tools/Module/Module.h"


MODULE(LibJollyProvider,
{,
  REQUIRES(RobotPose),
  REQUIRES(FieldDimensions),
  REQUIRES(TeamBallModel),
  REQUIRES(TeamPlayersModel),
  REQUIRES(TeamData),
  REQUIRES(LibMisc),
  REQUIRES(FieldBall),
  REQUIRES(LibSpec),
  REQUIRES(GameInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotInfo),
  PROVIDES(LibJolly),
  // LOADS_PARAMETERS(
  // {,
  //   // nothing for now!
  // }),
});

class LibJollyProvider : public LibJollyProviderBase
{
private:
  
  /**
   * Updates LibJolly
   * @param libJolly The representation provided
   */
  void update(LibJolly& libJolly) override;

  /** 
   * Provides the jolly reference position
   */
  Vector2f getJollyPosition() const;

  /** 
   * Holds the angle between robot's current position and reference jolly position.
   */
  Angle getJollyOrientation(bool ballSeen, Vector2f robotPosition) const;

};
