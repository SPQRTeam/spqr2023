/**
 * @file LibLiberoProvider.h
 * 
 * See LibLibero
 *
 * @author Flavio Volpi
 */

#pragma once

#include "Representations/Modeling/RobotPose.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibLibero.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Libraries/LibSpec.h"
#include "Representations/Communication/GameInfo.h"   
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/spqr_representations/Voronoi.h"

#include "Tools/Module/Module.h"


MODULE(LibLiberoProvider,
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
  REQUIRES(Voronoi),
  PROVIDES(LibLibero),
  // LOADS_PARAMETERS(
  // {,
  //   // nothing for now!
  // }),
});

class LibLiberoProvider : public LibLiberoProviderBase
{
private:
  
  /**
   * Updates LibLibero
   * @param libLibero The representation provided
   */
  void update(LibLibero& libLibero) override;

  /** 
   * Provides the libero reference position
   */
  Vector2f getLiberoPosition() const;

  /** 
   * Holds the angle between robot's current position and reference libero position.
   */
  Angle getLiberoOrientation(bool ballSeen, Vector2f target) const;

  //bool compare_nodes(Voronoi::Node n1, Voronoi::Node n2) const;

};
