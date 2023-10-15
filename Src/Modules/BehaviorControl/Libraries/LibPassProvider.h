/**
 * @file LibPassProvider.h
 * 
 * See LibPass
 *
 * @author Francesco Petri
 */

#pragma once

#include "Representations/Modeling/RobotPose.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/BehaviorControl/Libraries/LibPass.h"
#include "Tools/Module/Module.h"

MODULE(LibPassProvider,
{,
  REQUIRES(RobotPose),
  REQUIRES(FieldDimensions),
  REQUIRES(TeamPlayersModel),
  REQUIRES(TeamData),
  PROVIDES(LibPass),
  // LOADS_PARAMETERS(
  // {,
  //   // nothing for now!
  // }),
});

class LibPassProvider : public LibPassProviderBase
{
private:
  
  /**
   * Updates LibPass
   * @param libPass The representation provided
   */
  void update(LibPass& libPass) override;


  // ===== IMPLEMENTATIONS OF LibPass =====

  /**
   * Find a free passing line to the target
   * Returns a valid target or goal center if fails
   */
  Vector2f findPassingLine(Vector2f target, std::vector<Vector2f> mates);

  /**
   * This function orderes the team mates according to their distance w.r.t. the opponents
   * and then, starting from the most free one, tries to find a passing line
   * @return a Vector2f poiting to the passing target or the goal line if no passing is found
   */
  Pose2f poseToPass(LibPass& libPass);     // this gets the libPass parameter b/c it needs to modify something

  /**
   * <no doc was available, easy guess>
   * Returns the passing information of the striker's PassShare.
   * Despite the name's first impression, this is useful to anyone BUT the striker.
   * Most likely the jolly.
   * 
   * @returns The following fields of the striker's PassShare, in this order:
   *          <readyPass, passingTo, passTarget>
   */
  std::tuple<int,int,Pose2f> strikerPassShare() const;


  // ===== FOR INTERNAL USE =====

  /** 
   * An auxiliary for findPassingLine.
   * No other doc was available at the time of this port (2022->2023).
   */
  bool findPassingLineAux(std::vector<Obstacle> opponents, Vector2f& target, const Vector2f& translation);
};
