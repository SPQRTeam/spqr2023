/**
 * @file LibJolly.h
 *
 * This file defines a representation that holds some utilities (primarily) for the jolly.
 *
 * @author Francesco Petri
 */

#pragma once

#include "Tools/Function.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(LibJolly,
{
  /** 
   * Provides the jolly reference position
   */
  FUNCTION(Vector2f()) getJollyPosition;
  /** 
   * Holds the angle between robot's current position and reference jolly position.
   */
  FUNCTION(Angle(bool ballSeen, Vector2f robotPose)) getJollyOrientation,
});
