/**
 * @file LibSupporter.h
 *
 * This file defines a representation that holds some utilities (primarily) for the supporter.
 *
 * @author Francesco Petri
 */

#pragma once

#include "Tools/Function.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(LibSupporter,
{
  /** 
   * Provides the supporter reference position
   */
  FUNCTION(Vector2f()) getSupporterPosition;

  /** 
   * Holds the angle between robot's current position and reference supporter position.
   */
  FUNCTION(Angle(bool ballSeen, Vector2f robotPosition)) getSupporterOrientation,
});
