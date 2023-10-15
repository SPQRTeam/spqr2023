/**
 * @file Whistle.h
 *
 * Identified whistle sound
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Tools/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(Whistle, COMMA public BHumanCompressedMessageParticle<Whistle>
{,
  (float)(0)         confidenceOfLastWhistleDetection, /**< Confidence based on hearing capability. */
  (unsigned char)(0) channelsUsedForWhistleDetection,  /**< Number of channels the robot used to listen. */
  (unsigned int)(0)  lastTimeWhistleDetected,          /**< Timestamp */
  (Vector2f)  lastLocalWhistlePosition,                     /** Position of the last detected whistle (NOTICE: only the direction is reliable) */

  (unsigned)(0) lastTimeGoalDetected,                  /**< Timestamp when a goal was detected. Only the striker has a relevant value here. */
  (unsigned) goalScoredPacketTimeout,                  /**< Exposes the cfg param of the WhistleRecognizer to the others */
});
