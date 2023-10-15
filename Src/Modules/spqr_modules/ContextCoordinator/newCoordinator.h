/**
* @file newCoordinator.cpp
*   This file implements the new team coordination module updated in 2023
* @author Daniele Affinita, Flavio Maiorana
*/

#pragma once

#include <iostream>
#include <set>
#include "Tools/Module/Module.h"
#include "Tools/Math/Transformation.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Communication/MessageManagement.h"
#include "Representations/spqr_representations/ConfigurationParameters.h"
#include "Representations/BehaviorControl/PlayerRole.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesFieldPercept.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibDefender.h"
#include "Representations/BehaviorControl/Libraries/LibSupporter.h"
#include "Representations/BehaviorControl/Libraries/LibJolly.h"
#include "Representations/BehaviorControl/Libraries/LibStriker.h"
#include "Representations/BehaviorControl/Libraries/LibLibero.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Tools/Math/Geometry.h"
#include "Tools/SpqrTools/Timer.h"
#include "Tools/Math/Eigen.h"

using namespace Eigen;

#define magic_value -999999

#define printMatrix(m) (std::cout<<"\n"<<std::string(m.cols()*11, '=')<<"\n"<<m.format(cleanFmt)<<"\n"<<std::string(m.cols()*11, '=')<<"\n")

MODULE(NewCoordinator,
{,
REQUIRES(RobotPose),
REQUIRES(RobotInfo),
REQUIRES(TeamData),
REQUIRES(FieldBall),
REQUIRES(FrameInfo),
REQUIRES(BallModel),
REQUIRES(TeamBallModel),
REQUIRES(OwnTeamInfo),
REQUIRES(GameInfo),
REQUIRES(MessageManagement),
REQUIRES(LibStriker),
REQUIRES(LibDefender),
REQUIRES(LibSupporter),
REQUIRES(LibJolly),
REQUIRES(LibLibero),

PROVIDES(PlayerRole),

LOADS_PARAMETERS(
	{,
		(float) distance_weight,
		(float) angle_weight,
		(float) persistence_weight,
		(float) persistence_discount_factor,
		(float) fieldBall_time_threshold,
		(float) teamBall_time_threshold,
		(float) discounting_time, //in seconds
		(float) hysteresis_time,
		(int) hysteresis_rate,
	}),
});

class NewCoordinator: public NewCoordinatorBase
{
private:
	IOFormat cleanFmt;

	std::vector<PlayerRole::RoleType> priority = { PlayerRole::RoleType::striker, PlayerRole::RoleType::defenderone, PlayerRole::RoleType::defendertwo, PlayerRole::RoleType::supporter, PlayerRole::RoleType::jolly, PlayerRole::RoleType::libero };

	/**
	 * Updates my view of the poses of other robots and puts them into role
	*/
	void updateRobotPoses(PlayerRole& role);

	/**
	 * Returns the best pose for RoleType Role, based on the considered robotPose
	*/
	Pose2f getBestPoseForRole(PlayerRole::RoleType role, Pose2f robotPose); 

	// r is the matrix index related to the robot we are looking for
	bool isRobotPenalized(int r);

	void computeMatrix(const PlayerRole role);
	void removeRow(int row);
	int getBestPlayerForCol(int col);
	PlayerRole::RoleType getBestRole();
	
	// Utility Matrix components
	float getPersistenceComponent(const PlayerRole role, int role_idx, int robot_idx);
	float getPoseComponent(const PlayerRole role, int role_idx, int robot_idx);
	
	//Updates
	void updateContext(PlayerRole& role);
	void updateRole(PlayerRole& role);
	void updateFixedRole(PlayerRole& role);
	void updatePlayingRole(PlayerRole& role);
	void updatePlayingRoleHysteresis(PlayerRole& role);
	void updateSearchRole(PlayerRole& role);

	float persistence;
	PlayerRole::RoleType potentialBestRole;


	/**		   STRIKER	 DEF1	DEF2   SUPPORTER	JOLLY   LIBERO
	 * [2-7]
	*/
	Matrix<float, 6, 6> utilityMatrix;

	Timer discountCycle;
	Timer hysteresisCycle;
	int hysteresisCounter;
	
	PlayerRole::Context current_context;
	PlayerRole::Context previous_context;
public:
	NewCoordinator();
    void update(PlayerRole& role);

};
