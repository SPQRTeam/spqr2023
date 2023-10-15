/**
* @file newCoordinator.cpp
*   This file implements the new team coordination module updated in 2023
* @author Daniele Affinita, Flavio Maiorana
*/

#include "newCoordinator.h"

MAKE_MODULE(NewCoordinator, behaviorControl);

NewCoordinator::NewCoordinator(){
    //Asserts
    ASSERT(priority.size() == utilityMatrix.cols());
    ASSERT(persistence_discount_factor < 1.f);

    //Printing
    cleanFmt = IOFormat(4, 0, ", ", "\n", "[", "]");

    //Persistence and Hysteresis
    discountCycle.start();
    hysteresisCycle.start();
    hysteresisCounter = 0;
    persistence = persistence_weight;
    potentialBestRole = PlayerRole::RoleType::undefined;

    //Context
    current_context = PlayerRole::Context::no_context;
    previous_context = PlayerRole::Context::no_context;
}

Pose2f NewCoordinator::getBestPoseForRole(PlayerRole::RoleType role, Pose2f robotPose){
    Vector2f target_position;
    Angle target_orientation;
    bool ballSeen = theFieldBall.ballWasSeen(); // TODO: set threshold
    switch (role)
    {
        case PlayerRole::RoleType::striker:{
            target_position = theLibStriker.getStrikerPosition(ballSeen);
            target_orientation = theLibStriker.getStrikerOrientation(ballSeen,robotPose.translation);
            return Pose2f(target_orientation,target_position);
            break;
        }

        case PlayerRole::RoleType::defenderone:{
            target_position = theLibDefender.getDefenderonePosition();
            target_orientation = theLibDefender.getDefenderoneOrientation(ballSeen ,robotPose.translation);
            return Pose2f(target_orientation,target_position);
            break;
        }
        
        case PlayerRole::RoleType::defendertwo:{
            target_position = theLibDefender.getDefendertwoPosition();
            target_orientation = theLibDefender.getDefendertwoOrientation(ballSeen, robotPose.translation);
            return Pose2f(target_orientation,target_position);
            break;
        }

        case PlayerRole::RoleType::supporter:{
            target_position = theLibSupporter.getSupporterPosition();
            target_orientation = theLibSupporter.getSupporterOrientation(ballSeen, robotPose.translation);
            return Pose2f(target_orientation,target_position);
            break;
        }
        
        case PlayerRole::RoleType::jolly:{
            target_position = theLibJolly.getJollyPosition();
            target_orientation = theLibJolly.getJollyOrientation(ballSeen, robotPose.translation);
            return Pose2f(target_orientation,target_position);
            break;
        }

        case PlayerRole::RoleType::libero:{
            target_position = theLibLibero.getLiberoPosition();
            target_orientation = theLibLibero.getLiberoOrientation(ballSeen, robotPose.translation);
            return Pose2f(target_orientation,target_position);
            break;
        }
        
        default:{
            FAIL("No suitable position for role "<< TypeRegistry::getEnumName(role) << " found " );
            break;
        }
    }
}

bool NewCoordinator::isRobotPenalized(int r)
{
    if(r+2 == theRobotInfo.number)
        return false;

    for(auto const& teammate : theTeamData.teammates)
        if(teammate.number == r+2)
            return teammate.status == Teammate::PENALIZED || teammate.theRobotPose.translation.x() == magic_value || teammate.theRobotPose.translation.y() == magic_value;

    return false;
}

float NewCoordinator::getPoseComponent(const PlayerRole role, int role_idx, int robot_idx){

    Pose2f tmpRobotPose = role.robots_poses.at(robot_idx);
    Pose2f tmpBestPose = getBestPoseForRole(priority[role_idx], tmpRobotPose);

    float positionError = (tmpBestPose.translation - tmpRobotPose.translation).norm();
    float rotationError = std::abs(tmpBestPose.rotation - tmpRobotPose.rotation);
    rotationError = rotationError > pi ? 2*pi - rotationError : rotationError;

    return distance_weight/(positionError + 1e-10) + angle_weight/(rotationError + 1e-10);
}

float NewCoordinator::getPersistenceComponent(const PlayerRole role, int role_idx, int robot_idx){
    if(discountCycle.elapsedSeconds() >= discounting_time){
        persistence *= persistence_discount_factor;
        discountCycle.start();
    }

    return theRobotInfo.number == robot_idx+2 && role.lastRole == priority[role_idx] ? persistence : 0.f;
}

void NewCoordinator::computeMatrix(const PlayerRole role){
    Index matrixRows = utilityMatrix.rows(),
          matrixCols = utilityMatrix.cols();

    float utility;

    for(int robot_idx = 0; robot_idx < matrixRows; ++robot_idx){
        for(int role_idx = 0; role_idx < matrixCols; ++role_idx){

            if( !isRobotPenalized(robot_idx) ){
                utility = getPoseComponent(role, role_idx, robot_idx)
                        + getPersistenceComponent(role, role_idx, robot_idx);

                utilityMatrix(robot_idx, role_idx) = utility;
            }
            else
                utilityMatrix(robot_idx, role_idx) = -1.f;
            
        }
    }
}

void NewCoordinator::updateRobotPoses(PlayerRole& role){
    if(theRobotInfo.number == 1)
            return;

    for(Teammate mate : theTeamData.teammates){
        if(mate.number == 1)
            continue;

        Pose2f tmp_pose = Pose2f(mate.theRobotPose.rotation,
                                 mate.theRobotPose.translation.x(),
                                 mate.theRobotPose.translation.y());

        role.robots_poses.at(mate.number-2) = tmp_pose;

    }

    role.robots_poses.at(theRobotInfo.number-2) = Pose2f(theRobotPose.rotation,theRobotPose.translation.x(),theRobotPose.translation.y());
}

int NewCoordinator::getBestPlayerForCol(int col){
    Index matrixRows = utilityMatrix.rows();
    int bestPlayer = 0;
    float bestValue = -2.f;

    for(int player_index = 0; player_index < matrixRows; ++player_index){
        if(utilityMatrix(player_index, col) > bestValue){
            bestValue = utilityMatrix(player_index, col);
            bestPlayer = player_index;
        }
    }

    return bestPlayer;
}

void NewCoordinator::removeRow(int row){
    Index matrixCols = utilityMatrix.cols();

    for(int role_index = 0; role_index < matrixCols; ++role_index)
        utilityMatrix(row, role_index) = -1.f;
}


PlayerRole::RoleType NewCoordinator::getBestRole(){
    Index matrixCols = utilityMatrix.cols();
    int bestPlayer;
    for(int role_index = 0; role_index < matrixCols; ++role_index){
        bestPlayer = getBestPlayerForCol(role_index);

        if(bestPlayer+2 == theRobotInfo.number)
            return priority[role_index];
            
        removeRow(bestPlayer);
    }

    return PlayerRole::striker;
}

void NewCoordinator::updatePlayingRole(PlayerRole& role){
    role.lastRole = role.role;

    computeMatrix(role);

    PlayerRole::RoleType best = getBestRole();

    if(best != role.lastRole){
        persistence = persistence_weight;
        discountCycle.start();
    }

    role.role = best;
}

void NewCoordinator::updatePlayingRoleHysteresis(PlayerRole& role){

    //Getting the best role frome the updated matrix
    role.lastRole = role.role;
    computeMatrix(role);
    PlayerRole::RoleType tmpBestRole = getBestRole();

    if(potentialBestRole == PlayerRole::RoleType::undefined){
        role.role = tmpBestRole;
        potentialBestRole = tmpBestRole;
        hysteresisCycle.start();
    }
    else{
        if(potentialBestRole != tmpBestRole)
            ++hysteresisCounter;

        //potential best role is not reliable anymore
        if (hysteresisCounter == hysteresis_rate){
            potentialBestRole = tmpBestRole;
            hysteresisCycle.start();
            hysteresisCounter = 0;
        }

        //after 1 second i effectively change the role
        if (hysteresisCycle.elapsedSeconds() > hysteresis_time || tmpBestRole == PlayerRole::RoleType::striker){

            //discounting factor is zeroed only if i actually change the role
            if(role.role != potentialBestRole){
                persistence = persistence_weight;
                discountCycle.start();
            }

            role.role = potentialBestRole;
            hysteresisCycle.start();
        }
    }
}

void NewCoordinator::updateSearchRole(PlayerRole& role){
    int fartherToOwnGoal = 0;
    role.lastRole = role.role;

    for(int r=0; r < role.robots_poses.size(); ++r){
        if(isRobotPenalized(r) || r == theRobotInfo.number-2)
            continue;
        if(role.robots_poses.at(r).translation.x() > role.robots_poses.at(theRobotInfo.number-2).translation.x())
            ++fartherToOwnGoal;
    }
    
    role.role = fartherToOwnGoal >= 4 ? PlayerRole::passiveSearcher : PlayerRole::activeSearcher;

}

void NewCoordinator::updateFixedRole(PlayerRole& role){
    Index matrixRows = utilityMatrix.rows();
    int role_idx = 0;
    role.lastRole = role.role;

    for(int robot_idx = theRobotInfo.number-1; robot_idx < matrixRows; ++robot_idx)
        if(!isRobotPenalized(robot_idx))
            ++role_idx;

    role.role = priority[role_idx];
}

void NewCoordinator::updateRole(PlayerRole& role){
    if(theRobotInfo.number == 1){
        role.role = PlayerRole::goalie;
        role.lastRole = PlayerRole::goalie;
    }
    else if(theMessageManagement.outOfPackets)
        updateFixedRole(role);
    else if(current_context == PlayerRole::playing)
        updatePlayingRole(role);
    else if(current_context == PlayerRole::search_for_ball && current_context != previous_context)
        updateSearchRole(role);
}

void NewCoordinator::updateContext(PlayerRole& role){

    unsigned timeWhenBallLastSeenByTeammate = 0;
    for (Teammate mate : theTeamData.teammates) {
        unsigned twls = mate.theBallModel.timeWhenLastSeen;
        if (twls > timeWhenBallLastSeenByTeammate)
            timeWhenBallLastSeenByTeammate = twls;
    }

    bool localBallSeen = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < fieldBall_time_threshold;
    bool teamBallSeen = theTeamBallModel.isValid
                        || (unsigned) theFrameInfo.getTimeSince(theTeamBallModel.timeWhenLastSeen) < teamBall_time_threshold
                        || (unsigned) theFrameInfo.getTimeSince(timeWhenBallLastSeenByTeammate) < teamBall_time_threshold;

    previous_context = current_context;

    if(!localBallSeen && !teamBallSeen)
        current_context = PlayerRole::search_for_ball;
    else
        current_context = PlayerRole::playing;

    role.current_context = current_context;
}

void NewCoordinator::update(PlayerRole& role)
{
    if(theGameInfo.state==STATE_PLAYING){
        updateRobotPoses(role);

        updateContext(role);

        updateRole(role);
    }
}