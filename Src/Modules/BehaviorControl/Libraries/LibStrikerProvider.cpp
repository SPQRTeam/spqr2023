/**
 * @file LibStrikerProvider.cpp
 * 
 * See LibStriker
 *
 * @author Francesco Petri
 */

#include "LibStrikerProvider.h"
#include "Tools/Math/BHMath.h"    // sqr
#include <iostream>
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Debugging/DebugDrawings.h"




MAKE_MODULE(LibStrikerProvider, behaviorControl);

void LibStrikerProvider::update(LibStriker& libStriker)
{
  DECLARE_DEBUG_DRAWING3D("module:LibStrikerProvider:strikerPosition", "field");
  DECLARE_DEBUG_DRAWING("module:LibStrikerProvider:strikerPosition", "drawingOnField");


  libStriker.projectGazeOntoOpponentGroundline = [this]() -> float {
    return projectGazeOntoOpponentGroundline();
  };
  libStriker.computeFreeAreas = [this](float minimumDiscretizedAreaSize) -> std::vector<FreeGoalTargetableArea> {
    return computeFreeAreas(minimumDiscretizedAreaSize);
  };
  libStriker.goalTarget = [this](bool shootASAP, bool forceHeuristic) -> Vector2f {
    return goalTarget(shootASAP, forceHeuristic);
  };
  libStriker.goalTargetWithArea = [this](bool shootASAP, bool forceHeuristic) -> std::pair<Vector2f, FreeGoalTargetableArea> {
    return goalTargetWithArea(shootASAP, forceHeuristic);
  };
  libStriker.strikerPassCommonConditions = [this](int hysteresisSign) -> bool {
    return strikerPassCommonConditions(hysteresisSign);
  };
  libStriker.strikerMovementPoint = [this]() -> Vector2f {
    return strikerMovementPoint();
  };
  libStriker.getApproachSpeed = [this](Rangef range, float kp, float kd, float minSpeed) -> Pose2f {
    return getApproachSpeed(range, kp, kd, minSpeed);
  };
  libStriker.getKick = [this](bool kickAsap, bool kickRight) -> KickInfo::KickType {
    return getKick(kickAsap, kickRight);
  };
  libStriker.getStrikerPosition = [this](bool ballSeen) -> Vector2f {
    return getStrikerPosition(ballSeen);
  };
  libStriker.getStrikerOrientation = [this](bool ballSeen, Vector2f robotPosition) -> Angle {
    return getStrikerOrientation(ballSeen, robotPosition);
  };
}


// TODO maybe rewrite according to better practices (and using Eigen)
float LibStrikerProvider::projectGazeOntoOpponentGroundline() const {
  const float EPS = 1e-6f;
  float rotation = theRobotPose.rotation;
  //To avoid tan singularities
  if(std::abs(rotation) == pi/2) rotation -= EPS;
  return theRobotPose.translation.y() + std::abs(theFieldDimensions.xPosOpponentGoal - theRobotPose.translation.x())*tanf(theRobotPose.rotation);
}

float LibStrikerProvider::projectPointOntoOpponentGroundline(float x, float y) const
{
  return ((theFieldDimensions.xPosOpponentGroundLine - theRobotPose.translation.x())/(x - theRobotPose.translation.x()))*(y - theRobotPose.translation.y()) + theRobotPose.translation.y();
};

bool LibStrikerProvider::areOverlappingSegmentsOnYAxis(float l1, float r1, float l2, float r2) const
{
  return ((l1<l2 && l1>r2) || (r1<l2 && r1>r2) || (l2<l1 && l2>r1) || (r2<l1 && r2>r1));
};

// Note: We are considering only the closest obstacle to the freeArea.
float LibStrikerProvider::areaValueHeuristic(const float leftLimit, const float rightLimit, const float poles_weight, const float opponents_weight, const float teammates_weight) const
{
  // Useful notes for the reader:
  //       leftLimit is the left limit of THIS free area, rightLimit is the right one.
  //       Note that the axis is directed to the left, then leftLimit > righLimit.

  //std::vector<float> opponents_distances;
  //std::vector<float> teammates_distances;
  std::vector<float> distances;
  std::vector<float> weights;


  Pose2f freeAreaPoseleftLimit= Pose2f(theFieldDimensions.xPosOpponentGroundLine,leftLimit);
  Pose2f freeAreaPoserightLimit= Pose2f(theFieldDimensions.xPosOpponentGroundLine,rightLimit);

  float final_utility = 0;

  Obstacle nearest_opponent;
  float nearest_opponent_distance = INFINITY;

  //1) Find nearest opponent
  for(auto obs: theTeamPlayersModel.obstacles)
  {
    //float obs_center_proj = projectPointOntoOpponentGroundline(obs.center.x(),obs.center.y());
    float obs_left_proj = projectPointOntoOpponentGroundline(obs.left.x(),obs.left.y());
    float obs_right_proj = projectPointOntoOpponentGroundline(obs.right.x(),obs.right.y());

    // IT'S NOT POSSIBLE THAT THIS PROJECTION IS INTO SOME FREE_AREAS. Then, just check if it is left or right wrt the obstacle.

    float distance;
    if(obs_right_proj >= leftLimit){ // Obstacle is on the right side of the freeArea
      distance=obs_right_proj-leftLimit;
    }

    if(obs_left_proj<= rightLimit){   // Obstacle is on the left side of the freeArea
      distance=rightLimit-obs_left_proj;
    }

    if(distance < nearest_opponent_distance)
    {
      nearest_opponent = obs;
      nearest_opponent_distance = distance;
    }
  }

  //Add distance from nearest opponent, appropriately weighed
  final_utility = nearest_opponent_distance * theOpponentGoalModel.utilityNearestOpponentWeight;

  //2) Find other distances and weight them appropriately
  for(auto obs : theTeamPlayersModel.obstacles) {

    //Skip the nearest opponent as it is treated differently
    // [2022] TODO compiler says this line has unreachable code? why?
    if(&obs == &nearest_opponent) continue;

    if(theRobotPose.translation.x() > obs.center.x()){
      continue; // if the obstacle is behind me, it's not considered
    }

    float final_distance;
    float final_weight;

    //###########     Considering projection wrt my eyes

    //float obs_center_proj = projectPointOntoOpponentGroundline(obs.center.x(),obs.center.y());
    float obs_left_proj = projectPointOntoOpponentGroundline(obs.left.x(),obs.left.y());
    float obs_right_proj = projectPointOntoOpponentGroundline(obs.right.x(),obs.right.y());

    //Obstacles can be only to the left or to the right of free areas (not inside)

    if(obs_right_proj >= leftLimit){ // Obstacle is on the right side of the freeArea
      final_distance=obs_right_proj-leftLimit;
    }

    if(obs_left_proj<= rightLimit){   // Obstacle is on the left side of the freeArea
      final_distance=rightLimit-obs_left_proj;
    }

    switch(obs.type){
      case Obstacle::Type::opponent:
      {
        //Reweight based on vicinity of the opponent to area midpoint
        Vector2f midpoint(theFieldDimensions.xPosOpponentGroundLine, (leftLimit - rightLimit)/2);
        float opponent_distance_factor = theOpponentGoalModel.utilityOpponentsXDistanceThreshold - theLibMisc.distance(midpoint, obs.center);
        float remapped_opponent_weight = theLibMisc.mapToInterval(opponent_distance_factor, 0.0, theOpponentGoalModel.utilityOpponentsXDistanceThreshold, 0.0, theOpponentGoalModel.utilityOpponentsWeight);
        //std::cout<<"remapped_opponent_weight: "<<remapped_opponent_weight<<std::endl;
        final_weight = remapped_opponent_weight;

        final_utility += final_weight * final_distance;
        break;
      }
      case Obstacle::Type::teammate:
      {
        final_weight = theOpponentGoalModel.utilityTeammatesWeight;
        final_utility += final_weight * final_distance;
        break;
      }
      default:
      {
        final_weight = 0;
        break;
      }
    }

    distances.push_back(final_distance);
    weights.push_back(final_weight);
  }

  float pole_distance_penalty;
  if(std::abs(theFieldDimensions.yPosLeftGoal - leftLimit) > std::abs(theFieldDimensions.yPosRightGoal - rightLimit))
  {
    //nearest_pole_distance = std::abs(theFieldDimensions.yPosRightGoal - rightLimit);
    pole_distance_penalty = std::abs(rightLimit);
  }
  else
  {
    //nearest_pole_distance = std::abs(theFieldDimensions.yPosLeftGoal - leftLimit);
    pole_distance_penalty = std::abs(leftLimit);
  }

  pole_distance_penalty *= theOpponentGoalModel.utilityPolesWeight;
  //std::cout<<"pole_distance_penalty: "<<std::endl;
  //std::cout<<pole_distance_penalty<<std::endl;
  //std::cout<<"final_utility: "<<std::endl;
  //std::cout<<final_utility<<std::endl;

  //return final_utility-pole_distance_penalty;
  //Squared to avoid oscillations
  // [2022] cast to silence warning
  return static_cast<float> (pow(final_utility-pole_distance_penalty, 2));

  //Ideas that can be exploited in order to improve this function:
  /*
  1) First criteria: vicinity to jolly (or in alternative the second robot nearest to the ball, as the first one is the striker)
  2) Second criteria: number of opponent in intercept range
  3) Third criteria: vicinity of enemy goalie (is this possible?)
  4) Width of parent area
  5) Penalize the poles, but this can be done at an higher level of abstraction
  */
};

std::vector<FreeGoalTargetableArea> LibStrikerProvider::computeFreeAreas(float minimumDiscretizedAreaSize) const
{
  /*
  NOTICE:
  begin = leftLimit
  end = rightLimit
  */

  Pose2f myPose = Pose2f(theRobotPose.translation);

  float GOAL_TARGET_OBSTACLE_INFLATION = theOpponentGoalModel.goalTargetObstacleInflation;
  float GOAL_TARGET_AREA_DISCRETIZATION = theOpponentGoalModel.useAreaDiscretization;
  float GOAL_POST_RADIUS = theFieldDimensions.goalPostRadius;
  float GOAL_POLE_MINIMUM_TARGET_OFFSET = theOpponentGoalModel.goalPoleMinimumTargetOffset;

  float GOAL_LINE_LEFT_LIMIT = theFieldDimensions.yPosLeftGoal - GOAL_POST_RADIUS - GOAL_POLE_MINIMUM_TARGET_OFFSET;
  float GOAL_LINE_RIGHT_LIMIT = theFieldDimensions.yPosRightGoal + GOAL_POST_RADIUS + GOAL_POLE_MINIMUM_TARGET_OFFSET;

  //vector of opponents
  std::vector<Obstacle> opponents;
  //vector of poles
  std::vector<Obstacle> poles;

  for(auto obs : theTeamPlayersModel.obstacles){
    /*NOTICE: poles are added statically (a priori) by the vision system
      so the 4 goal poles will always be in the obstacle list*/
    switch(obs.type)
    {
      case Obstacle::Type::goalpost:
      {
        if(obs.type==Obstacle::Type::goalpost && obs.center.x()>0)
        {
          //std::cout<<"Found opponent goal post: (left:"<<obs.left.y()<<", right:"<<obs.right.y()<<")\n";
          poles.push_back(obs);
        }
        break;
      }
      default:
      {
        if(obs.center.x()>theRobotPose.translation.x() || obs.left.x()>theRobotPose.translation.x() || obs.right.x()>theRobotPose.translation.x())
        {
          opponents.push_back(obs);
        }
        break;
      }
    }
  }

  Pose2f leftPole, rightPole;

  if(poles.size()==0)
  {
    leftPole.translation.x() = (float)theFieldDimensions.xPosOpponentGroundLine;
    leftPole.translation.y() = projectPointOntoOpponentGroundline(leftPole.translation.x(),730.);
    if(leftPole.translation.y()>GOAL_LINE_LEFT_LIMIT) leftPole.translation.y()=GOAL_LINE_LEFT_LIMIT;
    rightPole.translation.x() = (float)theFieldDimensions.xPosOpponentGroundLine;
    rightPole.translation.y() = projectPointOntoOpponentGroundline(rightPole.translation.x(),-730.);
    if(rightPole.translation.y()<GOAL_LINE_RIGHT_LIMIT) leftPole.translation.y()=GOAL_LINE_RIGHT_LIMIT;
  }
  else
  {

    leftPole.translation.x() = poles.at(0).right.x();
    if(leftPole.translation.x()>(float)theFieldDimensions.xPosOpponentGroundLine) leftPole.translation.x()=(float)theFieldDimensions.xPosOpponentGroundLine;

    leftPole.translation.y() = projectPointOntoOpponentGroundline(leftPole.translation.x(),poles.at(0).right.y());
    if(leftPole.translation.y()>GOAL_LINE_LEFT_LIMIT) leftPole.translation.y()=GOAL_LINE_LEFT_LIMIT;

    rightPole.translation.x() = poles.at(1).left.x();
    if(rightPole.translation.x()>(float)theFieldDimensions.xPosOpponentGroundLine) rightPole.translation.x()=(float)theFieldDimensions.xPosOpponentGroundLine;

    rightPole.translation.y() = projectPointOntoOpponentGroundline(rightPole.translation.x(),poles.at(1).left.y());
    if(rightPole.translation.y()<GOAL_LINE_RIGHT_LIMIT) rightPole.translation.y()=GOAL_LINE_RIGHT_LIMIT;
  }

  //FREE AREAS COMPUTATION

  std::vector<float> leftPoints;
  std::vector<float> rightPoints;
  std::vector<FreeGoalTargetableArea> freeAreas;


  Obstacle swapper;

  /*1) Sort the opponents in vector based on the y coordinate of their left points
  (for each obstacle, the leftmost point I see)*/
  for(int i = 0; i < opponents.size(); i++){
    for(int k = 0; k < opponents.size(); k++){
      float firstLeft = projectPointOntoOpponentGroundline(opponents.at(i).left.x(),opponents.at(i).left.y());
      float secondLeft = projectPointOntoOpponentGroundline(opponents.at(k).left.x(),opponents.at(k).left.y());
      if(firstLeft > secondLeft ){
        swapper = opponents.at(k);
        opponents.at(k) = opponents.at(i);
        opponents.at(i) = swapper;
      }
    }
  }

  /*2) Find overlapping obstacles and merge them into a single one. For each obstacle (including the
    ones obtained by merging) save the projections on the goal line of its left and right point, in order
    to populate a list of obstacles from the player's point of view.

    NOTICE: the obstacles are ordered from left to right

    NOTICE: GOAL_TARGET_OBSTACLE_INFLATION is used as an "obstacle inflation" factor
    to enlarge obstacles (as the visualization system makes the opponent robots smaller than their
    feet width
  */

  if(opponents.size()>0)
  {
    float leftPointY = projectPointOntoOpponentGroundline(opponents.at(0).left.x(),opponents.at(0).left.y())+GOAL_TARGET_OBSTACLE_INFLATION*1000/theLibMisc.distance(theRobotPose,opponents.at(0).left);
    float rightPointY = projectPointOntoOpponentGroundline(opponents.at(0).right.x(),opponents.at(0).right.y())-GOAL_TARGET_OBSTACLE_INFLATION*1000/theLibMisc.distance(theRobotPose,opponents.at(0).right);
    if(opponents.size()==1)
    {
      //If the obstacle projection is at least partially inside the goal line add it
      if(areOverlappingSegmentsOnYAxis(leftPointY,rightPointY,leftPole.translation.y(),rightPole.translation.y()))
      {
        //std::cout<<"1 Adding single obstacle: ("+std::to_string(leftPointY)+","+std::to_string(rightPointY)+")\n";
        leftPoints.push_back(leftPointY);
        rightPoints.push_back(rightPointY);
      }
    }
    else if(opponents.size()>1)
    {
      int i=1;
      bool wereOverlapping;

      /*
        For each obstacle in the list, compare it against the next one to check for overlapping and,
        depending on the kind of overlapping, merge them accordingly or add them separately
      */
      while(i<opponents.size()+1)
      {
        float nextLeftPointY, nextRightPointY;
        if(i==opponents.size())
        {
          /*
            If leftPoint and rightPoint identify the last obstacle in the opponents list, use
            the right pole as the next obstacle to compare for overlapping
          */
          nextLeftPointY = projectPointOntoOpponentGroundline(poles.at(1).left.x(),poles.at(1).left.y());
          nextRightPointY = projectPointOntoOpponentGroundline(poles.at(1).right.x(),poles.at(1).right.y());
        }
        else
        {
          /*
            Else use the next obstacle in the list
          */
          nextLeftPointY = projectPointOntoOpponentGroundline(opponents.at(i).left.x(),opponents.at(i).left.y())+GOAL_TARGET_OBSTACLE_INFLATION*1000/theLibMisc.distance(theRobotPose,opponents.at(i).left);
          nextRightPointY = projectPointOntoOpponentGroundline(opponents.at(i).right.x(),opponents.at(i).right.y())-GOAL_TARGET_OBSTACLE_INFLATION*1000/theLibMisc.distance(theRobotPose,opponents.at(i).right);
        }

        /*
          Check for overlapping: there are three cases to manage
          1) One obstacle is inside the other: do nothing
          2) Obstacles overlap only partially: merge the obstacles
          3) No overlap: add the first obstacle and pass to the second one
        */
        if(areOverlappingSegmentsOnYAxis(leftPointY, rightPointY, nextLeftPointY, nextRightPointY))
        {

          if(leftPointY>nextLeftPointY && rightPointY<nextRightPointY)
          {
            //1) One obstacle is inside the other: do nothing

            //std::cout<<"CASE 1\n";
            //std::cout<<"Current obstacle: ("+std::to_string(leftPointY)+","+std::to_string(rightPointY)+")\n";
          }
          else
          {
            //2) Obstacles overlap only partially: merge the obstacles

            //std::cout<<"CASE 2\n";
            rightPointY = nextRightPointY;

            //std::cout<<"Current obstacle: ("+std::to_string(leftPointY)+","+std::to_string(rightPointY)+")\n";

          }
          wereOverlapping=true;
        }
        else
        {
          //3) No overlap: add the first obstacle and pass to the second one
          if(areOverlappingSegmentsOnYAxis(leftPointY,rightPointY,leftPole.translation.y(),rightPole.translation.y()))
          {
            //Add the obstacle projection only if it falls (even only partially) inside the goal line
            //std::cout<<"2 Adding obstacle: ("+std::to_string(leftPointY)+","+std::to_string(rightPointY)+")\n";
            leftPoints.push_back(leftPointY);
            rightPoints.push_back(rightPointY);
          }
          leftPointY = nextLeftPointY;
          rightPointY = nextRightPointY;
          wereOverlapping=false;
        }

        i++;
      }

      //Add last remaining obstacle points (only in case where it overlaps right pole)
      if(wereOverlapping)
      {
        if(areOverlappingSegmentsOnYAxis(leftPointY,rightPointY,leftPole.translation.y(),rightPole.translation.y()))
        {
          //Add the obstacle projection only if it falls (even only partially) inside the goal line
          //std::cout<<"3 Adding obstacle: ("+std::to_string(leftPointY)+","+std::to_string(rightPointY)+")\n";
          leftPoints.push_back(leftPointY);
          rightPoints.push_back(rightPointY);
        }
      }
    }
  }

  //std::cout<<"End of overlap check\n\n";

  /*3) Now that we have left and right points of the obstacles
    3.1) We first determine if there is any free area (or if a single obstacle
          projection overlaps the whole goal line)
    3.2) We shrink the left and right limit (begin/end) of the goal line if there are
          obstacles overlapping the left/right poles
  */
  //determines if all obstacles are outside goal
  bool noneInside = true;

  //Consider my angle in inflating the pole
  float begin = leftPole.translation.y();
  float end = rightPole.translation.y();

  for(int i = 0; i < leftPoints.size(); i++){

    //3.1)
    //at least one point inside goal
    if(leftPoints.at(i)<leftPole.translation.y() && rightPoints.at(i)>rightPole.translation.y() ||
    leftPoints.at(i)>leftPole.translation.y() && rightPoints.at(i)<leftPole.translation.y() ||
    leftPoints.at(i)>rightPole.translation.y() && rightPoints.at(i)<rightPole.translation.y())
    {
      noneInside = false;
    }
    //WATCH OUT, TEMPORARY SOLUTION: If an obstacle projection cover the whole goal area I return an empty area list
    if(leftPoints.at(i)>leftPole.translation.y() && rightPoints.at(i) < rightPole.translation.y())
    {
      return freeAreas;
    }

    //3.2)
    //left obstacle border outside goal, right border inside
    if(leftPoints.at(i) > leftPole.translation.y() && rightPoints.at(i) < leftPole.translation.y()){
      begin = rightPoints.at(i);
    }
    //right obstacle border outside goal, left border inside
    if(leftPoints.at(i) > rightPole.translation.y() && rightPoints.at(i) < rightPole.translation.y()){
      end = leftPoints.at(i);
    }
  }

  /*4) Build the free targetable segments vector
    4.1) If there is no targetable segment, return the empty vector
    4.2) Else populate the freeAreas vector
  */

  //4.1)
  if(noneInside == true){
    freeAreas.push_back(FreeGoalTargetableArea(begin, end, 1));
    //std::cout<<"NONE INSIDE\n";
    return freeAreas;
  }
  std::vector<float> freeAreasPoints;
  freeAreasPoints.push_back(begin);

  //4.2)
  for(int i = 0; i < leftPoints.size(); i++){
    if(leftPoints.at(i) < begin && leftPoints.at(i) > end){
      freeAreasPoints.push_back(leftPoints.at(i));
    }
    if(rightPoints.at(i) < begin && rightPoints.at(i) > end ){
      freeAreasPoints.push_back(rightPoints.at(i));
    }
  }
  freeAreasPoints.push_back(end);

  sort(freeAreasPoints.begin(),freeAreasPoints.end());

  /*5A) Apply discretization to the targetable segments, by dividing targetable areas in smaller segments,
      in order to assign them a utility value
  */

  if(GOAL_TARGET_AREA_DISCRETIZATION)
  {
    for(int i = freeAreasPoints.size()-1; i-1>=0; i-=2)
    {
      float beginning = freeAreasPoints.at(i);
      float end = freeAreasPoints.at(i-1);
      float size = std::abs(end-beginning);
      if(size>minimumDiscretizedAreaSize)
      {
        int numberOfDiscretizedAreasInFreeArea = floor(size/minimumDiscretizedAreaSize);

        float discretizedAreaSize = size/numberOfDiscretizedAreasInFreeArea;

        float firstPoint = beginning;
        for(int i = numberOfDiscretizedAreasInFreeArea -1; i>0; i--)
        {
          float lastPoint = firstPoint-discretizedAreaSize;
          freeAreas.push_back(FreeGoalTargetableArea(firstPoint,lastPoint,areaValueHeuristic(firstPoint,lastPoint)));
          firstPoint = lastPoint;
        }
          freeAreas.push_back(FreeGoalTargetableArea(firstPoint,end,areaValueHeuristic(firstPoint,end)));
      }
      else
      {
        continue;
      }

    }

    /*
      6) Sort the result of the discretization on its utility value
    */

    //Sort the freeAreas vector in a decreasing order of utility values
    sort(freeAreas.begin(),freeAreas.end(), [](const FreeGoalTargetableArea &x, const FreeGoalTargetableArea &y){ return (x.value > y.value);});
  }
  else
  {
    /*
      5B) Do not discretize free areas: the whole area between opponent projections is used:
          might be imprecise and less effective and also will require longer times to align with the ball
    */
    for(int i = freeAreasPoints.size() -1; i-1 >= 0; i-=2)
    {
      freeAreas.push_back(FreeGoalTargetableArea(freeAreasPoints.at(i), freeAreasPoints.at(i-1), areaValueHeuristic(freeAreasPoints.at(i),freeAreasPoints.at(i-1))));
    }
    sort(freeAreas.begin(),freeAreas.end(), [](const FreeGoalTargetableArea &x, const FreeGoalTargetableArea &y){ return (x.value > y.value);});
  }

  return freeAreas;
}

Vector2f LibStrikerProvider::goalTarget(bool shootASAP, bool forceHeuristic) const {
  return goalTargetWithArea(shootASAP, forceHeuristic).first;
}

/*
@author Emanuele Musumeci (original goalTarget), Francesco Petri, Fabian Fonseca (adaptation to return the area as well)
Return a Vector2f containing the chosen target and the associated FreeGoalTargetableArea based on the selected mode. There are two modes:
1) the default mode (shootASAP=false) decides whether
    A) to shoot to the nearest possible target (this happens if the robot
    has a distance from the opponent goal less or equal than GOAL_TARGET_DISTANCE_THRESHOLD)
    B) to choose the best target based on its utility value
2) the shootASAP mode (shootASAP=true) instead forces shooting to the nearest possible target
*/
//TODO: Move constant parameters to CFG
std::pair<Vector2f, FreeGoalTargetableArea> LibStrikerProvider::goalTargetWithArea(bool shootASAP, bool forceHeuristic) const
{
  float GOAL_TARGET_AREA_MIN_SIZE = theOpponentGoalModel.goalTargetAreaMinSize;
  float GOAL_TARGET_MIN_OFFSET_FROM_SIDE = theOpponentGoalModel.goalTargetMinOffsetFromSide;
  float GOAL_TARGET_DISTANCE_THRESHOLD = theOpponentGoalModel.goalTargetDistanceThreshold;

  std::vector<FreeGoalTargetableArea> freeAreas = computeFreeAreas(GOAL_TARGET_AREA_MIN_SIZE);

  /*
    1) Filter free areas, ignoring the smaller ones
  */
  std::vector<FreeGoalTargetableArea> filteredFreeAreas;
  for(const auto& area : freeAreas)
  {
    if(area.interval>=GOAL_TARGET_AREA_MIN_SIZE)
    {
      filteredFreeAreas.push_back(area);
      //std::cout << ("[Robot #"+std::to_string(theRobotInfo.number)+"] FreeGoalTargetableArea: ("+std::to_string(area.begin)+","+std::to_string(area.end)+")") << "\n";
    }
  }

  std::pair<Vector2f, FreeGoalTargetableArea> targetAndArea;

  /*
    2) If there is no free area, return the median point on the opponent goal line
  */
  //WATCH OUT: SPECIAL RETURN AS AN AREA WITH 0 LENGTH AND NEGATIVE UTILITY, TO MANAGE IN STRIKER BEHAVIOR
  if(filteredFreeAreas.size()==0)
  {
    targetAndArea.first = Vector2f(theFieldDimensions.xPosOpponentGroundLine,0);
    targetAndArea.second = FreeGoalTargetableArea(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.xPosOpponentGroundLine, -1000);
    return targetAndArea;
  }

  //project my line of view onto the goal line
  float myGazeProjection = projectGazeOntoOpponentGroundline();

  float targetPoint = 0;
  FreeGoalTargetableArea areaAssociatedToTargetPoint;

  float minTargetableAreaDistance=INFINITY;

  //CASE 1: I'm near the goal post -> choose the nearest free area
  if(!forceHeuristic && 
      (shootASAP || (std::abs(theFieldDimensions.xPosOpponentGroundLine-theRobotPose.translation.x())<GOAL_TARGET_DISTANCE_THRESHOLD && filteredFreeAreas.size()!=0) || filteredFreeAreas.size()==1))
  {
    for(int i = 0; i < filteredFreeAreas.size(); i++)
    {
      FreeGoalTargetableArea currentArea = filteredFreeAreas.at(i);
      //CASE 1.1: Looking at free area directly
      if(myGazeProjection<currentArea.begin && myGazeProjection>currentArea.end)
      {
        if(myGazeProjection>currentArea.begin-GOAL_TARGET_MIN_OFFSET_FROM_SIDE) targetPoint = currentArea.begin-GOAL_TARGET_MIN_OFFSET_FROM_SIDE;
        else if(myGazeProjection<currentArea.end+GOAL_TARGET_MIN_OFFSET_FROM_SIDE) targetPoint = currentArea.end+GOAL_TARGET_MIN_OFFSET_FROM_SIDE;
        else targetPoint = myGazeProjection;
        areaAssociatedToTargetPoint = currentArea;
        break;
      }
      //CASE 1.2: Looking away from free area
      else
      {
        //if freeArea is on the left
        if(currentArea.begin<myGazeProjection)
        {
          float currentFreeAreaDistance=myGazeProjection-currentArea.begin;
          if(minTargetableAreaDistance>currentFreeAreaDistance)
          {
            minTargetableAreaDistance=currentFreeAreaDistance;
            targetPoint= currentArea.begin-GOAL_TARGET_MIN_OFFSET_FROM_SIDE;
            areaAssociatedToTargetPoint = currentArea;
          }
        }
        //if freeArea is on the right
        else if(currentArea.end>myGazeProjection)
        {
          float currentFreeAreaDistance=currentArea.end-myGazeProjection;
          if(minTargetableAreaDistance>currentFreeAreaDistance)
          {
            minTargetableAreaDistance=currentFreeAreaDistance;
            targetPoint= currentArea.end+GOAL_TARGET_MIN_OFFSET_FROM_SIDE;
            areaAssociatedToTargetPoint = currentArea;
          }
        }
      }
    }
  }
  //CASE 2: I'm far away from the goal post -> find the area with the highest utility
  else
  {
    //The freeAreas vector is sorted in a decreasing order of utility values, so I select the first area    

    //Use whole area
    targetPoint = filteredFreeAreas.at(0).midpoint;
    areaAssociatedToTargetPoint = filteredFreeAreas.at(0);
  }

  //If I'm too near to the goal line, shoot inside the goal, else shoot on the goal line
  Vector2f target;
  
  //EMANUELE M. changed the threshold a bit, feel free to change back (was 600, is 1000) (also note this change was made prior to 2022)
  if (theRobotPose.translation.x() >= (theFieldDimensions.xPosOpponentGroundLine - 1000.f) &&
          std::abs(theRobotPose.translation.y()) < 500.f )
    target = Vector2f(theFieldDimensions.xPosOpponentGroundLine + 1000.f, targetPoint);
  else
    target = Vector2f(theFieldDimensions.xPosOpponentGroundLine, targetPoint);
  
  targetAndArea.first = target;
  targetAndArea.second = areaAssociatedToTargetPoint;
  return targetAndArea;
}

float LibStrikerProvider::sqrDistanceOfClosestOpponentToPoint(const Vector2f& p) const {
  //initializing the minimum distance to a value greater than the diagonal of the field
  //(therefore greater than any distance possible in the game)
  float minSqrDist = sqr(2*theFieldDimensions.xPosOpponentFieldBorder + 2*theFieldDimensions.yPosLeftFieldBorder);
  //simply loop over all opponents...
  for(const Obstacle& opp : theTeamPlayersModel.obstacles) {
    if (opp.type == Obstacle::opponent) {
      float sqrdist = (p - opp.center).squaredNorm();
      //...and update the minimum distance, that's it
      if (sqrdist < minSqrDist) {
        minSqrDist = sqrdist;
      }
    }
  }
  return minSqrDist;
}

Vector2f LibStrikerProvider::strikerMovementPoint() const{
  std::vector<Vector2f> movepoints;
  float steplen = 1000.f;
  movepoints.push_back(Vector2f(theFieldBall.recentBallPositionOnField().x() + steplen, theFieldBall.recentBallPositionOnField().y())); //center move
  movepoints.push_back(Vector2f(theFieldBall.recentBallPositionOnField().x() + (steplen * cos(45)), theFieldBall.recentBallPositionOnField().y() - (steplen * sin(45)))); //right move
  movepoints.push_back(Vector2f(theFieldBall.recentBallPositionOnField().x() + (steplen * cos(45)), theFieldBall.recentBallPositionOnField().y() + (steplen * sin(45)))); //left move
  movepoints.push_back(Vector2f(theFieldBall.recentBallPositionOnField().x() + (steplen * cos(80)), theFieldBall.recentBallPositionOnField().y() - (steplen * sin(80)))); //very right move
  movepoints.push_back(Vector2f(theFieldBall.recentBallPositionOnField().x() + (steplen * cos(80)), theFieldBall.recentBallPositionOnField().y() + (steplen * sin(80)))); //very left move

  float ballX = theFieldBall.recentBallPositionOnField().x();
  float ballY = theFieldBall.recentBallPositionOnField().y();
  float k1 = 1.5, k2 = 1.25;
  
  float scores[5];
  for(int i = 0; i < movepoints.size(); i++){
    scores[i] = 0;
    //Penalize points by distance from the goal
    scores[i] -= Vector2f(theFieldDimensions.xPosOpponentGroundLine - movepoints.at(i).x(), 0 - movepoints.at(i).y()).norm() * k1;
    //Penalize points if they are out of the field's bounds
    if((std::abs(movepoints.at(i).y()) > theFieldDimensions.yPosLeftSideline) || 
    (std::abs(movepoints.at(i).x()) > theFieldDimensions.xPosOpponentGroundLine) ){
      scores[i] -= 100000.f;
    }
    for(auto ob : theTeamPlayersModel.obstacles){
      //Check obstacles only if they are ahead and in a radius of 1300 mm from the robot
      if(ob.center.x() > ballX){
        if(Vector2f(ob.center.x() - ballX, ob.center.y() - ballY).norm() < 1300.f){
          //reward points the more they are far from the obstacles
           scores[i] +=  Vector2f(ob.center.x() - movepoints.at(i).x(), ob.center.y() - movepoints.at(i).y()).norm() * k2;
        }
      }
    }
    
  }//end of point assignement for

  float maxScore = -10000000.f;
  int index = 0;
  for(int i = 0; i < 5; i++){
    if(maxScore < scores[i]){
      maxScore = scores[i];
      index = i;
    }
  }
    
  return movepoints.at(index);
}

bool LibStrikerProvider::strikerPassCommonConditions(int hysteresisSign) const {
  //the conditions branch depending on whether the striker
  //is under immediate pressure to send the ball *somewhere*.
  //This is true if the striker is close to the ball
  //and either an opponent is immediately adjacent to the striker,
  //    or the striker is close to the goal.
  //in these conditions, we don't want the striker to take too much time
  //walking around the ball.
  //TODO this will have to change when we have contrast
  bool act_asap = (
    theFieldBall.positionRelative.squaredNorm()<sqr(300.f) && (
      theRobotPose.translation.x() > theFieldDimensions.xPosOpponentPenaltyMark ||
      sqrDistanceOfClosestOpponentToPoint(theRobotPose.translation)<sqr(500.0f)
    )
  );
  if (act_asap) {
    //if the striker is pressed to act, then the choice between kick or pass
    //depends solely on which action is faster to execute in the immediate future
    //(remember that if the control flow gets here, there *is* an available pass).
    //Thus, choose to pass if it's easier than kicking,
    //measured in terms of how much walk-around-the-ball the striker needs
    //(because we've already assume the striker is close to the ball)
    Pose2f goal_target = goalTarget(act_asap, false);
    Angle kickAngle = Angle(atan2f(
      goal_target.translation.y()-theRobotPose.translation.y(),
      goal_target.translation.x()-theRobotPose.translation.x()
    ));
    Angle passAngle = Angle(atan2f(
      thePassShare.passTarget.translation.y()-theRobotPose.translation.y(),
      thePassShare.passTarget.translation.x()-theRobotPose.translation.x()
    ));
    Angle angDistToKick = theRobotPose.rotation.diffAbs(kickAngle);
    Angle angDistToPass = theRobotPose.rotation.diffAbs(passAngle);
    // adding a constant epsilon in order to make the striker prefer passing
    // if the two angles are practically equal
    bool shouldPass = (angDistToPass <= angDistToKick + pi/30 + hysteresisSign*pi/10);
    /*
    if (shouldPass) {
      std::cout << "Act ASAP: pass" << '\n';
    }
    else {
      std::cout << "Act ASAP: carry" << '\n';
    }
    */
    return shouldPass;
  }
  else {
    //if the striker is not under immediate pressure,
    //it can actually reason and choose whether to kick or pass.
    //for now, the only discriminating condition in this case is:
    //only pass if the passtarget is sufficiently clear of opponents
    //(the pass routines only find the target w/ highest opp distance, but don't set a minimum threshold)
    bool shouldPass = (sqrDistanceOfClosestOpponentToPoint(thePassShare.passTarget.translation) >= sqr(500.0f - hysteresisSign*100.0f));
    /*
    if (shouldPass) {
      std::cout << "Act normal: pass" << '\n';
    }
    else {
      std::cout << "Act normal: carry" << '\n';
    }
    */
    return shouldPass;
  }

  
}

/**
 * PD controller that returns the approaching speed.
 * @param range The distance range taken to pass from speed 1 to the specified minimum speed
 * @param kp The controller proportional gain
 * @param kd The controller derivative gain
 * @param minSpeed The minimum speed allowed 
 * @return speed in Pose2f in a range from 1 to minSpeed 
* **/
Pose2f LibStrikerProvider::getApproachSpeed(Rangef range, float kp, float kd, float minSpeed) const{
  if(theFieldBall.recentBallPositionRelative().norm() > range.max)
    return Pose2f(1,1,1);

  if(kd>kp)
    OUTPUT_WARNING("Derivative gain is greater than proportional gain, this may lead to an extremely slow convergence");

  if(kd == 0)
    OUTPUT_WARNING("Derivative gain is set to 0, this may lead a to lack of convergence due to rippling");

  Vector2f target = goalTarget(true, true);

  float e_angle = abs((theRobotPose.inversePose * target).angle()),
        e_p = mapToRange(theFieldBall.recentBallPositionRelative().norm()+e_angle  , range.min, range.max, minSpeed, 1.f),
        e_d = theMotionInfo.speed.translation.norm()/1e3,
        out = clip(kp*e_p - kd*e_d, minSpeed, 1.f);

  return Pose2f(out,out,out);
}

/**
 * @param kickAsap a bool attribute, is true when you have to kick as soon as possible
 * @param kickRight a bool attribute, is true when you want to kick with right foot
 * @return The best kickType chosen according to the opponent goal ditance
* **/
KickInfo::KickType LibStrikerProvider::getKick(bool kickAsap, bool kickRight) const{
  //Rangef 
  std::array<Rangef, 3> rgs = {Rangef(0.f, 2000.f), Rangef(2000.f, INFINITY), Rangef(INFINITY, INFINITY)};

  // The first element of the pair is the case of kickAsap false, the second one is kickAsap true
  std::array<std::pair<KickInfo::KickType,KickInfo::KickType>, 3> kicksRight {std::make_pair(KickInfo::forwardFastRight, KickInfo::walkForwardsRight), std::make_pair(KickInfo::forwardFastRightLong, KickInfo::walkForwardsRightLong), std::make_pair(KickInfo::ottoMetriRightKick,KickInfo::ottoMetriLooseRightKick)};
  std::array<std::pair<KickInfo::KickType,KickInfo::KickType>, 3> kicksLeft {std::make_pair(KickInfo::forwardFastLeft, KickInfo::walkForwardsLeft), std::make_pair(KickInfo::forwardFastLeftLong, KickInfo::walkForwardsLeftLong), std::make_pair(KickInfo::ottoMetriLeftKick,KickInfo::ottoMetriLooseLeftKick)};

  ASSERT(rgs.size() == kicksRight.size());
  ASSERT(kicksLeft.size() == kicksRight.size());

  std::array<std::pair<KickInfo::KickType,KickInfo::KickType>, 3> kicks = kickRight ? kicksRight : kicksLeft;
  Vector2f distVector = theRobotPose.translation - Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0);
  float dist = distVector.norm();

  for(int i = 0; i < rgs.size(); ++i)
    if(rgs[i].isInside(dist))
      return kickAsap ? kicks[i].second : kicks[i].first;
  
  OUTPUT_ERROR("getKick function failed, no suitable kick found");
}


Vector2f LibStrikerProvider::getStrikerPosition(bool ballSeen) const{
  Vector2f target;
  Vector2f bestBall = ballSeen ? theFieldBall.recentBallPositionOnField() : theTeamBallModel.position;
  // opponent corner kick
  if(theGameInfo.state==STATE_PLAYING && theGameInfo.setPlay == SET_PLAY_CORNER_KICK && theGameInfo.kickingTeam == theOpponentTeamInfo.teamNumber){
    bool defenderone = false;
    bool defendertwo = false;

    for(auto teammate:theTeamData.teammates){
      if(teammate.role==PlayerRole::defenderone){
        defenderone=true;
      }
      if(teammate.role==PlayerRole::defendertwo){
        defendertwo=true;
      }
    }

    if(!defenderone)
      target = theLibDefender.getDefenderonePosition();
    else if(!defendertwo)
      target = theLibDefender.getDefendertwoPosition();
    else
      if(theTeamBallModel.position.y() > 0) //left corner
        target = Vector2f(theFieldDimensions.xPosOwnGroundLine+1100, theFieldDimensions.yPosLeftSideline-750);
      else
        target = Vector2f(theFieldDimensions.xPosOwnGroundLine+1100, theFieldDimensions.yPosRightSideline+750);
  }
  else{
    target = bestBall;
  }

  CIRCLE("module:LibDefenderProvider:defenderPosition", target.x(), target.y(), 80, 50, Drawings::PenStyle::solidPen, ColorRGBA::red, Drawings::BrushStyle::solidBrush, ColorRGBA::red);
  CYLINDER3D("module:LibStrikerProvider:strikerPosition", target.x(), target.y(), 0.0f, 0.0f, 0.0f, 0.0f, 100.0f, 0.0f, ColorRGBA::red);

  return target.hasNaN() ? Vector2f(0.f,0.f) : target;
}

Angle LibStrikerProvider::getStrikerOrientation(bool ballSeen, Vector2f robotPosition) const {
  Vector2f bestBall = ballSeen ? theFieldBall.recentBallPositionOnField() : theTeamBallModel.position;
  Angle recoveryAngle = (Vector2f(theFieldDimensions.xPosOpponentGoal, 0.f) - robotPosition).angle();
  Angle targetAngle = (bestBall - robotPosition).angle();
  return std::isnan(targetAngle) ? recoveryAngle : targetAngle;
}