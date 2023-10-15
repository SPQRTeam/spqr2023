/**
 * @file LibSpecProvider.cpp
 * 
 * See LibSpec
 *
 */

#include "LibSpecProvider.h"


MAKE_MODULE(LibSpecProvider, behaviorControl);

void LibSpecProvider::update(LibSpec& libSpec)
{
  DECLARE_DEBUG_DRAWING3D("module:LibSpecProvider:targetPosition", "field");
  DECLARE_DEBUG_DRAWING3D("module:LibSpecProvider:corridor", "field");
  DECLARE_DEBUG_DRAWING3D("module:LibSpecProvider:shortestpath", "field");
  DECLARE_DEBUG_DRAWING("module:LibSpecProvider:targetPosition", "drawingOnField");

  libSpec.compare_obstacles = [this](Vector2f obs1, Vector2f obs2) -> bool {
    return compare_obstacles(obs1, obs2);
  };
  libSpec.calcAngleCorner = [this]() -> std::tuple<float, float> {
    return calcAngleCorner();
  };
  libSpec.targetCornerPoint = [this](float angle, float radius) -> Vector2f {
    return targetCornerPoint(angle, radius);
  };
  libSpec.calcCornerZone = [this](const Vector2f& point, const char side) -> int {
    return calcCornerZone(point, side);
  };
  libSpec.targetDefenseCornerPoint = [this]() -> Vector2f {
    return targetDefenseCornerPoint();
  };
  libSpec.getPositionCoveringDangerousOpponent = [this](bool onBall) -> Vector2f {
    return getPositionCoveringDangerousOpponent(onBall);
  };
  libSpec.isCorridorCovered = [this](const Vector2f source, const Vector2f target, const float width, const Vector2f teammate) -> bool {
    return isCorridorCovered(source, target, width, teammate);
  };
  libSpec.nearestPointOnCorridor = [this](const Vector2f source, const Vector2f target) -> Vector2f {
    return nearestPointOnCorridor(source, target);
  };
  libSpec.getMostDangerousOpponent = [this]() -> Vector2f {
    return getMostDangerousOpponent();
  };
}

bool LibSpecProvider::compare_obstacles(Vector2f obs1, Vector2f obs2) const{
  Vector2f start = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosLeftSideline); //start on left corner    
  Vector2f endBound1 = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosRightGoalArea); //end on right bound goal area 

  float angle1 = theLibMisc.angleBetweenGlobalVectors(start, endBound1, obs1);
  float angle2 = theLibMisc.angleBetweenGlobalVectors(start, endBound1, obs2);

  return angle1 <= angle2;
}


std::tuple<float, float> LibSpecProvider::calcAngleCorner() const{
  Vector2f startBounds;
  Vector2f endBound1;
  Vector2f endBound2;
  
  (theTeamBallModel.position.y() > 0) ? ({   // left corner
    startBounds = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosLeftSideline); //start on left corner
    endBound1 = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosRightGoalArea); //end on right bound goal area 
    endBound2 = Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea); //end on left corner of penalty area
    
  }) : ({                                   // right corner
    startBounds = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosRightSideline); //start on right corner
    endBound1 = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosLeftGoalArea); //end on left bound goal area 
    endBound2 = Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosRightPenaltyArea); //end on right corner of penalty area
  });
  
  float radius =  3000;

  std::list<Vector2f> obstaclesDetected;

  for(auto obstacle: theObstacleModel.obstacles){
    Vector2f obstacle_center = theLibMisc.rel2Glob(obstacle.center.x(), obstacle.center.y()).translation;

    if(obstacle.type != Obstacle::teammate && theLibMisc.isInsideGlobalSector(startBounds, endBound1, endBound2, .0f, radius, obstacle_center)){
      obstaclesDetected.push_back(obstacle_center);
    }
  }
  obstaclesDetected.push_back(endBound2);

  obstaclesDetected.sort([this] (const Vector2f obs1, const Vector2f obs2) {
    return compare_obstacles(obs1, obs2); 
  });

  std::list<Vector2f>::iterator it=obstaclesDetected.begin();    
  ++it;

  std::list<float> angle_list;
  float best_angle = 0;
  int counter_best_angle = 0;
  int k = 0;

  for(it; it!=obstaclesDetected.end(); ++it){
    Vector2f obs1 = *prev(it);
    Vector2f obs2 = *(it);
    float angle = theLibMisc.angleBetweenGlobalVectors(startBounds, obs1, obs2);
    angle_list.push_back(angle);
    if(angle > best_angle){
      best_angle = angle;  
      counter_best_angle = k;
    }          
    ++k;
  }
  
  float angle_to_kick = 0;
  std::list<float>::iterator angle = angle_list.begin();
  for(int i=0; i<=counter_best_angle; i++){
    (i==counter_best_angle) ? (angle_to_kick += (*angle/2)) : ( angle_to_kick += (*angle));
    ++angle;
  }
  
  return std::make_tuple(angle_to_kick, best_angle);
}


Vector2f LibSpecProvider::targetCornerPoint(float angle, float radius) const{
  float x, y;
  x = theFieldDimensions.xPosOpponentGroundLine - radius*sin(angle);
  (theTeamBallModel.position[1] > 0) ? 
    (y = theFieldDimensions.yPosLeftSideline - radius*cos(angle)) : (y = theFieldDimensions.yPosRightSideline + radius*cos(angle)); 
  return Vector2f(x,y);
}

double LibSpecProvider::pointToSegmentDistance(Vector2f A, Vector2f B, Vector2f P) const{
    float angle = theLibMisc.angleBetweenGlobalVectors(A, P, B);
    float distance_AP = theLibMisc.distanceVec(A, P);
    float distance_AB = theLibMisc.distanceVec(A, B);
    if(cos(angle) < 0.f || distance_AP*cos(angle) > distance_AB)
      return -1;
    return distance_AP*sin(abs(angle));
}

bool orderObstaclesByXCoordinate(const Obstacle o1, const Obstacle o2) {
    return (o1.center.x() < o2.center.x());
}

Vector2f LibSpecProvider::getPositionCoveringDangerousOpponent(bool onBall) const {
      
    Vector2f start = theFieldBall.recentBallPositionOnField();
    Vector2f end = getMostDangerousOpponent();
    // Vector2f end  = Vector2f(0.f,0.f);
    LINE3D("module:LibSpecProvider:corridor", start.x(), start.y(), 0.0, end.x(), end.y(), 0.0, 5, ColorRGBA::red);
    Geometry::Line segment(start, end - start);
    Vector2f robot = theRobotPose.translation;
    const float length = segment.direction.dot(segment.direction);
    const float localProjection = (robot - segment.base).dot(segment.direction) / length;
    const Vector2f unitdir = segment.direction / length;

    Vector2f target{};
    if(localProjection <= 0)
      target = start + (1/4)*length * unitdir;
    else if((localProjection >= 1.0f && onBall) || onBall)
      target = end - (1/4)*length * unitdir;
    else
      target = start + localProjection * segment.direction;

    LINE3D("module:LibSpecProvider:shortestpath", robot.x(), robot.y(), 0.0, target.x(), target.y(), 0.0, 5, ColorRGBA::blue);
    return target; 
}

Vector2f LibSpecProvider::freeKickWall(Vector2f robot) const{
    return Vector2f(0.f,0.f);
}


Vector2f LibSpecProvider::getMostDangerousOpponent() const
{
    //Parameters for checking corridor coverage
    bool corridorCovered = false;
    Vector2f source = theFieldBall.recentBallPositionOnField();
    Vector2f target = Vector2f(theFieldDimensions.xPosOwnGroundLine,0.f);
    float width = theFieldDimensions.yPosLeftGoal - 100.f;

    // generate and fill an array of obstacles in absolute coordinates
    std::vector<Obstacle> orderedObstacles;
    for(const auto& obstacle : theTeamPlayersModel.obstacles) 
        if(obstacle.type == Obstacle::opponent) 
            orderedObstacles.push_back(obstacle);
    std::sort(orderedObstacles.begin(), orderedObstacles.end(), orderObstaclesByXCoordinate);

    //Take the nearest opponent to own goal (not in the corridor to the goal)
    int i = 0;
    for (; i < orderedObstacles.size(); i++){
      Vector2f nearestOppToGoal = orderedObstacles[i].center;
      if(!isCorridorCovered(source,target,width,nearestOppToGoal))
        return nearestOppToGoal;
    }

    if (i > 0)
      return orderedObstacles[0].center; //placeholder if all are covered
    else
      return target;
}

bool LibSpecProvider::isCorridorCovered(const Vector2f start, const Vector2f end, const float width, const Vector2f robot) const {
    
    Geometry::Line segment(start, end - start);
    float distance = Geometry::getDistanceToEdge(segment, robot);

    return (distance >= 0 && distance < width);
}

Vector2f LibSpecProvider::nearestPointOnCorridor(const Vector2f start, const Vector2f end) const {

      LINE3D("module:LibSpecProvider:corridor", start.x(), start.y(), 0.0, end.x(), end.y(), 0.0, 5, ColorRGBA::red);
      Geometry::Line segment(start, end - start);
      Vector2f robot = theRobotPose.translation;
      const float length = segment.direction.dot(segment.direction);
      const float localProjection = (robot - segment.base).dot(segment.direction) / length;
      const Vector2f unitdir = segment.direction / length;

      Vector2f target{};
      if(localProjection <= 0)
        target = start + 2000.f * unitdir;
      else if(localProjection >= 1.0f)
        target = end - 700.f * unitdir;
      else
        target = start + localProjection * segment.direction;

      LINE3D("module:LibSpecProvider:shortestpath", robot.x(), robot.y(), 0.0, target.x(), target.y(), 0.0, 5, ColorRGBA::blue);
      return target; 
}


/*params
    side: l=left, r=right
    return: 0=most dangerous zone, 1=near dangerous zone, 2=far dangerous zone
*/
int LibSpecProvider::calcCornerZone(const Vector2f& point, const char side) const{    //l=left    r=right

  if(side=='l'){
    if(theLibMisc.isInsideGlobalRectangle(point, Vector2f(theFieldDimensions.xPosOwnGroundLine, 0), 
      Vector2f(0, theFieldDimensions.yPosLeftSideline),
      Vector2f(0, theFieldDimensions.yPosRightGoalArea),
      Vector2f(theFieldDimensions.xPosOwnPenaltyArea, 0)))
        return 0;
    else if(theLibMisc.isInsideGlobalRectangle(point, Vector2f(theFieldDimensions.xPosOwnPenaltyArea, 0), 
      Vector2f(0, theFieldDimensions.yPosLeftSideline),
      Vector2f(0, theFieldDimensions.yPosCenterGoal),
      Vector2f(theFieldDimensions.xPosHalfWayLine - theFieldDimensions.centerCircleRadius, 0)))
        return 1;
    else
        return 2;
  }

  else if(side=='r'){
    if(theLibMisc.isInsideGlobalRectangle(point, Vector2f(theFieldDimensions.xPosOwnGroundLine, 0), 
      Vector2f(0, theFieldDimensions.yPosLeftGoalArea),
      Vector2f(0, theFieldDimensions.yPosRightSideline),
      Vector2f(theFieldDimensions.xPosOwnPenaltyArea, 0)))
        return 0;
    else if(theLibMisc.isInsideGlobalRectangle(point, Vector2f(theFieldDimensions.xPosOwnPenaltyArea, 0), 
      Vector2f(0, theFieldDimensions.yPosCenterGoal),
      Vector2f(0, theFieldDimensions.yPosRightSideline),
      Vector2f(theFieldDimensions.xPosHalfWayLine - theFieldDimensions.centerCircleRadius, 0)))
        return 1;
    else
        return 2;
  }
}



Vector2f LibSpecProvider::targetDefenseCornerPoint() const{

  // set the bounds for the sector of defender (startbounds = corner, bound1 = point of defender closer to ground line, bound2 = other)
  Vector2f target_point;
  Vector2f corner;
  (theTeamBallModel.position.y() > 0) ? ({ // left corner
    //preliminary target position (to avoid crash)
    target_point = Vector2f((theFieldDimensions.xPosOwnGoalArea + theFieldDimensions.xPosOwnPenaltyMark)/2, (theFieldDimensions.yPosLeftPenaltyArea + theFieldDimensions.yPosLeftGoalArea)/2);
    corner = Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosLeftSideline);
  }) : ({                                   // right corner
    //preliminary target position (to avoid crash)
    target_point = Vector2f((theFieldDimensions.xPosOwnGoalArea + theFieldDimensions.xPosOwnPenaltyMark)/2, (theFieldDimensions.yPosRightPenaltyArea + theFieldDimensions.yPosRightGoalArea)/2);
    corner = Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosRightSideline); 
  });

  // float angle_left = theLibMisc.angleBetweenGlobalVectors(startBounds, 
  //                   Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosRightGoalArea),
  //                   defender_left);
  // float angle_right = theLibMisc.angleBetweenGlobalVectors(startBounds, 
  //                   Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosRightGoalArea),
  //                   defender_right);
    
  // (angle_left < angle_right) ? ({
  //   endBound1 = defender_left;
  //   endBound2 = defender_right;
  // }) : ({
  //   endBound1 = defender_right;
  //   endBound2 = defender_left;
  // });


  // start and end radius of sector 
  float startRadius = 1000;
  float endRadius = 6000;

  // iterate on all obstacles and check if they are covered or not by the defender
  std::list<Vector2f> obstaclesUncovered; // obstacles uncovered by defender
  for(auto obstacle: theObstacleModel.obstacles){
    if((obstacle.type == Obstacle::opponent || obstacle.type == Obstacle::fallenOpponent)){ 
      Vector2f obstacle_center = theLibMisc.rel2Glob(obstacle.center.x(), obstacle.center.y()).translation;
      bool covered = false;
      
      for(auto teammate:theTeamData.teammates){
        Vector2f teammate_center = teammate.theRobotPose.translation;
        if(isCorridorCovered(corner, obstacle_center, 300, teammate_center))
          covered = true;
      }
    }


    // if((obstacle.type == Obstacle::opponent || obstacle.type == Obstacle::fallenOpponent)){ // if obstacle is opponent
    //   if(!theLibMisc.isInsideGlobalSector(startBounds, endBound1, endBound2, startRadius, endRadius, obstacle_center) || // if not inside sector OR
    //     ((theTeamBallModel.position.y()>0 && obstacle_center.y()>defender_center.y()) ||  // ((is left corner AND is over defender) OR 
    //       (theTeamBallModel.position.y()<0 && obstacle_center.y()<defender_center.y()))){  //  (is right corner AND is over defender))
    //         obstaclesUncovered.push_back(obstacle_center);
    //   }
    // }
  }

  if(obstaclesUncovered.size()!=0){
    // take the dangerous obstacle (among the uncovered obstacles): is the obstacle with minimum distance from the point  (-3900,0)
    Vector2f dangerous_obstacle;
    float dangerous_distance;
    Vector2f point_dangerous = Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosCenterGoal);
    std::list<Vector2f>::iterator it=obstaclesUncovered.begin();    
    
    for(it; it!=obstaclesUncovered.end(); ++it){
      Vector2f obs = *(it);

      if(it == obstaclesUncovered.begin()){
        dangerous_obstacle = obs;
        dangerous_distance = theLibMisc.distanceVec(dangerous_obstacle, point_dangerous);
      }
      else{
        float actual_distance = theLibMisc.distanceVec(obs, point_dangerous);
        if(actual_distance < dangerous_distance){
          dangerous_obstacle = obs;      
          dangerous_distance = theLibMisc.distanceVec(dangerous_obstacle, point_dangerous);     
        }
      }
    }

    // choose the target point by checking how much is dangerous the obstacle (looking in which zone the obstacle is)
    // (zone = 0: most dangerous, zone = 1: could be dangerous, zone = 2: not really dangerous)
    float angle;
    float dist2opp = 300;
    float ring1 = 1700;
    float ring2 = 1300;
    
    if(theTeamBallModel.position.y()>0){ //LEFT CORNER
      int cornerZone = calcCornerZone(dangerous_obstacle, 'l');
      switch(cornerZone){
        case 0:
          if(theLibMisc.distanceVec(Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosLeftSideline), dangerous_obstacle)<1100){   //value can be changed
            angle = theLibMisc.angleBetweenGlobalVectors(Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosCenterGoal), dangerous_obstacle, Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosLeftSideline));
            target_point = Vector2f(dangerous_obstacle.x()-(dangerous_obstacle.y()-(theFieldDimensions.yPosLeftSideline-1100.f))*tan(angle), 
                                    theFieldDimensions.yPosLeftSideline-1100.f);
          }
          else if(theLibMisc.distanceVec(Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosLeftSideline), dangerous_obstacle)<1500){
            angle = theLibMisc.angleBetweenGlobalVectors(Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosCenterGoal), dangerous_obstacle, Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosLeftSideline));
            target_point = Vector2f(dangerous_obstacle.x()-dist2opp*sin(angle), dangerous_obstacle.y()-dist2opp*cos(angle));

          }
          else{
            angle = theLibMisc.angleBetweenGlobalVectors(Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosLeftSideline), dangerous_obstacle, Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosCenterGoal));
            target_point = Vector2f(dangerous_obstacle.x()-dist2opp*sin(angle), dangerous_obstacle.y()+dist2opp*cos(angle));

          }
          break;
        
        case 1:
          angle = theLibMisc.angleBetweenGlobalVectors(Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosLeftSideline), dangerous_obstacle, Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosCenterGoal));
          target_point = Vector2f(theFieldDimensions.xPosOwnGroundLine+ring1*sin(angle), theFieldDimensions.yPosLeftSideline-ring1*cos(angle));
          break;

        case 2:
          angle = theLibMisc.angleBetweenGlobalVectors(Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosLeftSideline), dangerous_obstacle, Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosCenterGoal));
          target_point = Vector2f(theFieldDimensions.xPosOwnGroundLine+ring2*sin(angle), theFieldDimensions.yPosLeftSideline-ring2*cos(angle));
          break;
      }
    }
    else{ //RIGHT CORNER
      int cornerZone = calcCornerZone(dangerous_obstacle, 'r');
      switch(cornerZone){
        case 0:
          // ZONE 0
          if(theLibMisc.distanceVec(Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosRightSideline), dangerous_obstacle)<1100){   //value can be changed
            angle = theLibMisc.angleBetweenGlobalVectors(Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosCenterGoal), dangerous_obstacle, Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosRightSideline));
            target_point = Vector2f(dangerous_obstacle.x()+(dangerous_obstacle.y()-(theFieldDimensions.yPosRightSideline+1100.f))*tan(angle), 
                                    theFieldDimensions.yPosRightSideline+1100.f);
          }
          else if(theLibMisc.distanceVec(Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosRightSideline), dangerous_obstacle)<1500){
            angle = theLibMisc.angleBetweenGlobalVectors(Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosCenterGoal), dangerous_obstacle, Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosRightSideline));
            target_point = Vector2f(dangerous_obstacle.x()-dist2opp*sin(angle), dangerous_obstacle.y()+dist2opp*cos(angle));
          }
          else{
            angle = theLibMisc.angleBetweenGlobalVectors(Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosRightSideline), dangerous_obstacle, Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosCenterGoal));
            target_point = Vector2f(dangerous_obstacle.x()-dist2opp*sin(angle), dangerous_obstacle.y()-dist2opp*cos(angle));
          }
          break;
        
        case 1:
          // ZONE 1
          angle = theLibMisc.angleBetweenGlobalVectors(Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosRightSideline), dangerous_obstacle, Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosCenterGoal));
          target_point = Vector2f(theFieldDimensions.xPosOwnGroundLine+ring1*sin(angle), theFieldDimensions.yPosRightSideline+ring1*cos(angle));
          break;

        case 2:
          // ZONE 2
          angle = theLibMisc.angleBetweenGlobalVectors(Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosRightSideline), dangerous_obstacle, Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosCenterGoal));
          target_point = Vector2f(theFieldDimensions.xPosOwnGroundLine+ring2*sin(angle), theFieldDimensions.yPosRightSideline+ring2*cos(angle));
          break;
      }

    }

  }
  else{
    target_point = Vector2f(theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosCenterGoal);
  }
  
  //CYLINDER3D("module:LibSpecProvider:targetPosition", target_point.x(), target_point.y(), 0.0f, 0.0f, 0.0f, 0.0f, 50.0f, 60.0f, ColorRGBA::violet);
  //CIRCLE("module:LibSpecProvider:targetPosition", target_point.x(), target_point.y(), 80, 50, Drawings::PenStyle::solidPen, ColorRGBA::blue, Drawings::BrushStyle::solidBrush, ColorRGBA::blue);

  return target_point;
}
