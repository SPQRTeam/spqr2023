/**
 * @file LibPassProvider.cpp
 * 
 * See LibPass
 *
 * @author Francesco Petri
 */

#include "LibPassProvider.h"

MAKE_MODULE(LibPassProvider, behaviorControl);

void LibPassProvider::update(LibPass& libPass)
{
  libPass.findPassingLine = [this](Vector2f target, std::vector<Vector2f> mates) -> Vector2f {
    return findPassingLine(target, mates);
  };
  libPass.poseToPass = [this, &libPass]() -> Pose2f {
    return poseToPass(libPass);
  };
  libPass.strikerPassShare = [this]() -> std::tuple<int,int,Pose2f> {
    return strikerPassShare();
  };

  // isTargetToPass provided by calls to poseToPass
}


bool LibPassProvider::findPassingLineAux(std::vector<Obstacle> opponents, Vector2f& target, const Vector2f& translation) {
  if(opponents.empty()) {
    target = translation;
    return true;
  }

  // compare against translation
  Eigen::ParametrizedLine<float,2> line = Eigen::ParametrizedLine<float,2>::Through(theRobotPose.translation, translation);

  // in case of failure
  std::vector<Obstacle> originalVector = opponents;

  for(auto it=opponents.begin(); it!=opponents.end(); ++it) {
    if(line.distance(it->center) > 400.f) {
      // call this with reduced obstacles
      opponents.erase(it);
      if(findPassingLineAux(opponents, target, translation)) {
        target = translation;
        return true;
      }
    }
    opponents = originalVector;
  }
  return false;
};

Vector2f LibPassProvider::findPassingLine(Vector2f target, std::vector<Vector2f> mates) {
  bool found = false;
  Vector2f originalTarget = target;
  Vector2f leftTarget = target + Vector2f(0,0);
  Vector2f rightTarget = target - Vector2f(0,0);

  // normalization factor for distance w.r.t. the receiver
  // used to avoid throwing the ball on the back of the receiver
  float nobacknormalization = 1;
  bool isThrowingOnTheBack = false;

  // only use opponents ahead
  // or if behind the opponent penalty mark also use those behind
  std::vector<Obstacle> opponents;
  for(const auto& obs : theTeamPlayersModel.obstacles) {
    if(obs.type == Obstacle::opponent) {
      // if theRobot is inside penalty area then push robots from 2000f to theRobot position
      // else push those ahead only
      if(theRobotPose.translation.x() > theFieldDimensions.xPosOpponentPenaltyMark &&
        obs.center.x() < theRobotPose.translation.x() &&
        obs.center.x() > 2000.f) {
        opponents.push_back(obs);
      } else if(obs.center.x() > theRobotPose.translation.x() &&
        obs.center.x() < target.x()) {
        opponents.push_back(obs);
      }
    }
  }

  // now cycle over the merged obstacles
  while(!found &&
        leftTarget.y() < theFieldDimensions.yPosLeftSideline &&
        rightTarget.y() > theFieldDimensions.yPosRightSideline) {

    // check if we are throwing the ball on the back of the receiver
    Eigen::ParametrizedLine<float,2> toReceiver1 = Eigen::ParametrizedLine<float,2>::Through(theRobotPose.translation, leftTarget);
    Eigen::ParametrizedLine<float,2> toReceiver2 = Eigen::ParametrizedLine<float,2>::Through(theRobotPose.translation, rightTarget);
    for(Vector2f mate : mates) {
      // do not do this for back passes
      if(mate.x() < theRobotPose.translation.x()) {
        continue;
      }

      nobacknormalization = 1+(std::abs(mate.x() - theRobotPose.translation.x())/theFieldDimensions.xPosOpponentGroundLine);
      if(std::abs(toReceiver1.distance(mate)) < 200.f*nobacknormalization ||
              std::abs(toReceiver2.distance(mate)) < 200.f*nobacknormalization) {
        isThrowingOnTheBack = true;
        break;
      }
    }

    if(isThrowingOnTheBack) {
      // increase left and right
      isThrowingOnTheBack = false;
    } else {
      // compute the actual passage
      if(target.y() > theRobotPose.translation.y()) {
        if(findPassingLineAux(opponents, target, rightTarget) ||
          findPassingLineAux(opponents, target, leftTarget)) {
          return target;
        }
      } else {
        if(findPassingLineAux(opponents, target, leftTarget) ||
          findPassingLineAux(opponents, target, rightTarget)) {
          return target;
        }
      }
    }

    // increase left and right
    leftTarget = leftTarget + Vector2f(0,50);
    rightTarget = rightTarget - Vector2f(0,50);
  }
  // maybe next time
  return Vector2f(theFieldDimensions.xPosOpponentGroundLine, 0);
}

Pose2f LibPassProvider::poseToPass(LibPass& libPass) {
  // custom team mate definition
  struct eMate {
    Vector2f position;
    float utility;
    int number;
  };

  // generate the list of valid teammates
  std::vector<eMate> orderedMates;
  // generate auxiliary list of vector2f of available mates
  std::vector<Vector2f> auxMates;
  // for rear pass do not consider forward passage, do this only for forward pass
  float passForward = 600.f;

  for(const auto& mate : theTeamData.teammates) {
    // enable rear passage if inside the opponent area
    // or if not robots have to be ahead of us
    if((theRobotPose.translation.x() > theFieldDimensions.xPosOpponentPenaltyMark && mate.theRobotPose.translation.x() > 2000.f)
        || mate.theRobotPose.translation.x() > theRobotPose.translation.x()) {
      eMate emate;
      emate.position = mate.theRobotPose.translation;
      emate.utility = std::numeric_limits<float>::max();
      emate.number = mate.number;
      // std::cout<<"mate number "<<mate.number<<std::endl;
      // std::cout<<"mate role "<<mate.theTeamBehaviorStatus.role.getName()<<std::endl;   // QUESTA RIGA NON FUNZIONA SE TEAMBEHAVIORSTATUS NON È IN RETE (ma si può sistemare usando direttamente TypeRegistry::getEnumName su mate.role)
      // order w.r.t. distance from the closest opponent
      for(const auto& obs : theTeamPlayersModel.obstacles) {
        if(obs.type == Obstacle::opponent &&
            obs.center.x() <= mate.theRobotPose.translation.x()) {
          // compute emate utility w.r.t. obs
          float newUtility = (float)(std::pow(emate.position.x()-obs.center.x(),2) +
            std::pow(emate.position.y()-obs.center.y(),2));
          if(newUtility < emate.utility) {
            emate.utility = newUtility;
          }
        }
      }
      // order w.r.t utility
      bool added = false;
      if(orderedMates.empty()) {
        orderedMates.push_back(emate);
        auxMates.push_back(mate.theRobotPose.translation);
      } else {
        for(auto it = orderedMates.begin(); it != orderedMates.end(); ++it) {
          if(it->utility < emate.utility) {
            orderedMates.insert(it, emate);
            added = true;
            break;
          }
        }
      }
      if(!added) {
        // actual ordered list of mates
        // TODO remove to optimize
        orderedMates.push_back(emate);
        // use for the chek of robot backs (see find passing line)
        auxMates.push_back(mate.theRobotPose.translation);
      }
    }
  }

  // now call the findPassingLine starting from the best mate
  for(const auto& mate : orderedMates) {
    if(theRobotPose.translation.x() > theFieldDimensions.xPosOpponentPenaltyMark) {
      passForward = 0.f;
    }
    Vector2f target = findPassingLine(mate.position, auxMates);
    // make forward pass
    if(mate.position.x() < theFieldDimensions.xPosOpponentPenaltyMark) {
      // increase the x
      target.x() = target.x()+passForward;
    }

    // if target is the center of the goal then no valid passing line, continue
    if(target != Vector2f(theFieldDimensions.xPosOpponentGroundLine+passForward,0.f)){
      libPass.isTargetToPass = mate.number;
      return target;
    }
  }
  return Vector2f(theFieldDimensions.xPosOpponentGroundLine,0.f);
};

std::tuple<int,int,Pose2f> LibPassProvider::strikerPassShare() const {
  for(const auto& teammate : theTeamData.teammates) {
    // [2022] cleared lots of commented stuff. Check spqrnao2022 if you REALLY need it.
    if(teammate.role == PlayerRole::RoleType::striker) {
      return std::tuple<int,int,Pose2f>(teammate.thePassShare.readyPass,teammate.thePassShare.passingTo,teammate.thePassShare.passTarget);
    }
  }
  return std::tuple<int,int,Pose2f>(0,0,theRobotPose);
};
