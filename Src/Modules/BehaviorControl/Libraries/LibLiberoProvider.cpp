/**
 * @file LibLiberoProvider.cpp
 * 
 * See LibLibero
 *
 * @author Flavio Volpi
 */

#include "LibLiberoProvider.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Debugging/DebugDrawings.h"

MAKE_MODULE(LibLiberoProvider, behaviorControl);

void LibLiberoProvider::update(LibLibero& libLibero)
{
    DECLARE_DEBUG_DRAWING3D("module:LibLiberoProvider:liberoPosition", "field");
    DECLARE_DEBUG_DRAWING("module:LibLiberoProvider:liberoPosition", "drawingOnField");

    libLibero.getLiberoPosition = [this]() -> Vector2f {
        return getLiberoPosition();
    };

    libLibero.getLiberoOrientation = [this](bool ballSeen, Vector2f target) -> Angle {
        return getLiberoOrientation(ballSeen, target);
    };
}

Vector2f LibLiberoProvider::getLiberoPosition() const {
    Vector2f target;
    // std::vector<Voronoi::Node> g = theVoronoi.graph;
    for(int i=0; i<theVoronoi.graph.size(); ++i){
        if(theVoronoi.graph[i].position.x() < theTeamBallModel.position.x()){
            target = theVoronoi.graph[i].position;
            break;
        }
    }
    CYLINDER3D("module:LibLiberoProvider:liberoPosition", target.x(), target.y(), 0.0f, 0.0f, 0.0f, 0.0f, 50.0f, 60.0f, ColorRGBA::yellow);
    CIRCLE("module:LibLiberoProvider:liberoPosition", target.x(), target.y(), 80, 50, Drawings::PenStyle::solidPen, ColorRGBA::yellow, Drawings::BrushStyle::solidBrush, ColorRGBA::yellow);

    return target.hasNaN() ? Vector2f(-2000.f, 0.f): target;
}

Angle LibLiberoProvider::getLiberoOrientation(bool ballSeen, Vector2f robotPosition) const {
  Vector2f bestBall = ballSeen ? theFieldBall.recentBallPositionOnField() : theTeamBallModel.position;
  Angle recoveryAngle = (Vector2f(theFieldDimensions.xPosOpponentGoal, 0.f) - robotPosition).angle();
  Angle targetAngle = (bestBall - robotPosition).angle();
  return std::isnan(targetAngle) ? recoveryAngle : targetAngle;
}


