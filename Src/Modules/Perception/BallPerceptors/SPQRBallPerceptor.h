/**
 * @file CascadeBallPerceptor.h
 * This file declares a module that provides the white and black ball percept
 * using cascade.
 * If possible, it uses the full YCbCr422 image.
 * @author Francesco Del Duchetto
 * @author Tiziano Manoni
  */

#pragma once
#include "Representations/spqr_representations/OurDefinitions.h"


#include "Representations/Perception/ImagePreprocessing/BodyContour.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Perception/FieldFeatures/FieldFeatureOverview.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/spqr_representations/SPQRPatches.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesFieldPercept.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesImagePercept.h"
#include "Representations/Modeling/WorldModelPrediction.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Communication/RobotInfo.h"
#include "Tools/ImageProcessing/InImageSizeCalculations.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <iostream>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h> 
#include <sys/mman.h>

#ifdef TARGET_ROBOT
#define IS_FULL_SIZE true
#else
#define IS_FULL_SIZE theCameraImage.isFullSize
#endif




MODULE(SPQRBallPerceptor,
{,
    REQUIRES(FieldDimensions),
    REQUIRES(BallSpecification),
    REQUIRES(CameraImage),
    REQUIRES(CameraMatrix),
    REQUIRES(ImageCoordinateSystem),
    REQUIRES(CameraInfo),
    REQUIRES(FrameInfo),
    REQUIRES(FieldFeatureOverview),
    REQUIRES(Odometer),
    REQUIRES(SPQRPatches),
    REQUIRES(WorldModelPrediction),
    REQUIRES(BodyContour),
    REQUIRES(ObstaclesImagePercept),
    REQUIRES(ObstaclesFieldPercept),
    REQUIRES(BallModel),
    USES(ObstacleModel),
    USES(TeamBallModel),
    USES(RobotPose),
    USES(RobotInfo),
    USES(BallPercept),
    PROVIDES(BallPercept),
    LOADS_PARAMETERS(
    {,
        (float)(1.1f) SCALE_FACTOR, //The factor to determine the scale
        (int)(15) DIFFERENCE_RADIUS_THRESHOLD,
        (int) (0) FLAGS,
        (bool) DEBUG,
        (bool) DEBUG_SPEED,
        (bool) DEBUG_BALL_BLOCKER,
        (bool) (false) SAVE_POSITIVES,
        (bool) (false) SAVE_NEGATIVES,
        (bool) BLOCK_BALLS_ON_OBSTACLES_IMAGE,
        (bool) BLOCK_BALLS_ON_OBSTACLES_FIELD,
        (bool) BLOCK_BALLS_WITH_BAD_RADIUS,
        (bool) BLOCK_BALLS_OUTSIDE_FIELD,
        (bool) BLOCK_BALLS_INSIDE_BODYCONTOUR,
    }),

});

/**
* @class SPQRBallPerceptorScaler
* The class scales the input and output data if full size images are available.
*/
class SPQRBallPerceptorScaler : public SPQRBallPerceptorBase
{
private:
    using SPQRBallPerceptorBase::theCameraImage; //> prevent direct access to image

protected:
    CameraInfo theCameraInfo;
    //BallSpots theBallSpots;
    ImageCoordinateSystem theImageCoordinateSystem;

    /**
    * The only access to image pixels.
    * @param x The x coordinate of the pixel in the range defined in theCameraInfo.
    * @param y The y coordinate of the pixel in the range defined in theCameraInfo.
    * @return The pixel as a temporary object. Do not use yCbCrPadding of that pixel.
    */
    const CameraImage::PixelType getPixel(int y, int x) const
    {
        if(theCameraInfo.camera == CameraInfo::upper && IS_FULL_SIZE)
            return theCameraImage.getFullSizePixel(y, x);
        else
            return theCameraImage[y][x];
    }

    /**
    * Update the copies of input representations that contain data
    * that might have to be scaled up.
    * Must be called in each cycle before any computations are performed.
    */
    void initialize();

    /**
    * Scale down a single value if required.
    * This is only supposed to be used in debug drawings.
    * @param value The value that might be scaled.
    * @return The scaled value.
    */
    template<typename T> T scale(T value) const
    {
        if(theCameraInfo.camera == CameraInfo::upper && IS_FULL_SIZE)
            return value / (T) 2;
        else
            return value;
    }
};

/**
* @class SPQRBallPerceptor
* The actual ball perceptor.
* It is basically identical to the module "BallPerceptor".
*/
class SPQRBallPerceptor : public SPQRBallPerceptorScaler
{


    using SPQRBallPerceptorBase::theCameraImage; //> prevent direct access to image

    std::string ball_cascade_name_top;
    std::string ball_cascade_name_top_highRes;
    std::string ball_cascade_name_bottom;
    cv::CascadeClassifier top_cascade;
    cv::CascadeClassifier top_cascade_highRes;
    cv::CascadeClassifier bottom_cascade;

    void* shm_ptr;
    unsigned timeOfLastPerceivedFieldFeature;

    /**
    * The main method of this module.
    * This module use cascade to determine the ball position, records
    * how many frames was not seen and how much time cascade needs for
    * find the ball
    * @param ballPercept The percept that is filled by this module.
    */
    void update(BallPercept& ballPercept);


    /**
    * If the patch is a kalman prediction creates a matrix of the image
    * and detect the dimensions subject to the camera which takes the data.
    * Then finds the ball in the patch and determine the position and
    * the radius.
    * @param patch the image patch
    * @param ballPercept the ball percept
    * @param cam the camera which takes the frames
    * @param ball the coordinates of the ball
    * @param radius the ball radius
    * @return false if the size of the patch is too small for cascade
    * else
    * @return true if there are no errors
    */
    bool findBallOnPatch(SPQRPatch patch, BallPercept ballPercept, CameraInfo::Camera cam, std::vector<Vector2i>& ball, std::vector<short>& radius);

    bool perceptCanBeExcludedByLocalization(Vector2f localPosition) const;

    /**
    * Calculate the correct position of the ball in the field and
    * compute velocity
    * @param ballPercept the ball percept
    * @return false if there are errors
    * else
    * @return true if the operation was completed successfully
    */
    bool calculateBallOnField(BallPercept& ballPercept) const;

    /**
    * Determine if the ball is on field in a correct position
    * @param ballPercept the ball percept
    * @return false if the ball is not on the carpet, if there is an
    * obstacle near the ball or if the radius was bad calculated
    * else
    * @return true if the ball is on field
    */
    bool checkBallOnField(BallPercept& ballPercept) const;

    /**
    * Determine if the ball is inside the image patch
    * @param ball the coordinates of the ball
    * @param patch the image patch
    * @return false if the ball is above horizon or inside the body contour
    * or if there is an obstacle in between
    * else
    * @return true if the ball is inside the image limits
    */
    bool checkBallOnImage(Vector2i ball, SPQRPatch patch);


public:
    SPQRBallPerceptor() ;
};
