//Ip Periv.cpp
/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2023 German Aerospace Center (DLR) and others.
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// https://www.eclipse.org/legal/epl-2.0/
// This Source Code may also be made available under the following Secondary
// Licenses when the conditions for such availability set forth in the Eclipse
// Public License 2.0 are satisfied: GNU General Public License, version 2
// or later which is available at
// https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
// SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later
/****************************************************************************/
/// @file    MSPerIvan.cpp
/// @author  Melanie Weber
/// @author  Andreas Kendziorra
/// @author  Michael Behrisch
/// @date    Thu, 12 Jun 2014
///
// The common superclass for modelling transportable objects like persons and containers
/****************************************************************************/
#include <config.h>
#include <math.h>
#include <cmath>
#include <utils/common/RandHelper.h>
#include <utils/common/SUMOTime.h>
//#include <microsim/MSVehicle.h>
#include <microsim/transportables/MSPerson.h>
//#include <microsim/MSLane.h>
#include <microsim/MSEdge.h>
//#include <microsim/MSGlobals.h>
//#include <microsim/MSNet.h>
#include <microsim/traffic_lights/MSTrafficLightLogic.h>
#include <microsim/lcmodels/MSAbstractLaneChangeModel.h>
#include "MSPerIvan.h"

// ===========================================================================
// DEBUG constants
// ===========================================================================
//#define DEBUG_OUPROCESS
//#define DEBUG_TRAFFIC_ITEMS
//#define DEBUG_PERCEPTION_ERRORS
//#define DEBUG_DRIVERSTATE
//#define DEBUG_COND (true)
//#define DEBUG_COND (myVehicle->isSelected())


/* -------------------------------------------------------------------------
 * static member definitions
 * ----------------------------------------------------------------------- */
 // hash function
 //std::hash<std::string> MSPerIvan::MSTrafficItem::hash = std::hash<std::string>();
SumoRNG OUProcessIV::myRNG("perivan");

// ===========================================================================
// Default value definitions
// ===========================================================================

double PerIvanDefaults::errorTimeScaleCoefficient = 25.0;
double PerIvanDefaults::perceptionDelay = 0.0;
double PerIvanDefaults::errorNoiseIntensityCoefficient = 0.2;
double PerIvanDefaults::speedDifferenceErrorCoefficient = 0.15;
double PerIvanDefaults::headwayErrorCoefficient = 0.75;
double PerIvanDefaults::persistentHeadwayError = 0.0;
double PerIvanDefaults::optimalPerceptionRange = 50.0;
double PerIvanDefaults::maximalPerceptionRange = 150.0;
double PerIvanDefaults::maxHeadwayError = 5.0;
double PerIvanDefaults::headwayErrorShape = 1.0;
double PerIvanDefaults::minDistanceNoiseHeadway = 0.1;
double PerIvanDefaults::minSpeedNoiseHeadway = 0.1;
double PerIvanDefaults::distanceNoiseHeadwayCoeff = 0.001;
double PerIvanDefaults::speedNoiseHeadwayCoeff = 0.001;
double PerIvanDefaults::optimalSpeedRange = 10.0;
double PerIvanDefaults::persistentDeltaVError = 0.1;
double PerIvanDefaults::maxDeltaVError = 0.5;
double PerIvanDefaults::deltaVErrorShape = 1.0;
double PerIvanDefaults::minDistanceNoiseDeltaV = 0.1;
double PerIvanDefaults::minSpeedNoiseDeltaV = 0.1;
double PerIvanDefaults::distanceNoiseDeltaVCoeff = 0.001;
double PerIvanDefaults::speedNoiseDeltaVCoeff = 0.001;
double PerIvanDefaults::param1 = 1.0;
double PerIvanDefaults::param2 = 1.0;
double PerIvanDefaults::freeSpeedErrorCoefficient = 0.0;
double PerIvanDefaults::speedDifferenceChangePerceptionThreshold = 0.1;
double PerIvanDefaults::headwayChangePerceptionThreshold = 0.1;
double PerIvanDefaults::maximalReactionTimeFactor = 1.0;


// ===========================================================================
// method definitions
// ===========================================================================

OUProcessIV::OUProcessIV(double initialState, double timeScale, double noiseIntensity)
    : myState(initialState),
      myTimeScale(timeScale),
      myNoiseIntensity(noiseIntensity) {}

OUProcessIV::~OUProcessIV() {}

void
OUProcessIV::step(double dt) {
    myState = exp(-dt / myTimeScale) * myState + myNoiseIntensity * sqrt(2 * dt / myTimeScale) * RandHelper::randNorm(0, 1, &myRNG);
}

double
OUProcessIV::step(double state, double dt, double timeScale, double noiseIntensity) {
    /// see above
    return exp(-dt / timeScale) * state + noiseIntensity * sqrt(2 * dt / timeScale) * RandHelper::randNorm(0, 1, &myRNG);
}

double
OUProcessIV::getState() const {
    return myState;
}

MSSimplePerIvan::MSSimplePerIvan(MSVehicle* veh) :
    myVehicle(veh),
    myError(0., 1., 1.),
    myErrorTimeScaleCoefficient(PerIvanDefaults::errorTimeScaleCoefficient),
    myPerceptionDelay(PerIvanDefaults::perceptionDelay),
    myErrorNoiseIntensityCoefficient(PerIvanDefaults::errorNoiseIntensityCoefficient),
    mySpeedDifferenceErrorCoefficient(PerIvanDefaults::speedDifferenceErrorCoefficient),
    myHeadwayErrorCoefficient(PerIvanDefaults::headwayErrorCoefficient),
    myPersistentHeadwayError(PerIvanDefaults::persistentHeadwayError),  
    myOptimalPerceptionRange(PerIvanDefaults::optimalPerceptionRange),
    myMaximalPerceptionRange(PerIvanDefaults::maximalPerceptionRange),
    myMaxHeadwayError(PerIvanDefaults::maxHeadwayError),  
    myHeadwayErrorShape(PerIvanDefaults::headwayErrorShape),
    myMinDistanceNoiseHeadway(PerIvanDefaults::minDistanceNoiseHeadway),
    myMinSpeedNoiseHeadway(PerIvanDefaults::minSpeedNoiseHeadway),
    myDistanceNoiseHeadwayCoeff(PerIvanDefaults::distanceNoiseHeadwayCoeff),
    mySpeedNoiseHeadwayCoeff(PerIvanDefaults::speedNoiseHeadwayCoeff),
    myOptimalSpeedRange(PerIvanDefaults::optimalSpeedRange),
    myPersistentDeltaVError(PerIvanDefaults::persistentDeltaVError),
    myMaxDeltaVError(PerIvanDefaults::maxDeltaVError),
    myDeltaVErrorShape(PerIvanDefaults::deltaVErrorShape),
    myMinDistanceNoiseDeltaV(PerIvanDefaults::minDistanceNoiseDeltaV),
    myMinSpeedNoiseDeltaV(PerIvanDefaults::minSpeedNoiseDeltaV),
    myDistanceNoiseDeltaVCoeff(PerIvanDefaults::distanceNoiseDeltaVCoeff),
    mySpeedNoiseDeltaVCoeff(PerIvanDefaults::speedNoiseDeltaVCoeff),
    myParam1(PerIvanDefaults::param1),
    myParam2(PerIvanDefaults::param2),
    myFreeSpeedErrorCoefficient(PerIvanDefaults::freeSpeedErrorCoefficient),
    myHeadwayChangePerceptionThreshold(PerIvanDefaults::headwayChangePerceptionThreshold),
    mySpeedDifferenceChangePerceptionThreshold(PerIvanDefaults::speedDifferenceChangePerceptionThreshold),
    myOriginalReactionTime(veh->getActionStepLengthSecs()),
    myMaximalReactionTime(PerIvanDefaults::maximalReactionTimeFactor* myOriginalReactionTime),
    myStepDuration(TS),
    myLastUpdateTime(SIMTIME - TS),
    myDebugLock(false) {
    updateError();
    updateReactionTime();
}

void
MSSimplePerIvan::update() {
    // Adapt step duration
    updateStepDuration();
    // Update error
    updateError();
    // Update actionStepLength, aka reaction time
    updateReactionTime();
}

void
MSSimplePerIvan::updateStepDuration() {
    myStepDuration = SIMTIME - myLastUpdateTime;
    myLastUpdateTime = SIMTIME;
}

void
MSSimplePerIvan::updateError() {
    myError.setTimeScale(myErrorTimeScaleCoefficient);
    myError.setNoiseIntensity(myErrorNoiseIntensityCoefficient);
    myError.step(myStepDuration);
}

void
MSSimplePerIvan::updateReactionTime() {
    myActionStepLength = myOriginalReactionTime + myPerceptionDelay;
}

double
MSSimplePerIvan::getPerceivedDistance(const double trueDistance, const double speed, const void* objID) {
    double accuracy = 500 * myMaximalPerceptionRange;
    //this means that if the vehicle is beyond my range, then i assign a very large error.
    double precisionDistance = myMinDistanceNoiseHeadway;
    double precisionSpeed = myMinSpeedNoiseHeadway;
    
    if (trueDistance <= myOptimalPerceptionRange) {
        accuracy = myPersistentHeadwayError;
    }
    else if (trueDistance <= myMaximalPerceptionRange) {
        accuracy = myPersistentHeadwayError + (((myMaxHeadwayError - myPersistentHeadwayError) / pow((myMaximalPerceptionRange - myOptimalPerceptionRange), myHeadwayErrorShape)) * pow((trueDistance - myOptimalPerceptionRange), myHeadwayErrorShape));
    }
    // distance precision
    if (trueDistance > myOptimalPerceptionRange) {
        precisionDistance = myMinDistanceNoiseHeadway + myDistanceNoiseHeadwayCoeff*(trueDistance - myOptimalPerceptionRange);
    }
    // speed precision
    if (speed > myOptimalSpeedRange) {
        precisionSpeed = myMinSpeedNoiseHeadway + mySpeedNoiseHeadwayCoeff * (speed - myOptimalSpeedRange);
    }
    double auxW = myError.getState();
    double auxWtrans = (2 / (1+exp(-auxW))) - 1;
    double precision = precisionDistance + precisionSpeed;   
    const double perceivedDistance = trueDistance + accuracy + (auxWtrans*precision);

    // store the current perceived distance to the object
    const auto lastPerceivedDistance = myPerceivedDistances.find(objID);
    double thisPerceivedSpeedDifference;
    if (lastPerceivedDistance == myPerceivedDistances.end()) {
        myPerceivedDistances[objID] = perceivedDistance;
        myPerceivedSpeedDifference[objID] = 500;//this is a large value in m/s for it not to be considered
    }
    else {
        thisPerceivedSpeedDifference = (perceivedDistance - lastPerceivedDistance->second) / myStepDuration;
        myPerceivedSpeedDifference[objID] = thisPerceivedSpeedDifference;
        myPerceivedDistances[objID] = perceivedDistance;
    }

    return perceivedDistance;
}

double
MSSimplePerIvan::getPerceivedSpeedDifference(const double trueSpeedDifference, const void* objID) {
    const auto perceivedSpeedDifference = myPerceivedSpeedDifference.find(objID);
    if (perceivedSpeedDifference == myPerceivedSpeedDifference.end()) {
        return trueSpeedDifference;
    }
    else {
        if (perceivedSpeedDifference->second > 1.2 * trueSpeedDifference || perceivedSpeedDifference->second < 0.8 * trueSpeedDifference) {
            return trueSpeedDifference;
        }
        else {
            return perceivedSpeedDifference->second;
        }
    }
}

/****************************************************************************/
