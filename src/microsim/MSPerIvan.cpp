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
//#define DEBUG_AWARENESS
//#define DEBUG_PERCEPTION_ERRORS
//#define DEBUG_DRIVERSTATE
#define DEBUG_COND (true)
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

double PerIvanDefaults::minAwareness = 0.1;
double PerIvanDefaults::initialAwareness = 1.0;
double PerIvanDefaults::errorTimeScaleCoefficient = 100.0;
double PerIvanDefaults::errorNoiseIntensityCoefficient = 0.2;
double PerIvanDefaults::speedDifferenceErrorCoefficient = 0.15;
double PerIvanDefaults::headwayErrorCoefficient = 0.75;
double PerIvanDefaults::persistentHeadwayError = 0.0;
double PerIvanDefaults::optimalPerceptionRange = 50.0;
double PerIvanDefaults::maximalPerceptionRange = 150.0;
double PerIvanDefaults::maxHeadwayError = 5.0; ///@todo: ensure this value is larger than persistentHeadwayError  
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
#ifdef DEBUG_OUPROCESS
    const double oldstate = myState;
#endif
    myState = exp(-dt / myTimeScale) * myState + myNoiseIntensity * sqrt(2 * dt / myTimeScale) * RandHelper::randNorm(0, 1, &myRNG);
#ifdef DEBUG_OUPROCESS
    std::cout << "  OU-step (" << dt << " s.): " << oldstate << "->" << myState << std::endl;
#endif
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
    myAwareness(1.),
    myMinAwareness(PerIvanDefaults::minAwareness),
    myError(0., 1., 1.),
    myErrorTimeScaleCoefficient(PerIvanDefaults::errorTimeScaleCoefficient),
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
    myFreeSpeedErrorCoefficient(PerIvanDefaults::freeSpeedErrorCoefficient),
    myHeadwayChangePerceptionThreshold(PerIvanDefaults::headwayChangePerceptionThreshold),
    mySpeedDifferenceChangePerceptionThreshold(PerIvanDefaults::speedDifferenceChangePerceptionThreshold),
    myOriginalReactionTime(veh->getActionStepLengthSecs()),
    myMaximalReactionTime(PerIvanDefaults::maximalReactionTimeFactor* myOriginalReactionTime),
    //    myActionStepLength(TS),
    myStepDuration(TS),
    myLastUpdateTime(SIMTIME - TS),
    myDebugLock(false) {
#ifdef DEBUG_DRIVERSTATE
    std::cout << "Constructing per ivan for veh '" << veh->getID() << "'." << std::endl;
#endif
    updateError();
    updateReactionTime();
}


void
MSSimplePerIvan::update() {
#ifdef DEBUG_AWARENESS
    if (DEBUG_COND) {
        std::cout << SIMTIME << " veh=" << myVehicle->getID() << ", DriverState::update()" << std::endl;
    }
#endif
    // Adapt step duration
    updateStepDuration();
    // Update error
    updateError();
    // Update actionStepLength, aka reaction time
    updateReactionTime();
    // Update assumed gaps
    updateAssumedGaps();
}

void
MSSimplePerIvan::updateStepDuration() {
    myStepDuration = SIMTIME - myLastUpdateTime;
    myLastUpdateTime = SIMTIME;
}

void
MSSimplePerIvan::updateError() {
    if (myAwareness == 1.0 || myAwareness == 0.0) {
        myError.setState(0.);
    }
    else {
        myError.setTimeScale(myErrorTimeScaleCoefficient * myAwareness);
        myError.setNoiseIntensity(myErrorNoiseIntensityCoefficient * (1. - myAwareness));
        myError.step(myStepDuration);
    }
}

void
MSSimplePerIvan::updateReactionTime() {
    if (myAwareness == 1.0 || myAwareness == 0.0) {
        myActionStepLength = myOriginalReactionTime;
    }
    else {
        const double theta = (myAwareness - myMinAwareness) / (1.0 - myMinAwareness);
        myActionStepLength = myOriginalReactionTime + theta * (myMaximalReactionTime - myOriginalReactionTime);
        // Round to multiple of simstep length
        int quotient;
        remquo(myActionStepLength, TS, &quotient);
        myActionStepLength = TS * MAX2(quotient, 1);
    }
}

void
MSSimplePerIvan::setAwareness(const double value) {
    assert(value >= 0.);
    assert(value <= 1.);
    myAwareness = MAX2(value, myMinAwareness);
    if (myAwareness == 1.) {
        myError.setState(0.);
    }
    updateReactionTime();
}


double
MSSimplePerIvan::getPerceivedOwnSpeed(double speed) {
    //return speed + myFreeSpeedErrorCoefficient * myError.getState() * sqrt(speed);
    return speed;
}


double
MSSimplePerIvan::getPerceivedHeadway(const double trueGap, const double speed, const void* objID) {
    double headwayAccuracy = 50 * myMaximalPerceptionRange;

    if (trueGap<=myOptimalPerceptionRange) {
        headwayAccuracy = myPersistentHeadwayError;
    }
    else if (trueGap<=myMaximalPerceptionRange) {
        if (myHeadwayErrorShape == 1.0) { //linear
            headwayAccuracy = (myMaxHeadwayError - myPersistentHeadwayError) * ((trueGap - myMaxHeadwayError) / (myMaximalPerceptionRange - myOptimalPerceptionRange)) + myPersistentHeadwayError;
        }
        else if (myHeadwayErrorShape == 2.0) { //quadratic
            headwayAccuracy = (myMaxHeadwayError - myPersistentHeadwayError) * pow(((trueGap - myMaxHeadwayError) / (myMaximalPerceptionRange - myOptimalPerceptionRange)), 2) + myPersistentHeadwayError;
        }
        else if (myHeadwayErrorShape == 3.0) { //ellipse
            headwayAccuracy = (myMaxHeadwayError - myPersistentHeadwayError) * (1 - sqrt(1 - pow(((trueGap - myMaxHeadwayError) / (myMaximalPerceptionRange - myOptimalPerceptionRange)), 2))) + myPersistentHeadwayError;
        }
    }

    double headwayDistancePrecision = myMinDistanceNoiseHeadway;
    double headwaySpeedPrecision = myMinSpeedNoiseHeadway;
    // distance precision (noise)
    if (trueGap > myOptimalPerceptionRange) {
        headwayDistancePrecision = myMinDistanceNoiseHeadway + myDistanceNoiseHeadwayCoeff*(trueGap - myOptimalPerceptionRange);
    }
    // speed precision (noise)
    if (speed > myOptimalSpeedRange) {
        headwaySpeedPrecision = myMinSpeedNoiseHeadway + mySpeedNoiseHeadwayCoeff * (speed - myOptimalSpeedRange);
    }
   
    double headwayPrecision = headwayDistancePrecision + headwaySpeedPrecision;
   
    const double perceivedHeadway = trueGap + headwayAccuracy + myParam1*myError.getState()*headwayPrecision;
        return perceivedHeadway;
}

void
MSSimplePerIvan::updateAssumedGaps() {
    for (auto& p : myAssumedGap) {
        const void* objID = p.first;
        const auto speedDiff = myLastPerceivedSpeedDifference.find(objID);
        double assumedSpeedDiff;
        if (speedDiff != myLastPerceivedSpeedDifference.end()) {
            // update the assumed gap with the last perceived speed difference
            assumedSpeedDiff = speedDiff->second;
        }
        else {
            // Assume the object is not moving, if no perceived speed difference is known.
            assumedSpeedDiff = -myVehicle->getSpeed();
        }
        p.second += SPEED2DIST(assumedSpeedDiff);
    }
}

double
MSSimplePerIvan::getPerceivedSpeedDifference(const double trueSpeedDifference, const double trueGap, const double speed, const void* objID) {
    double deltaVAccuracy = 0;

    if (trueGap <= myOptimalPerceptionRange) {
        deltaVAccuracy = myPersistentDeltaVError;
    }
    else if (trueGap <= myMaximalPerceptionRange) {
        if (myDeltaVErrorShape == 1.0) { //linear
            deltaVAccuracy = (myMaxDeltaVError - myPersistentDeltaVError) * ((trueGap - myMaxDeltaVError) / (myMaximalPerceptionRange - myOptimalPerceptionRange)) + myPersistentDeltaVError;
        }
        else if (myDeltaVErrorShape == 2.0) { //quadratic
            deltaVAccuracy = (myMaxDeltaVError - myPersistentDeltaVError) * pow(((trueGap - myMaxDeltaVError) / (myMaximalPerceptionRange - myOptimalPerceptionRange)), 2) + myPersistentDeltaVError;
        }
        else if (myDeltaVErrorShape == 3.0) { //ellipse
            deltaVAccuracy = (myMaxDeltaVError - myPersistentDeltaVError) * (1 - sqrt(1 - pow(((trueGap - myMaxDeltaVError) / (myMaximalPerceptionRange - myOptimalPerceptionRange)), 2))) + myPersistentDeltaVError;
        }
    }

    double deltaVDistancePrecision = myMinDistanceNoiseDeltaV;
    double deltaVSpeedPrecision = myMinSpeedNoiseDeltaV;
    // distance precision (noise)
    if (trueGap > myOptimalPerceptionRange) {
        deltaVDistancePrecision = myMinDistanceNoiseDeltaV + myDistanceNoiseDeltaVCoeff * (trueGap - myOptimalPerceptionRange);
    }
    // speed precision (noise)
    if (speed > myOptimalSpeedRange) {
        deltaVSpeedPrecision = myMinSpeedNoiseDeltaV + mySpeedNoiseDeltaVCoeff * (speed - myOptimalSpeedRange);
    }

    double deltaVPrecision = deltaVDistancePrecision + deltaVSpeedPrecision;

    const double perceivedDeltaV = trueSpeedDifference + deltaVAccuracy + myParam1 * myError.getState() * deltaVPrecision;
    return perceivedDeltaV;
}




/****************************************************************************/
