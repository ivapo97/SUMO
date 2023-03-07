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
SumoRNG OUProcess::myRNG("perivan");

// ===========================================================================
// Default value definitions
// ===========================================================================
//double TCIDefaults::myMinTaskCapability = 0.1;
//double TCIDefaults::myMaxTaskCapability = 10.0;
//double TCIDefaults::myMaxTaskDemand = 20.0;
//double TCIDefaults::myMaxDifficulty = 10.0;
//double TCIDefaults::mySubCriticalDifficultyCoefficient = 0.1;
//double TCIDefaults::mySuperCriticalDifficultyCoefficient = 1.0;
//double TCIDefaults::myOppositeDirectionDrivingFactor = 1.3;
//double TCIDefaults::myHomeostasisDifficulty = 1.5;
//double TCIDefaults::myCapabilityTimeScale = 0.5;
//double TCIDefaults::myAccelerationErrorTimeScaleCoefficient = 1.0;
//double TCIDefaults::myAccelerationErrorNoiseIntensityCoefficient = 1.0;
//double TCIDefaults::myActionStepLengthCoefficient = 1.0;
//double TCIDefaults::myMinActionStepLength = 0.0;
//double TCIDefaults::myMaxActionStepLength = 3.0;
//double TCIDefaults::mySpeedPerceptionErrorTimeScaleCoefficient = 1.0;
//double TCIDefaults::mySpeedPerceptionErrorNoiseIntensityCoefficient = 1.0;
//double TCIDefaults::myHeadwayPerceptionErrorTimeScaleCoefficient = 1.0;
//double TCIDefaults::myHeadwayPerceptionErrorNoiseIntensityCoefficient = 1.0;

double PerIvanDefaults::minAwareness = 0.1;
double PerIvanDefaults::initialAwareness = 1.0;
double PerIvanDefaults::errorTimeScaleCoefficient = 100.0;
double PerIvanDefaults::errorNoiseIntensityCoefficient = 0.2;
double PerIvanDefaults::speedDifferenceErrorCoefficient = 0.15;
double PerIvanDefaults::headwayErrorCoefficient = 0.75;
double PerIvanDefaults::freeSpeedErrorCoefficient = 0.0;
double PerIvanDefaults::speedDifferenceChangePerceptionThreshold = 0.1;
double PerIvanDefaults::headwayChangePerceptionThreshold = 0.1;
double PerIvanDefaults::maximalReactionTimeFactor = 1.0;


// ===========================================================================
// method definitions
// ===========================================================================

OUProcess::OUProcess(double initialState, double timeScale, double noiseIntensity)
    : myState(initialState),
      myTimeScale(timeScale),
      myNoiseIntensity(noiseIntensity) {}


OUProcess::~OUProcess() {}


void
OUProcess::step(double dt) {
#ifdef DEBUG_OUPROCESS
    const double oldstate = myState;
#endif
    myState = exp(-dt / myTimeScale) * myState + myNoiseIntensity * sqrt(2 * dt / myTimeScale) * RandHelper::randNorm(0, 1, &myRNG);
#ifdef DEBUG_OUPROCESS
    std::cout << "  OU-step (" << dt << " s.): " << oldstate << "->" << myState << std::endl;
#endif
}

double
OUProcess::step(double state, double dt, double timeScale, double noiseIntensity) {
    /// see above
    return exp(-dt / timeScale) * state + noiseIntensity * sqrt(2 * dt / timeScale) * RandHelper::randNorm(0, 1, &myRNG);
}

double
OUProcess::getState() const {
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
#ifdef DEBUG_AWARENESS
    if (DEBUG_COND) {
        std::cout << SIMTIME << " stepDuration=" << myStepDuration << ", error=" << myError.getState() << std::endl;
    }
#endif
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
#ifdef DEBUG_AWARENESS
    if (DEBUG_COND) {
        std::cout << SIMTIME << " veh=" << myVehicle->getID() << ", setAwareness(" << MAX2(value, myMinAwareness) << ")" << std::endl;
    }
#endif
    myAwareness = MAX2(value, myMinAwareness);
    if (myAwareness == 1.) {
        myError.setState(0.);
    }
    updateReactionTime();
}


double
MSSimplePerIvan::getPerceivedOwnSpeed(double speed) {
    return speed + myFreeSpeedErrorCoefficient * myError.getState() * sqrt(speed);
}


double
MSSimplePerIvan::getPerceivedHeadway(const double trueGap, const void* objID) {
#ifdef DEBUG_PERCEPTION_ERRORS
    if (DEBUG_COND) {
        if (!debugLocked()) {
            std::cout << SIMTIME << " getPerceivedHeadway() for veh '" << myVehicle->getID() << "'\n"
                << "    trueGap=" << trueGap << " objID=" << objID << std::endl;
        }
    }
#endif

    const double perceivedGap = trueGap + myHeadwayErrorCoefficient * myError.getState() * trueGap;
    const auto assumedGap = myAssumedGap.find(objID);
    if (assumedGap == myAssumedGap.end()
        || fabs(perceivedGap - assumedGap->second) > myHeadwayChangePerceptionThreshold * trueGap * (1.0 - myAwareness)) {

#ifdef DEBUG_PERCEPTION_ERRORS
        if (!debugLocked()) {
            std::cout << "    new perceived gap (=" << perceivedGap << ") differs significantly from the assumed (="
                << (assumedGap == myAssumedGap.end() ? "NA" : toString(assumedGap->second)) << ")" << std::endl;
        }
#endif

        // new perceived gap differs significantly from the previous
        myAssumedGap[objID] = perceivedGap;
        return perceivedGap;
    }
    else {

#ifdef DEBUG_PERCEPTION_ERRORS
        if (DEBUG_COND) {
            if (!debugLocked()) {
                std::cout << "    new perceived gap (=" << perceivedGap << ") does *not* differ significantly from the assumed (="
                    << (assumedGap->second) << ")" << std::endl;
            }
        }
#endif
        // new perceived gap doesn't differ significantly from the previous
        return myAssumedGap[objID];
    }
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
MSSimplePerIvan::getPerceivedSpeedDifference(const double trueSpeedDifference, const double trueGap, const void* objID) {
#ifdef DEBUG_PERCEPTION_ERRORS
    if (DEBUG_COND) {
        if (!debugLocked()) {
            std::cout << SIMTIME << " getPerceivedSpeedDifference() for veh '" << myVehicle->getID() << "'\n"
                << "    trueGap=" << trueGap << " trueSpeedDifference=" << trueSpeedDifference << " objID=" << objID << std::endl;
        }
    }
#endif
    const double perceivedSpeedDifference = trueSpeedDifference + mySpeedDifferenceErrorCoefficient * myError.getState() * trueGap;
    const auto lastPerceivedSpeedDifference = myLastPerceivedSpeedDifference.find(objID);
    if (lastPerceivedSpeedDifference == myLastPerceivedSpeedDifference.end()
        || fabs(perceivedSpeedDifference - lastPerceivedSpeedDifference->second) > mySpeedDifferenceChangePerceptionThreshold * trueGap * (1.0 - myAwareness)) {

#ifdef DEBUG_PERCEPTION_ERRORS
        if (DEBUG_COND) {
            if (!debugLocked()) {
                std::cout << "    new perceived speed difference (=" << perceivedSpeedDifference << ") differs significantly from the last perceived (="
                    << (lastPerceivedSpeedDifference == myLastPerceivedSpeedDifference.end() ? "NA" : toString(lastPerceivedSpeedDifference->second)) << ")"
                    << std::endl;
            }
        }
#endif

        // new perceived speed difference differs significantly from the previous
        myLastPerceivedSpeedDifference[objID] = perceivedSpeedDifference;
        return perceivedSpeedDifference;
    }
    else {
#ifdef DEBUG_PERCEPTION_ERRORS
        if (!debugLocked()) {
            std::cout << "    new perceived speed difference (=" << perceivedSpeedDifference << ") does *not* differ significantly from the last perceived (="
                << (lastPerceivedSpeedDifference->second) << ")" << std::endl;
        }
#endif
        // new perceived speed difference doesn't differ significantly from the previous
        return lastPerceivedSpeedDifference->second;
    }
}




/****************************************************************************/
