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

/* -------------------------------------------------------------------------
 * static member definitions
 * ----------------------------------------------------------------------- */
SumoRNG WienerProcess::myRNG("perivan");

// ===========================================================================
// Default value definitions
// ===========================================================================

double PerIvanDefaults::timeCorrelationWindow = 25.0;
double PerIvanDefaults::perceptionDelay = 0.0;
double PerIvanDefaults::minDistanceError = 0.0;
double PerIvanDefaults::optimalPerceptionDistance = 50.0;
double PerIvanDefaults::maximalPerceptionDistance = 150.0;
double PerIvanDefaults::maxDistanceError = 5.0;
double PerIvanDefaults::distanceErrorShape = 1.0;
double PerIvanDefaults::minDistancePrecision = 0.1;
double PerIvanDefaults::minSpeedPrecision = 0.1;
double PerIvanDefaults::distancePrecisionCoeff = 0.001;
double PerIvanDefaults::speedPrecisionCoeff = 0.001;
double PerIvanDefaults::optimalPerceptionSpeed = 10.0;
double PerIvanDefaults::param1 = 1.0;
double PerIvanDefaults::param2 = 1.0;

// ===========================================================================
// method definitions
// ===========================================================================

WienerProcess::WienerProcess(double initialState, double timeScale)
    : myState(initialState),
    myTimeScale(timeScale) {}

WienerProcess::~WienerProcess() {}

void
WienerProcess::step(double dt) {
    myState = exp(-dt / myTimeScale) * myState + sqrt(2 * dt / myTimeScale) * RandHelper::randNorm(0, 1, &myRNG);
}

double
WienerProcess::step(double state, double dt, double timeScale) {
    /// see above
    return exp(-dt / timeScale) * state + sqrt(2 * dt / timeScale) * RandHelper::randNorm(0, 1, &myRNG);
}

double
WienerProcess::getState() const {
    return myState;
}

MSSimplePerIvan::MSSimplePerIvan(MSVehicle* veh) :
    myVehicle(veh),
    myError(0., 25.),
    myTimeCorrelationWindow(PerIvanDefaults::timeCorrelationWindow),
    myPerceptionDelay(PerIvanDefaults::perceptionDelay),
    myMinDistanceError(PerIvanDefaults::minDistanceError),  
    myOptimalPerceptionDistance(PerIvanDefaults::optimalPerceptionDistance),
    myMaximalPerceptionDistance(PerIvanDefaults::maximalPerceptionDistance),
    myMaxDistanceError(PerIvanDefaults::maxDistanceError),  
    myDistanceErrorShape(PerIvanDefaults::distanceErrorShape),
    myMinDistancePrecision(PerIvanDefaults::minDistancePrecision),
    myMinSpeedPrecision(PerIvanDefaults::minSpeedPrecision),
    myDistancePrecisionCoeff(PerIvanDefaults::distancePrecisionCoeff),
    mySpeedPrecisionCoeff(PerIvanDefaults::speedPrecisionCoeff),
    myOptimalPerceptionSpeed(PerIvanDefaults::optimalPerceptionSpeed),
    myParam1(PerIvanDefaults::param1),
    myParam2(PerIvanDefaults::param2),
    myOriginalReactionTime(veh->getActionStepLengthSecs()),
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
    myError.setTimeScale(myTimeCorrelationWindow);
    myError.step(myStepDuration);
}

void
MSSimplePerIvan::updateReactionTime() {
    myActionStepLength = myOriginalReactionTime + myPerceptionDelay;
}

double
MSSimplePerIvan::getPerceivedDistance(const double trueDistance, const double speed, const void* objID) {
    double accuracy = 500 * myMaximalPerceptionDistance;
    //this means that if the vehicle is beyond my range, then i assign a very large error.
    double precisionDistance = myMinDistancePrecision;
    double precisionSpeed = myMinSpeedPrecision;
    
    if (trueDistance <= myOptimalPerceptionDistance) {
        accuracy = myMinDistanceError;
    }
    else if (trueDistance <= myMaximalPerceptionDistance) {
        accuracy = myMinDistanceError + (((myMaxDistanceError - myMinDistanceError) / pow((myMaximalPerceptionDistance - myOptimalPerceptionDistance), myDistanceErrorShape)) * pow((trueDistance - myOptimalPerceptionDistance), myDistanceErrorShape));
    }
    // distance precision
    if (trueDistance > myOptimalPerceptionDistance) {
        precisionDistance = myMinDistancePrecision + myDistancePrecisionCoeff*(trueDistance - myOptimalPerceptionDistance);
    }
    // speed precision
    if (speed > myOptimalPerceptionSpeed) {
        precisionSpeed = myMinSpeedPrecision + mySpeedPrecisionCoeff * (speed - myOptimalPerceptionSpeed);
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
