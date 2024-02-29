/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2013-2023 German Aerospace Center (DLR) and others.
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
/// @file    MSDevice_PerIvan.cpp
/// @author  Leonhard Luecken
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    15.06.2018
///
/// The Per Ivan Device mainly provides a configuration and interaction interface for the vehicle's per ivan.
/// @see microsim/MSPerIvan.h
///
/****************************************************************************/
#include <config.h>

#include <utils/common/StringUtils.h>
#include <utils/options/OptionsCont.h>
#include <utils/vehicle/SUMOVehicle.h>
#include <utils/common/WrappingCommand.h>
#include <utils/common/RGBColor.h>
#include <microsim/MSNet.h>
#include <microsim/MSVehicle.h>
#include <microsim/MSRouteHandler.h>
#include <microsim/MSVehicleControl.h>
#include <microsim/MSEventControl.h>
#include <microsim/MSPerIvan.h>
#include "MSDevice_PerIvan.h"


// ===========================================================================
// debug constants
// ===========================================================================
//#define DEBUG_DSDEVICE
//#define DEBUG_COND (myHolder.isSelected())


// ===========================================================================
// parameter defaults
// ===========================================================================

// see PerIvanDefaults in MSPerIvan


// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// static initialisation methods
// ---------------------------------------------------------------------------
void
MSDevice_PerIvan::insertOptions(OptionsCont& oc) {
    oc.addOptionSubTopic("Per Ivan Device");
    insertDefaultAssignmentOptions("perivan", "Per Ivan Device", oc);
    oc.doRegister("device.perivan.timeCorrelationWindow", new Option_Float(PerIvanDefaults::timeCorrelationWindow));
    oc.addDescription("device.perivan.timeCorrelationWindow", TL("Per Ivan Device"), TL("Time correlation window for the wiener process."));
    oc.doRegister("device.perivan.perceptionDelay", new Option_Float(PerIvanDefaults::perceptionDelay));
    oc.addDescription("device.perivan.perceptionDelay", TL("Per Ivan Device"), TL("Perception delay."));
    oc.doRegister("device.perivan.minDistanceError", new Option_Float(PerIvanDefaults::minDistanceError));
    oc.addDescription("device.perivan.minDistanceError", TL("Per Ivan Device"), TL("Persistent distance accuracy error."));
    oc.doRegister("device.perivan.optimalPerceptionDistance", new Option_Float(PerIvanDefaults::optimalPerceptionDistance));
    oc.addDescription("device.perivan.optimalPerceptionDistance", TL("Per Ivan Device"), TL("Distance range without decay in distance estimation accuracy."));
    oc.doRegister("device.perivan.maximalPerceptionDistance", new Option_Float(PerIvanDefaults::maximalPerceptionDistance));
    oc.addDescription("device.perivan.maximalPerceptionDistance", TL("Per Ivan Device"), TL("Max. perception disntance range."));
    oc.doRegister("device.perivan.maxDistanceError", new Option_Float(PerIvanDefaults::maxDistanceError));
    oc.addDescription("device.perivan.maxDistanceError", TL("Per Ivan Device"), TL("Accuracy error at max. perception range."));  
    oc.doRegister("device.perivan.distanceErrorShape", new Option_Float(PerIvanDefaults::distanceErrorShape));
    oc.addDescription("device.perivan.distanceErrorShape", TL("Per Ivan Device"), TL("Distance error decay - '1: linear','2: quadratic', or '3 :ellipse'")); 
    oc.doRegister("device.perivan.minDistancePrecision", new Option_Float(PerIvanDefaults::minDistancePrecision));
    oc.addDescription("device.perivan.minDistancePrecision", TL("Per Ivan Device"), TL("Min (best) precision due to distance. "));
    oc.doRegister("device.perivan.minSpeedPrecision", new Option_Float(PerIvanDefaults::minSpeedPrecision));
    oc.addDescription("device.perivan.minSpeedPrecision", TL("Per Ivan Device"), TL("Min (best) precision due to speed."));
    oc.doRegister("device.perivan.distancePrecisionCoeff", new Option_Float(PerIvanDefaults::distancePrecisionCoeff));
    oc.addDescription("device.perivan.distancePrecisionCoeff", TL("Per Ivan Device"), TL("Precision rate decrease due to distance."));    
    oc.doRegister("device.perivan.speedPrecisionCoeff", new Option_Float(PerIvanDefaults::speedPrecisionCoeff));
    oc.addDescription("device.perivan.speedPrecisionCoeff", TL("Per Ivan Device"), TL("Precision rate decrease due to speed."));
    oc.doRegister("device.perivan.optimalPerceptionSpeed", new Option_Float(PerIvanDefaults::optimalPerceptionSpeed));
    oc.addDescription("device.perivan.optimalPerceptionSpeed", TL("Per Ivan Device"), TL("Speed range without decrease in precision due to speed."));
    oc.doRegister("device.perivan.param1", new Option_Float(PerIvanDefaults::param1));
    oc.addDescription("device.perivan.param1", TL("Per Ivan Device"), TL("auxiliary calibration parameter 1 - dev."));
    oc.doRegister("device.perivan.param2", new Option_Float(PerIvanDefaults::param2));
    oc.addDescription("device.perivan.param2", TL("Per Ivan Device"), TL("auxiliary calibration parameter 2 - dev."));
}

void
MSDevice_PerIvan::buildVehicleDevices(SUMOVehicle& v, std::vector<MSVehicleDevice*>& into) {
    OptionsCont& oc = OptionsCont::getOptions();
    if (equippedByDefaultAssignmentOptions(oc, "perivan", v, false)) {
        const double timeCorrelationWindow = getTimeCorrelationWindow(v, oc);
        const double perceptionDelay = getPerceptionDelay(v, oc);
        const double minDistanceError = getMinDistanceError(v, oc);
        const double optimalPerceptionDistance = getOptimalPerceptionDistance(v, oc);
        const double maximalPerceptionDistance = getMaximalPerceptionDistance(v, oc);
        const double maxDistanceError = getMaxDistanceError(v, oc);
        const double distanceErrorShape = getDistanceErrorShape(v, oc);
        const double minDistancePrecision = getMinDistancePrecision(v,oc);
        const double minSpeedPrecision = getMinSpeedPrecision(v,oc);
        const double distancePrecisionCoeff = getDistancePrecisionCoeff(v,oc);
        const double speedPrecisionCoeff = getSpeedPrecisionCoeff(v,oc);
        const double optimalPerceptionSpeed = getOptimalPerceptionSpeed(v, oc);
        const double param1 = getParam1(v, oc);
        const double param2 = getParam2(v, oc);
        // build the device
        MSDevice_PerIvan* device = new MSDevice_PerIvan(v, "perivan" + v.getID(),
            timeCorrelationWindow,
            perceptionDelay,
            minDistanceError,
            optimalPerceptionDistance,
            maximalPerceptionDistance,
            maxDistanceError,
            distanceErrorShape,
            minDistancePrecision,
            minSpeedPrecision,
            distancePrecisionCoeff,
            speedPrecisionCoeff,
            optimalPerceptionSpeed,
            param1,
            param2);
        into.push_back(device);
    }
}

double
MSDevice_PerIvan::getTimeCorrelationWindow(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.timeCorrelationWindow", PerIvanDefaults::timeCorrelationWindow, false);
}
double
MSDevice_PerIvan::getPerceptionDelay(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.perceptionDelay", PerIvanDefaults::perceptionDelay, false);
}
double
MSDevice_PerIvan::getMinDistanceError(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.minDistanceError", PerIvanDefaults::minDistanceError, false);
}
double
MSDevice_PerIvan::getOptimalPerceptionDistance(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.optimalPerceptionDistance", PerIvanDefaults::optimalPerceptionDistance, false);
}
double
MSDevice_PerIvan::getMaximalPerceptionDistance(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.maximalPerceptionDistance", PerIvanDefaults::maximalPerceptionDistance, false);
}
double
MSDevice_PerIvan::getMaxDistanceError(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.maxDistanceError", PerIvanDefaults::maxDistanceError, false);
}
double
MSDevice_PerIvan::getDistanceErrorShape(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.distanceErrorShape", PerIvanDefaults::distanceErrorShape, false);
}
double
MSDevice_PerIvan::getMinDistancePrecision(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.minDistancePrecision", PerIvanDefaults::minDistancePrecision, false);
}
double
MSDevice_PerIvan::getMinSpeedPrecision(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.minSpeedPrecision", PerIvanDefaults::minSpeedPrecision, false);
}
double
MSDevice_PerIvan::getDistancePrecisionCoeff(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.distancePrecisionCoeff", PerIvanDefaults::distancePrecisionCoeff, false);
}
double
MSDevice_PerIvan::getSpeedPrecisionCoeff(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.speedPrecisionCoeff", PerIvanDefaults::speedPrecisionCoeff, false);
}
double
MSDevice_PerIvan::getOptimalPerceptionSpeed(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.optimalPerceptionSpeed", PerIvanDefaults::optimalPerceptionSpeed, false);
}
double
MSDevice_PerIvan::getParam1(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.param1", PerIvanDefaults::param1, false);
}
double
MSDevice_PerIvan::getParam2(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.param2", PerIvanDefaults::param2, false);
}

// ---------------------------------------------------------------------------
// MSDevice_PerIvan-methods
// ---------------------------------------------------------------------------
MSDevice_PerIvan::MSDevice_PerIvan(SUMOVehicle& holder, const std::string& id,
    double timeCorrelationWindow,
    double perceptionDelay,
    double minDistanceError,
    double optimalPerceptionDistance,
    double maximalPerceptionDistance,
    double maxDistanceError,
    double distanceErrorShape,
    double minDistancePrecision,
    double minSpeedPrecision,
    double distancePrecisionCoeff,
    double speedPrecisionCoeff,
    double optimalPerceptionSpeed,
    double param1,
    double param2) :
    MSVehicleDevice(holder, id),
    myTimeCorrelationWindow(timeCorrelationWindow),
    myPerceptionDelay(perceptionDelay),
    myMinDistanceError(minDistanceError),
    myOptimalPerceptionDistance(optimalPerceptionDistance),
    myMaximalPerceptionDistance(maximalPerceptionDistance),
    myMaxDistanceError(maxDistanceError),
    myDistanceErrorShape(distanceErrorShape),
    myMinDistancePrecision(minDistancePrecision),
    myMinSpeedPrecision(minSpeedPrecision),
    myDistancePrecisionCoeff(distancePrecisionCoeff),
    mySpeedPrecisionCoeff(speedPrecisionCoeff),
    myOptimalPerceptionSpeed(optimalPerceptionSpeed),
    myParam1(param1),
    myParam2(param2){
    // Take care! Holder is currently being constructed. Cast occurs before completion.
    myHolderMS = static_cast<MSVehicle*>(&holder);
    initPerIvan();
}

void
MSDevice_PerIvan::initPerIvan() {
    myPerIvan = std::make_shared<MSSimplePerIvan>(myHolderMS);
    myPerIvan->setTimeCorrelationWindow(myTimeCorrelationWindow);
    myPerIvan->setPerceptionDelay(myPerceptionDelay);
    myPerIvan->setMinDistanceError(myMinDistanceError);
    myPerIvan->setOptimalPerceptionDistance(myOptimalPerceptionDistance);
    myPerIvan->setMaximalPerceptionDistance(myMaximalPerceptionDistance);   
    myPerIvan->setMaxDistanceError(myMaxDistanceError);
    myPerIvan->setDistanceErrorShape(myDistanceErrorShape);
    myPerIvan->setMinDistancePrecision(myMinDistancePrecision);
    myPerIvan->setMinSpeedPrecision(myMinSpeedPrecision);
    myPerIvan->setDistancePrecisionCoeff(myDistancePrecisionCoeff);
    myPerIvan->setSpeedPrecisionCoeff(mySpeedPrecisionCoeff);
    myPerIvan->setOptimalPerceptionSpeed(myOptimalPerceptionSpeed);
    myPerIvan->setParam1(myParam1);
    myPerIvan->setParam2(myParam2);
}

void
MSDevice_PerIvan::update() {
    myPerIvan->update();
}

std::string
MSDevice_PerIvan::getParameter(const std::string& key) const {
    if (key == "WienerProcessState") {
        return toString(myPerIvan->getWienerProcessState());
    }
    else if (key == "errorTimeScale") {
        return toString(myPerIvan->getErrorTimeScale());
    }
    else if (key == "timeCorrelationWindow") {
        return toString(myPerIvan->getTimeCorrelationWindow());
    }
    else if (key == "perceptionDelay") {
        return toString(myPerIvan->getPerceptionDelay());
    }
    else if (key == "minDistanceError") {
        return toString(myPerIvan->getMinDistanceError());
    }
    else if (key == "optimalPerceptionDistance") {
        return toString(myPerIvan->getOptimalPerceptionDistance());
    }
    else if (key == "maximalPerceptionDistance") {
        return toString(myPerIvan->getMaximalPerceptionDistance());
    } 
    else if (key == "maxDistanceError") {
        return toString(myPerIvan->getMaxDistanceError());
    }
    else if (key == "distanceErrorShape") {
        return toString(myPerIvan->getDistanceErrorShape());
    } 
    else if (key == "minDistancePrecision") {
        return toString(myPerIvan->getMinDistancePrecision());
    }
    else if (key == "minSpeedPrecision") {
        return toString(myPerIvan->getMinSpeedPrecision());
    }
    else if (key == "distancePrecisionCoeff") {
        return toString(myPerIvan->getDistancePrecisionCoeff());
    }
    else if (key == "speedPrecisionCoeff") {
        return toString(myPerIvan->getSpeedPrecisionCoeff());
    }
    else if (key == "optimalPerceptionSpeed") {
        return toString(myPerIvan->getOptimalPerceptionSpeed());
    }
    else if (key == "param1") {
        return toString(myPerIvan->getParam1());
    }
    else if (key == "param2") {
        return toString(myPerIvan->getParam2());
    }
    else if (key == "originalReactionTime") {
        return toString(myPerIvan->getOriginalReactionTime());
    }
    else if (key == "actionStepLength") {
        return toString(myPerIvan->getActionStepLength());
    }
    throw InvalidArgument("Parameter '" + key + "' is not supported for device of type '" + deviceName() + "'");
}

void
MSDevice_PerIvan::setParameter(const std::string& key, const std::string& value) {
    if (key == "WienerProcessState") {
        myPerIvan->setWienerProcessState(StringUtils::toDouble(value));
    }
    else if (key == "errorTimeScale") {
        myPerIvan->setErrorTimeScale(StringUtils::toDouble(value));
    }
    else if (key == "timeCorrelationWindow") {
        myPerIvan->setTimeCorrelationWindow(StringUtils::toDouble(value));
    }
    else if (key == "perceptionDelay") {
        myPerIvan->setPerceptionDelay(StringUtils::toDouble(value));
    }
    else if (key == "minDistanceError") {
        myPerIvan->setMinDistanceError(StringUtils::toDouble(value));
    } 
    else if (key == "optimalPerceptionDistance") {
        myPerIvan->setOptimalPerceptionDistance(StringUtils::toDouble(value));
    }
    else if (key == "maximalPerceptionDistance") {
        myPerIvan->setMaximalPerceptionDistance(StringUtils::toDouble(value));
    }
    else if (key == "maxDistanceError") {
        myPerIvan->setMaxDistanceError(StringUtils::toDouble(value));
    }
    else if (key == "distanceErrorShape") {
        myPerIvan->setDistanceErrorShape(StringUtils::toDouble(value));
    }
    else if (key == "minDistancePrecision") {
        myPerIvan->setMinDistancePrecision(StringUtils::toDouble(value));
    }
    else if (key == "minSpeedPrecision") {
        myPerIvan->setMinSpeedPrecision(StringUtils::toDouble(value));
    }
    else if (key == "distancePrecisionCoeff") {
        myPerIvan->setDistancePrecisionCoeff(StringUtils::toDouble(value));
    }
    else if (key == "speedPrecisionCoeff") {
        myPerIvan->setSpeedPrecisionCoeff(StringUtils::toDouble(value));
    }
    else if (key == "optimalPerceptionSpeed") {
        myPerIvan->setOptimalPerceptionSpeed(StringUtils::toDouble(value));
    }
    else if (key == "param1") {
        myPerIvan->setParam1(StringUtils::toDouble(value));
    }
    else if (key == "param2") {
        myPerIvan->setParam2(StringUtils::toDouble(value));
    }
    else if (key == "originalReactionTime") {
        myPerIvan->setOriginalReactionTime(StringUtils::toDouble(value));
    }
    else {
        throw InvalidArgument("Parameter '" + key + "' is not supported for device of type '" + deviceName() + "'");
    }
}

/****************************************************************************/
