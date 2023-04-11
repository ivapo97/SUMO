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
    oc.doRegister("device.perivan.initialAwareness", new Option_Float(PerIvanDefaults::initialAwareness));
    oc.addDescription("device.perivan.initialAwareness", TL("Per Ivan Device"), TL("Initial value assigned to the driver's awareness."));
    oc.doRegister("device.perivan.errorTimeScaleCoefficient", new Option_Float(PerIvanDefaults::errorTimeScaleCoefficient));
    oc.addDescription("device.perivan.errorTimeScaleCoefficient", TL("Per Ivan Device"), TL("Time scale for the error process."));
    oc.doRegister("device.perivan.errorNoiseIntensityCoefficient", new Option_Float(PerIvanDefaults::errorNoiseIntensityCoefficient));
    oc.addDescription("device.perivan.errorNoiseIntensityCoefficient", TL("Per Ivan Device"), TL("Noise intensity driving the error process."));
    oc.doRegister("device.perivan.speedDifferenceErrorCoefficient", new Option_Float(PerIvanDefaults::speedDifferenceErrorCoefficient));
    oc.addDescription("device.perivan.speedDifferenceErrorCoefficient", TL("Per Ivan Device"), TL("General scaling coefficient for applying the error to the perceived speed difference (error also scales with distance)."));
    oc.doRegister("device.perivan.headwayErrorCoefficient", new Option_Float(PerIvanDefaults::headwayErrorCoefficient));
    oc.addDescription("device.perivan.headwayErrorCoefficient", TL("Per Ivan Device"), TL("General scaling coefficient for applying the error to the perceived distance (error also scales with distance)."));
    oc.doRegister("device.perivan.persistentHeadwayError", new Option_Float(PerIvanDefaults::persistentHeadwayError));
    oc.addDescription("device.perivan.persistentHeadwayError", TL("Per Ivan Device"), TL("Persistent headway error."));
    oc.doRegister("device.perivan.optimalPerceptionRange", new Option_Float(PerIvanDefaults::optimalPerceptionRange));
    oc.addDescription("device.perivan.optimalPerceptionRange", TL("Per Ivan Device"), TL("Headway range without increase in errors."));
    oc.doRegister("device.perivan.maximalPerceptionRange", new Option_Float(PerIvanDefaults::maximalPerceptionRange));
    oc.addDescription("device.perivan.maximalPerceptionRange", TL("Per Ivan Device"), TL("Max. headway range."));
    oc.doRegister("device.perivan.maxHeadwayError", new Option_Float(PerIvanDefaults::maxHeadwayError));
    oc.addDescription("device.perivan.maxHeadwayError", TL("Per Ivan Device"), TL("Headway error at max. perception range."));  
    oc.doRegister("device.perivan.headwayErrorShape", new Option_Float(PerIvanDefaults::headwayErrorShape));
    oc.addDescription("device.perivan.headwayErrorShape", TL("Per Ivan Device"), TL("Shape of headway error function - '1: linear','2: quadratic', or '3 :ellipse'")); 
    oc.doRegister("device.perivan.minDistanceNoiseHeadway", new Option_Float(PerIvanDefaults::minDistanceNoiseHeadway));
    oc.addDescription("device.perivan.minDistanceNoiseHeadway", TL("Per Ivan Device"), TL("Min distance noise on headway."));
    oc.doRegister("device.perivan.minSpeedNoiseHeadway", new Option_Float(PerIvanDefaults::minSpeedNoiseHeadway));
    oc.addDescription("device.perivan.minSpeedNoiseHeadway", TL("Per Ivan Device"), TL("Min speed noise on headway."));
    oc.doRegister("device.perivan.distanceNoiseHeadwayCoeff", new Option_Float(PerIvanDefaults::distanceNoiseHeadwayCoeff));
    oc.addDescription("device.perivan.distanceNoiseHeadwayCoeff", TL("Per Ivan Device"), TL("Distance noise rate coefficient on headway."));    
    oc.doRegister("device.perivan.speedNoiseHeadwayCoeff", new Option_Float(PerIvanDefaults::speedNoiseHeadwayCoeff));
    oc.addDescription("device.perivan.speedNoiseHeadwayCoeff", TL("Per Ivan Device"), TL("Speed noise rate coefficient on headway."));
    oc.doRegister("device.perivan.optimalSpeedRange", new Option_Float(PerIvanDefaults::optimalSpeedRange));
    oc.addDescription("device.perivan.optimalSpeedRange", TL("Per Ivan Device"), TL("Speed range without increase in noise."));
    oc.doRegister("device.perivan.persistentDeltaVError", new Option_Float(PerIvanDefaults::persistentDeltaVError));
    oc.addDescription("device.perivan.persistentDeltaVError", TL("Per Ivan Device"), TL("Persistent deltaV error."));
    oc.doRegister("device.perivan.maxDeltaVError", new Option_Float(PerIvanDefaults::maxDeltaVError));
    oc.addDescription("device.perivan.maxDeltaVError", TL("Per Ivan Device"), TL("DeltaV error at max. perception range."));
    oc.doRegister("device.perivan.deltaVErrorShape", new Option_Float(PerIvanDefaults::deltaVErrorShape));
    oc.addDescription("device.perivan.deltaVErrorShape", TL("Per Ivan Device"), TL("Shape of deltaV error function - '1: linear','2: quadratic', or '3 :ellipse'"));
    oc.doRegister("device.perivan.minDistanceNoiseDeltaV", new Option_Float(PerIvanDefaults::minDistanceNoiseDeltaV));
    oc.addDescription("device.perivan.minDistanceNoiseDeltaV", TL("Per Ivan Device"), TL("Min distance noise on deltaV."));
    oc.doRegister("device.perivan.minSpeedNoiseDeltaV", new Option_Float(PerIvanDefaults::minSpeedNoiseDeltaV));
    oc.addDescription("device.perivan.minSpeedNoiseDeltaV", TL("Per Ivan Device"), TL("Min speed noise on deltaV."));
    oc.doRegister("device.perivan.distanceNoiseDeltaVCoeff", new Option_Float(PerIvanDefaults::distanceNoiseDeltaVCoeff));
    oc.addDescription("device.perivan.distanceNoiseDeltaVCoeff", TL("Per Ivan Device"), TL("Distance noise rate coefficient on deltaV."));
    oc.doRegister("device.perivan.speedNoiseDeltaVCoeff", new Option_Float(PerIvanDefaults::speedNoiseDeltaVCoeff));
    oc.addDescription("device.perivan.speedNoiseDeltaVCoeff", TL("Per Ivan Device"), TL("Speed noise rate coefficient on deltaV."));
    oc.doRegister("device.perivan.param1", new Option_Float(PerIvanDefaults::param1));
    oc.addDescription("device.perivan.param1", TL("Per Ivan Device"), TL("calibration parameter 1 - scale precision."));
    oc.doRegister("device.perivan.freeSpeedErrorCoefficient", new Option_Float(PerIvanDefaults::freeSpeedErrorCoefficient));
    oc.addDescription("device.perivan.freeSpeedErrorCoefficient", TL("Per Ivan Device"), TL("General scaling coefficient for applying the error to the vehicle's own speed when driving without a leader (error also scales with own speed)."));
    oc.doRegister("device.perivan.speedDifferenceChangePerceptionThreshold", new Option_Float(PerIvanDefaults::speedDifferenceChangePerceptionThreshold));
    oc.addDescription("device.perivan.speedDifferenceChangePerceptionThreshold", TL("Per Ivan Device"), TL("Base threshold for recognizing changes in the speed difference (threshold also scales with distance)."));
    oc.doRegister("device.perivan.headwayChangePerceptionThreshold", new Option_Float(PerIvanDefaults::headwayChangePerceptionThreshold));
    oc.addDescription("device.perivan.headwayChangePerceptionThreshold", TL("Per Ivan Device"), TL("Base threshold for recognizing changes in the headway (threshold also scales with distance)."));
    oc.doRegister("device.perivan.minAwareness", new Option_Float(PerIvanDefaults::minAwareness));
    oc.addDescription("device.perivan.minAwareness", TL("Per Ivan Device"), TL("Minimal admissible value for the driver's awareness."));
    oc.doRegister("device.perivan.maximalReactionTime", new Option_Float(-1.0));
    oc.addDescription("device.perivan.maximalReactionTime", TL("Per Ivan Device"), TL("Maximal reaction time (~action step length) induced by decreased awareness level (reached for awareness=minAwareness)."));
}


void
MSDevice_PerIvan::buildVehicleDevices(SUMOVehicle& v, std::vector<MSVehicleDevice*>& into) {
    OptionsCont& oc = OptionsCont::getOptions();
    ///@todo: what is this about? Ivan
    // ToC device implies this device 
    if (equippedByDefaultAssignmentOptions(oc, "perivan", v, false) || equippedByDefaultAssignmentOptions(oc, "toc", v, false)) {
        const double minAwareness = getMinAwareness(v, oc);
        const double initialAwareness = getInitialAwareness(v, oc);
        const double errorTimeScaleCoefficient = getErrorTimeScaleCoefficient(v, oc);
        const double errorNoiseIntensityCoefficient = getErrorNoiseIntensityCoefficient(v, oc);
        const double speedDifferenceErrorCoefficient = getSpeedDifferenceErrorCoefficient(v, oc);
        const double speedDifferenceChangePerceptionThreshold = getSpeedDifferenceChangePerceptionThreshold(v, oc);
        const double headwayChangePerceptionThreshold = getHeadwayChangePerceptionThreshold(v, oc);
        const double headwayErrorCoefficient = getHeadwayErrorCoefficient(v, oc);
        const double persistentHeadwayError = getPersistentHeadwayError(v, oc);
        const double optimalPerceptionRange = getOptimalPerceptionRange(v, oc);
        const double maximalPerceptionRange = getMaximalPerceptionRange(v, oc);
        const double maxHeadwayError = getMaxHeadwayError(v, oc);
        const double headwayErrorShape = getHeadwayErrorShape(v, oc);
        const double minDistanceNoiseHeadway = getMinDistanceNoiseHeadway(v,oc);
        const double minSpeedNoiseHeadway = getMinSpeedNoiseHeadway(v,oc);
        const double distanceNoiseHeadwayCoeff = getDistanceNoiseHeadwayCoeff(v,oc);
        const double speedNoiseHeadwayCoeff = getSpeedNoiseHeadwayCoeff(v,oc);
        const double optimalSpeedRange = getOptimalSpeedRange(v, oc);
        const double persistentDeltaVError = getPersistentDeltaVError(v, oc);
        const double maxDeltaVError = getMaxDeltaVError(v, oc);
        const double deltaVErrorShape = getDeltaVErrorShape(v, oc);
        const double minDistanceNoiseDeltaV = getMinDistanceNoiseDeltaV(v, oc);
        const double minSpeedNoiseDeltaV = getMinSpeedNoiseDeltaV(v, oc);
        const double distanceNoiseDeltaVCoeff = getDistanceNoiseDeltaVCoeff(v, oc);
        const double speedNoiseDeltaVCoeff = getSpeedNoiseDeltaVCoeff(v, oc);
        const double param1 = getParam1(v, oc);
        const double freeSpeedErrorCoefficient = getFreeSpeedErrorCoefficient(v, oc);
        const double maximalReactionTime = getMaximalReactionTime(v, oc);
        // build the device
        MSDevice_PerIvan* device = new MSDevice_PerIvan(v, "perivan" + v.getID(),
            minAwareness,
            initialAwareness,
            errorTimeScaleCoefficient,
            errorNoiseIntensityCoefficient,
            speedDifferenceErrorCoefficient,
            speedDifferenceChangePerceptionThreshold,
            headwayChangePerceptionThreshold,
            headwayErrorCoefficient,
            persistentHeadwayError,
            optimalPerceptionRange,
            maximalPerceptionRange,
            maxHeadwayError,
            headwayErrorShape,
            minDistanceNoiseHeadway,
            minSpeedNoiseHeadway,
            distanceNoiseHeadwayCoeff,
            speedNoiseHeadwayCoeff,
            optimalSpeedRange,
            persistentDeltaVError,
            maxDeltaVError,
            deltaVErrorShape,
            minDistanceNoiseDeltaV,
            minSpeedNoiseDeltaV,
            distanceNoiseDeltaVCoeff,
            speedNoiseDeltaVCoeff,
            param1,
            freeSpeedErrorCoefficient,
            maximalReactionTime);
        into.push_back(device);
    }
}


double
MSDevice_PerIvan::getMinAwareness(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.minAwareness", PerIvanDefaults::minAwareness, false);
}
double
MSDevice_PerIvan::getInitialAwareness(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.initialAwareness", PerIvanDefaults::initialAwareness, false);
}
double
MSDevice_PerIvan::getErrorTimeScaleCoefficient(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.errorTimeScaleCoefficient", PerIvanDefaults::errorTimeScaleCoefficient, false);
}
double
MSDevice_PerIvan::getErrorNoiseIntensityCoefficient(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.errorNoiseIntensityCoefficient", PerIvanDefaults::errorNoiseIntensityCoefficient, false);
}
double
MSDevice_PerIvan::getSpeedDifferenceErrorCoefficient(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.speedDifferenceErrorCoefficient", PerIvanDefaults::speedDifferenceErrorCoefficient, false);
}
double
MSDevice_PerIvan::getSpeedDifferenceChangePerceptionThreshold(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.speedDifferenceChangePerceptionThreshold", PerIvanDefaults::speedDifferenceChangePerceptionThreshold, false);
}
double
MSDevice_PerIvan::getHeadwayChangePerceptionThreshold(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.headwayChangePerceptionThreshold", PerIvanDefaults::headwayChangePerceptionThreshold, false);
}
double
MSDevice_PerIvan::getHeadwayErrorCoefficient(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.headwayErrorCoefficient", PerIvanDefaults::headwayErrorCoefficient, false);
}
double
MSDevice_PerIvan::getPersistentHeadwayError(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.persistentHeadwayError", PerIvanDefaults::persistentHeadwayError, false);
}
double
MSDevice_PerIvan::getOptimalPerceptionRange(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.optimalPerceptionRange", PerIvanDefaults::optimalPerceptionRange, false);
}
double
MSDevice_PerIvan::getMaximalPerceptionRange(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.maximalPerceptionRange", PerIvanDefaults::maximalPerceptionRange, false);
}
double
MSDevice_PerIvan::getMaxHeadwayError(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.maxHeadwayError", PerIvanDefaults::maxHeadwayError, false);
}
double
MSDevice_PerIvan::getHeadwayErrorShape(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.headwayErrorShape", PerIvanDefaults::headwayErrorShape, false);
}

double
MSDevice_PerIvan::getMinDistanceNoiseHeadway(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.minDistanceNoiseHeadway", PerIvanDefaults::minDistanceNoiseHeadway, false);
}
double
MSDevice_PerIvan::getMinSpeedNoiseHeadway(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.minSpeedNoiseHeadway", PerIvanDefaults::minSpeedNoiseHeadway, false);
}
double
MSDevice_PerIvan::getDistanceNoiseHeadwayCoeff(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.distanceNoiseHeadwayCoeff", PerIvanDefaults::distanceNoiseHeadwayCoeff, false);
}
double
MSDevice_PerIvan::getSpeedNoiseHeadwayCoeff(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.speedNoiseHeadwayCoeff", PerIvanDefaults::speedNoiseHeadwayCoeff, false);
}
double
MSDevice_PerIvan::getOptimalSpeedRange(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.optimalSpeedRange", PerIvanDefaults::optimalSpeedRange, false);
}
double
MSDevice_PerIvan::getPersistentDeltaVError(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.persistentDeltaVError", PerIvanDefaults::persistentDeltaVError, false);
}
double
MSDevice_PerIvan::getMaxDeltaVError(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.maxDeltaVError", PerIvanDefaults::maxDeltaVError, false);
}
double
MSDevice_PerIvan::getDeltaVErrorShape(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.deltaVErrorShape", PerIvanDefaults::deltaVErrorShape, false);
}
double
MSDevice_PerIvan::getMinDistanceNoiseDeltaV(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.minDistanceNoiseDeltaV", PerIvanDefaults::minDistanceNoiseDeltaV, false);
}
double
MSDevice_PerIvan::getMinSpeedNoiseDeltaV(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.minSpeedNoiseDeltaV", PerIvanDefaults::minSpeedNoiseDeltaV, false);
}
double
MSDevice_PerIvan::getDistanceNoiseDeltaVCoeff(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.distanceNoiseDeltaVCoeff", PerIvanDefaults::distanceNoiseDeltaVCoeff, false);
}
double
MSDevice_PerIvan::getSpeedNoiseDeltaVCoeff(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.speedNoiseDeltaVCoeff", PerIvanDefaults::speedNoiseDeltaVCoeff, false);
}
double
MSDevice_PerIvan::getParam1(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.param1", PerIvanDefaults::param1, false);
}
double
MSDevice_PerIvan::getFreeSpeedErrorCoefficient(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.freeSpeedErrorCoefficient", PerIvanDefaults::freeSpeedErrorCoefficient, false);
}
double
MSDevice_PerIvan::getMaximalReactionTime(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "perivan.maximalReactionTime", -1.0, false);
}


// ---------------------------------------------------------------------------
// MSDevice_PerIvan-methods
// ---------------------------------------------------------------------------
MSDevice_PerIvan::MSDevice_PerIvan(SUMOVehicle& holder, const std::string& id,
    double minAwareness,
    double initialAwareness,
    double errorTimeScaleCoefficient,
    double errorNoiseIntensityCoefficient,
    double speedDifferenceErrorCoefficient,
    double speedDifferenceChangePerceptionThreshold,
    double headwayChangePerceptionThreshold,
    double headwayErrorCoefficient,
    double persistentHeadwayError,
    double optimalPerceptionRange,
    double maximalPerceptionRange,
    double maxHeadwayError,
    double headwayErrorShape,
    double minDistanceNoiseHeadway,
    double minSpeedNoiseHeadway,
    double distanceNoiseHeadwayCoeff,
    double speedNoiseHeadwayCoeff,
    double optimalSpeedRange,
    double persistentDeltaVError,
    double maxDeltaVError,
    double deltaVErrorShape,
    double minDistanceNoiseDeltaV,
    double minSpeedNoiseDeltaV,
    double distanceNoiseDeltaVCoeff,
    double speedNoiseDeltaVCoeff,
    double param1,
    double freeSpeedErrorCoefficient,
    double maximalReactionTime) :
    MSVehicleDevice(holder, id),
    myMinAwareness(minAwareness),
    myInitialAwareness(initialAwareness),
    myErrorTimeScaleCoefficient(errorTimeScaleCoefficient),
    myErrorNoiseIntensityCoefficient(errorNoiseIntensityCoefficient),
    mySpeedDifferenceErrorCoefficient(speedDifferenceErrorCoefficient),
    mySpeedDifferenceChangePerceptionThreshold(speedDifferenceChangePerceptionThreshold),
    myHeadwayChangePerceptionThreshold(headwayChangePerceptionThreshold),
    myHeadwayErrorCoefficient(headwayErrorCoefficient),
    myPersistentHeadwayError(persistentHeadwayError),
    myOptimalPerceptionRange(optimalPerceptionRange),
    myMaximalPerceptionRange(maximalPerceptionRange),
    myMaxHeadwayError(maxHeadwayError),
    myHeadwayErrorShape(headwayErrorShape),
    myMinDistanceNoiseHeadway(minDistanceNoiseHeadway),
    myMinSpeedNoiseHeadway(minSpeedNoiseHeadway),
    myDistanceNoiseHeadwayCoeff(distanceNoiseHeadwayCoeff),
    mySpeedNoiseHeadwayCoeff(speedNoiseHeadwayCoeff),
    myOptimalSpeedRange(optimalSpeedRange),
    myPersistentDeltaVError(persistentDeltaVError),
    myMaxDeltaVError(maxDeltaVError),
    myDeltaVErrorShape(deltaVErrorShape),
    myMinDistanceNoiseDeltaV(minDistanceNoiseDeltaV),
    myMinSpeedNoiseDeltaV(minSpeedNoiseDeltaV),
    myDistanceNoiseDeltaVCoeff(distanceNoiseDeltaVCoeff),
    mySpeedNoiseDeltaVCoeff(speedNoiseDeltaVCoeff),
    myParam1(param1),
    myFreeSpeedErrorCoefficient(freeSpeedErrorCoefficient),
    myMaximalReactionTime(maximalReactionTime) {
    // Take care! Holder is currently being constructed. Cast occurs before completion.
    myHolderMS = static_cast<MSVehicle*>(&holder);
    initPerIvan();


#ifdef DEBUG_DSDEVICE
    std::cout << "initialized device '" << id << "' with "
        << "myMinAwareness=" << myMinAwareness << ", "
        << "myInitialAwareness=" << myInitialAwareness << ", "
        << "myErrorTimeScaleCoefficient=" << myErrorTimeScaleCoefficient << ", "
        << "myErrorNoiseIntensityCoefficient=" << myErrorNoiseIntensityCoefficient << ", "
        << "mySpeedDifferenceErrorCoefficient=" << mySpeedDifferenceErrorCoefficient << ", "
        << "mySpeedDifferenceChangePerceptionThreshold=" << mySpeedDifferenceChangePerceptionThreshold << ", "
        << "myHeadwayChangePerceptionThreshold=" << myHeadwayChangePerceptionThreshold << ", "
        << "myHeadwayErrorCoefficient=" << myHeadwayErrorCoefficient << std::endl;
    << "myFreeSpeedErrorCoefficient=" << myFreeSpeedErrorCoefficient << std::endl;
#endif

}

void
MSDevice_PerIvan::initPerIvan() {
    myPerIvan = std::make_shared<MSSimplePerIvan>(myHolderMS);
    myPerIvan->setMinAwareness(myMinAwareness);
    myPerIvan->setInitialAwareness(myInitialAwareness);
    myPerIvan->setErrorTimeScaleCoefficient(myErrorTimeScaleCoefficient);
    myPerIvan->setErrorNoiseIntensityCoefficient(myErrorNoiseIntensityCoefficient);
    myPerIvan->setSpeedDifferenceErrorCoefficient(mySpeedDifferenceErrorCoefficient);
    myPerIvan->setHeadwayErrorCoefficient(myHeadwayErrorCoefficient);
    myPerIvan->setPersistentHeadwayError(myPersistentHeadwayError);
    myPerIvan->setOptimalPerceptionRange(myOptimalPerceptionRange);
    myPerIvan->setMaximalPerceptionRange(myMaximalPerceptionRange);   
    myPerIvan->setMaxHeadwayError(myMaxHeadwayError);
    myPerIvan->setHeadwayErrorShape(myHeadwayErrorShape);
    myPerIvan->setMinDistanceNoiseHeadway(myMinDistanceNoiseHeadway);
    myPerIvan->setMinSpeedNoiseHeadway(myMinSpeedNoiseHeadway);
    myPerIvan->setDistanceNoiseHeadwayCoeff(myDistanceNoiseHeadwayCoeff);
    myPerIvan->setSpeedNoiseHeadwayCoeff(mySpeedNoiseHeadwayCoeff);
    myPerIvan->setOptimalSpeedRange(myOptimalSpeedRange);
    myPerIvan->setPersistentDeltaVError(myPersistentDeltaVError);
    myPerIvan->setMaxDeltaVError(myMaxDeltaVError);
    myPerIvan->setDeltaVErrorShape(myDeltaVErrorShape);
    myPerIvan->setMinDistanceNoiseDeltaV(myMinDistanceNoiseDeltaV);
    myPerIvan->setMinSpeedNoiseDeltaV(myMinSpeedNoiseDeltaV);
    myPerIvan->setDistanceNoiseDeltaVCoeff(myDistanceNoiseDeltaVCoeff);
    myPerIvan->setSpeedNoiseDeltaVCoeff(mySpeedNoiseDeltaVCoeff);
    myPerIvan->setParam1(myParam1);
    myPerIvan->setFreeSpeedErrorCoefficient(myFreeSpeedErrorCoefficient);
    myPerIvan->setSpeedDifferenceChangePerceptionThreshold(mySpeedDifferenceChangePerceptionThreshold);
    myPerIvan->setHeadwayChangePerceptionThreshold(myHeadwayChangePerceptionThreshold);
    myPerIvan->setAwareness(myInitialAwareness);
    if (myMaximalReactionTime > 0) {
        myPerIvan->setMaximalReactionTime(myMaximalReactionTime);
    }
}

void
MSDevice_PerIvan::update() {
    myPerIvan->update();
}

std::string
MSDevice_PerIvan::getParameter(const std::string& key) const {
#ifdef DEBUG_DSDEVICE
    std::cout << "MSDevice_PerIvan::getParameter(key=" << key << ")" << std::endl;
#endif
    if (key == "awareness") {
        return toString(myPerIvan->getAwareness());
    }
    else if (key == "errorState") {
        return toString(myPerIvan->getErrorState());
    }
    else if (key == "errorTimeScale") {
        return toString(myPerIvan->getErrorTimeScale());
    }
    else if (key == "errorNoiseIntensity") {
        return toString(myPerIvan->getErrorNoiseIntensity());
    }
    else if (key == "minAwareness") {
        return toString(myPerIvan->getMinAwareness());
    }
    else if (key == "initialAwareness") {
        return toString(myPerIvan->getInitialAwareness());
    }
    else if (key == "errorTimeScaleCoefficient") {
        return toString(myPerIvan->getErrorTimeScaleCoefficient());
    }
    else if (key == "errorNoiseIntensityCoefficient") {
        return toString(myPerIvan->getErrorNoiseIntensityCoefficient());
    }
    else if (key == "speedDifferenceErrorCoefficient") {
        return toString(myPerIvan->getSpeedDifferenceErrorCoefficient());
    }
    else if (key == "headwayErrorCoefficient") {
        return toString(myPerIvan->getHeadwayErrorCoefficient());
    }
    else if (key == "persistentHeadwayError") {
        return toString(myPerIvan->getPersistentHeadwayError());
    }
    else if (key == "optimalPerceptionRange") {
        return toString(myPerIvan->getOptimalPerceptionRange());
    }
    else if (key == "maximalPerceptionRange") {
        return toString(myPerIvan->getMaximalPerceptionRange());
    } 
    else if (key == "maxHeadwayError") {
        return toString(myPerIvan->getMaxHeadwayError());
    }
    else if (key == "headwayErrorShape") {
        return toString(myPerIvan->getHeadwayErrorShape());
    } 
    else if (key == "minDistanceNoiseHeadway") {
        return toString(myPerIvan->getMinDistanceNoiseHeadway());
    }
    else if (key == "minSpeedNoiseHeadway") {
        return toString(myPerIvan->getMinSpeedNoiseHeadway());
    }
    else if (key == "distanceNoiseHeadwayCoeff") {
        return toString(myPerIvan->getDistanceNoiseHeadwayCoeff());
    }
    else if (key == "speedNoiseHeadwayCoeff") {
        return toString(myPerIvan->getSpeedNoiseHeadwayCoeff());
    }
    else if (key == "optimalSpeedRange") {
        return toString(myPerIvan->getOptimalSpeedRange());
    }
    else if (key == "persistentDeltaVError") {
        return toString(myPerIvan->getPersistentDeltaVError());
    }
    else if (key == "maxDeltaVError") {
        return toString(myPerIvan->getMaxDeltaVError());
    }
    else if (key == "deltaVErrorShape") {
        return toString(myPerIvan->getDeltaVErrorShape());
    }
    else if (key == "minDistanceNoiseDeltaV") {
        return toString(myPerIvan->getMinDistanceNoiseDeltaV());
    }
    else if (key == "minSpeedNoiseDeltaV") {
        return toString(myPerIvan->getMinSpeedNoiseDeltaV());
    }
    else if (key == "distanceNoiseDeltaVCoeff") {
        return toString(myPerIvan->getDistanceNoiseDeltaVCoeff());
    }
    else if (key == "speedNoiseDeltaVCoeff") {
        return toString(myPerIvan->getSpeedNoiseDeltaVCoeff());
    }
    else if (key == "param1") {
        return toString(myPerIvan->getParam1());
    }
    else if (key == "speedDifferenceChangePerceptionThreshold") {
        return toString(myPerIvan->getSpeedDifferenceChangePerceptionThreshold());
    }
    else if (key == "headwayChangePerceptionThreshold") {
        return toString(myPerIvan->getHeadwayChangePerceptionThreshold());
    }
    else if (key == "maximalReactionTime") {
        return toString(myPerIvan->getMaximalReactionTime());
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
#ifdef DEBUG_DSDEVICE
    std::cout << "MSDevice_PerIvan::setParameter(key=" << key << ", value=" << value << ")" << std::endl;
#endif
    if (key == "awareness") {
        myPerIvan->setAwareness(StringUtils::toDouble(value));
    }
    else if (key == "errorState") {
        myPerIvan->setErrorState(StringUtils::toDouble(value));
    }
    else if (key == "errorTimeScale") {
        myPerIvan->setErrorTimeScale(StringUtils::toDouble(value));
    }
    else if (key == "errorNoiseIntensity") {
        myPerIvan->setErrorNoiseIntensity(StringUtils::toDouble(value));
    }
    else if (key == "minAwareness") {
        myPerIvan->setMinAwareness(StringUtils::toDouble(value));
    }
    else if (key == "initialAwareness") {
        myPerIvan->setInitialAwareness(StringUtils::toDouble(value));
    }
    else if (key == "errorTimeScaleCoefficient") {
        myPerIvan->setErrorTimeScaleCoefficient(StringUtils::toDouble(value));
    }
    else if (key == "errorNoiseIntensityCoefficient") {
        myPerIvan->setErrorNoiseIntensityCoefficient(StringUtils::toDouble(value));
    }
    else if (key == "speedDifferenceErrorCoefficient") {
        myPerIvan->setSpeedDifferenceErrorCoefficient(StringUtils::toDouble(value));
    }
    else if (key == "headwayErrorCoefficient") {
        myPerIvan->setHeadwayErrorCoefficient(StringUtils::toDouble(value));
    }
    else if (key == "persistentHeadwayError") {
        myPerIvan->setPersistentHeadwayError(StringUtils::toDouble(value));
    } 
    else if (key == "optimalPerceptionRange") {
        myPerIvan->setOptimalPerceptionRange(StringUtils::toDouble(value));
    }
    else if (key == "maximalPerceptionRange") {
        myPerIvan->setMaximalPerceptionRange(StringUtils::toDouble(value));
    }
    else if (key == "maxHeadwayError") {
        myPerIvan->setMaxHeadwayError(StringUtils::toDouble(value));
    }
    else if (key == "headwayErrorShape") {
        myPerIvan->setHeadwayErrorShape(StringUtils::toDouble(value));
    }
    else if (key == "minDistanceNoiseHeadway") {
        myPerIvan->setMinDistanceNoiseHeadway(StringUtils::toDouble(value));
    }
    else if (key == "minSpeedNoiseHeadway") {
        myPerIvan->setMinSpeedNoiseHeadway(StringUtils::toDouble(value));
    }
    else if (key == "distanceNoiseHeadwayCoeff") {
        myPerIvan->setDistanceNoiseHeadwayCoeff(StringUtils::toDouble(value));
    }
    else if (key == "speedNoiseHeadwayCoeff") {
        myPerIvan->setSpeedNoiseHeadwayCoeff(StringUtils::toDouble(value));
    }
    else if (key == "optimalSpeedRange") {
        myPerIvan->setOptimalSpeedRange(StringUtils::toDouble(value));
    }
    else if (key == "persistentDeltaVError") {
        myPerIvan->setPersistentDeltaVError(StringUtils::toDouble(value));
    }
    else if (key == "maxDeltaVError") {
        myPerIvan->setMaxDeltaVError(StringUtils::toDouble(value));
    }
    else if (key == "deltaVErrorShape") {
        myPerIvan->setDeltaVErrorShape(StringUtils::toDouble(value));
    }
    else if (key == "minDistanceNoiseDeltaV") {
        myPerIvan->setMinDistanceNoiseDeltaV(StringUtils::toDouble(value));
    }
    else if (key == "minSpeedNoiseDeltaV") {
        myPerIvan->setMinSpeedNoiseDeltaV(StringUtils::toDouble(value));
    }
    else if (key == "distanceNoiseDeltaVCoeff") {
        myPerIvan->setDistanceNoiseDeltaVCoeff(StringUtils::toDouble(value));
    }
    else if (key == "speedNoiseDeltaVCoeff") {
        myPerIvan->setSpeedNoiseDeltaVCoeff(StringUtils::toDouble(value));
    }
    else if (key == "param1") {
        myPerIvan->setParam1(StringUtils::toDouble(value));
    }
    else if (key == "freeSpeedErrorCoefficient") {
        myPerIvan->setFreeSpeedErrorCoefficient(StringUtils::toDouble(value));
    }
    else if (key == "speedDifferenceChangePerceptionThreshold") {
        myPerIvan->setSpeedDifferenceChangePerceptionThreshold(StringUtils::toDouble(value));
    }
    else if (key == "headwayChangePerceptionThreshold") {
        myPerIvan->setHeadwayChangePerceptionThreshold(StringUtils::toDouble(value));
    }
    else if (key == "maximalReactionTime") {
        myPerIvan->setMaximalReactionTime(StringUtils::toDouble(value));
    }
    else if (key == "originalReactionTime") {
        myPerIvan->setOriginalReactionTime(StringUtils::toDouble(value));
    }
    else {
        throw InvalidArgument("Parameter '" + key + "' is not supported for device of type '" + deviceName() + "'");
    }
}


/****************************************************************************/
