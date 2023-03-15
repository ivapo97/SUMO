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
    // ToC device implies perivan
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
