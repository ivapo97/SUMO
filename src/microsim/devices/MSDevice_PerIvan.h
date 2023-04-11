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
/// @file    MSDevice_PerIvan.h
/// @author  Leonhard Luecken
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @date    15.06.2018
///
/// The Per Ivan Device mainly provides a configuration and interaction interface for the vehicle's Per Ivan.
/// @see microsim/MSPerIvan.h
///
/****************************************************************************/
#pragma once
#include <config.h>

#include "MSVehicleDevice.h"
#include <utils/common/SUMOTime.h>
#include <utils/common/WrappingCommand.h>


// ===========================================================================
// class declarations
// ===========================================================================
class SUMOVehicle;
class MSVehicle;
class MSSimplePerIvan;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSDevice_PerIvan
 *
 * @brief The ToC Device controls transition of control between automated and manual driving.
 * @todo: Provide logging facilities
 * @todo: allow manual and automated type to refer to vTypeDistributions
 *
 * @see MSDevice
 */
class MSDevice_PerIvan : public MSVehicleDevice {
public:
    /** @brief Inserts MSDevice_PerIvan-options
     * @param[filled] oc The options container to add the options to
     */
    static void insertOptions(OptionsCont& oc);


    /** @brief Build devices for the given vehicle, if needed
     *
     * The options are read and evaluated whether a ToC-device shall be built
     *  for the given vehicle.
     *
     * The built device is stored in the given vector.
     *
     * @param[in] v The vehicle for which a device may be built
     * @param[filled] into The vector to store the built device in
     */
    static void buildVehicleDevices(SUMOVehicle& v, std::vector<MSVehicleDevice*>& into);

    /// update internal state
    void update();

    /// return internal state
    inline std::shared_ptr<MSSimplePerIvan> getPerIvan() const {
        return myPerIvan;
    }

private:
    /// @name Helpers for parameter parsing
    /// @{
    static double getMinAwareness(const SUMOVehicle& v, const OptionsCont& oc);
    static double getInitialAwareness(const SUMOVehicle& v, const OptionsCont& oc);
    static double getErrorTimeScaleCoefficient(const SUMOVehicle& v, const OptionsCont& oc);
    static double getErrorNoiseIntensityCoefficient(const SUMOVehicle& v, const OptionsCont& oc);
    static double getSpeedDifferenceErrorCoefficient(const SUMOVehicle& v, const OptionsCont& oc);
    static double getSpeedDifferenceChangePerceptionThreshold(const SUMOVehicle& v, const OptionsCont& oc);
    static double getHeadwayChangePerceptionThreshold(const SUMOVehicle& v, const OptionsCont& oc);
    static double getHeadwayErrorCoefficient(const SUMOVehicle& v, const OptionsCont& oc);
    static double getPersistentHeadwayError(const SUMOVehicle& v, const OptionsCont& oc);  
    static double getOptimalPerceptionRange(const SUMOVehicle& v, const OptionsCont& oc); 
    static double getMaximalPerceptionRange(const SUMOVehicle& v, const OptionsCont& oc);
    static double getMaxHeadwayError(const SUMOVehicle& v, const OptionsCont& oc);    
    static double getHeadwayErrorShape(const SUMOVehicle& v, const OptionsCont& oc);
    static double getMinDistanceNoiseHeadway(const SUMOVehicle& v, const OptionsCont& oc);
    static double getMinSpeedNoiseHeadway(const SUMOVehicle& v, const OptionsCont& oc);
    static double getDistanceNoiseHeadwayCoeff(const SUMOVehicle& v, const OptionsCont& oc);
    static double getSpeedNoiseHeadwayCoeff(const SUMOVehicle& v, const OptionsCont& oc);
    static double getOptimalSpeedRange(const SUMOVehicle& v, const OptionsCont& oc);
    static double getPersistentDeltaVError(const SUMOVehicle& v, const OptionsCont& oc);
    static double getMaxDeltaVError(const SUMOVehicle& v, const OptionsCont& oc);
    static double getDeltaVErrorShape(const SUMOVehicle& v, const OptionsCont& oc);
    static double getMinDistanceNoiseDeltaV(const SUMOVehicle& v, const OptionsCont& oc);
    static double getMinSpeedNoiseDeltaV(const SUMOVehicle& v, const OptionsCont& oc);
    static double getDistanceNoiseDeltaVCoeff(const SUMOVehicle& v, const OptionsCont& oc);
    static double getSpeedNoiseDeltaVCoeff(const SUMOVehicle& v, const OptionsCont& oc);
    static double getParam1(const SUMOVehicle& v, const OptionsCont& oc);
    static double getFreeSpeedErrorCoefficient(const SUMOVehicle& v, const OptionsCont& oc);
    static double getMaximalReactionTime(const SUMOVehicle& v, const OptionsCont& oc);
    /// @}


public:
    /// @brief Destructor.
    ~MSDevice_PerIvan() {};

    /// @brief return the name for this type of device
    const std::string deviceName() const {
        return "perivan";
    }

    /// @brief try to retrieve the given parameter from this device. Throw exception for unsupported key
    std::string getParameter(const std::string& key) const;

    /// @brief try to set the given parameter for this device. Throw exception for unsupported key
    void setParameter(const std::string& key, const std::string& value);


private:
    /** @brief Constructor
     *
     * @param[in] holder The vehicle that holds this device
     * @param[in] id The ID of the device
     */
    MSDevice_PerIvan(SUMOVehicle& holder, const std::string& id,
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
        double maximalReactionTime);

    /// @brief Initializeses the per ivan parameters
    void initPerIvan();

private:
    /// @brief The holder vehicle casted to MSVehicle*
    MSVehicle* myHolderMS;

    /// @name Temporary to hold perivan parameters until initialization.
    /// @note Invalid after call to initPerIvan().
    /// @{
    double myMinAwareness;
    double myInitialAwareness;
    double myErrorTimeScaleCoefficient;
    double myErrorNoiseIntensityCoefficient;
    double mySpeedDifferenceErrorCoefficient;
    double mySpeedDifferenceChangePerceptionThreshold;
    double myHeadwayChangePerceptionThreshold;
    double myHeadwayErrorCoefficient;
    double myPersistentHeadwayError;
    double myOptimalPerceptionRange;
    double myMaximalPerceptionRange;
    double myMaxHeadwayError;
    double myHeadwayErrorShape;
    double myMinDistanceNoiseHeadway;
    double myMinSpeedNoiseHeadway;
    double myDistanceNoiseHeadwayCoeff;
    double mySpeedNoiseHeadwayCoeff;
    double myOptimalSpeedRange;
    double myPersistentDeltaVError;
    double myMaxDeltaVError;
    double myDeltaVErrorShape;
    double myMinDistanceNoiseDeltaV;
    double myMinSpeedNoiseDeltaV;
    double myDistanceNoiseDeltaVCoeff;
    double mySpeedNoiseDeltaVCoeff;
    double myParam1;
    double myFreeSpeedErrorCoefficient;
    double myMaximalReactionTime;
    /// @}

    /// @brief The per ivan of the holder.
    std::shared_ptr<MSSimplePerIvan> myPerIvan;

private:
    /// @brief Invalidated copy constructor.
    MSDevice_PerIvan(const MSDevice_PerIvan&);

    /// @brief Invalidated assignment operator.
    MSDevice_PerIvan& operator=(const MSDevice_PerIvan&);

};
