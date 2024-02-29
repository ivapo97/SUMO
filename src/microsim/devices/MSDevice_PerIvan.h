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
#include <utils/common/SUMOTime.h>
#include <utils/common/WrappingCommand.h>
#include "MSVehicleDevice.h"


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
 * @brief The PerIvan Device controls the estimation of the headway
 * @todo: Provide logging facilities
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
     * The options are read and evaluated whether a PerIvan-device shall be built
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
    static double getTimeCorrelationWindow(const SUMOVehicle& v, const OptionsCont& oc);
    static double getPerceptionDelay(const SUMOVehicle& v, const OptionsCont& oc);
    static double getMinDistanceError(const SUMOVehicle& v, const OptionsCont& oc);  
    static double getOptimalPerceptionDistance(const SUMOVehicle& v, const OptionsCont& oc); 
    static double getMaximalPerceptionDistance(const SUMOVehicle& v, const OptionsCont& oc);
    static double getMaxDistanceError(const SUMOVehicle& v, const OptionsCont& oc);    
    static double getDistanceErrorShape(const SUMOVehicle& v, const OptionsCont& oc);
    static double getMinDistancePrecision(const SUMOVehicle& v, const OptionsCont& oc);
    static double getMinSpeedPrecision(const SUMOVehicle& v, const OptionsCont& oc);
    static double getDistancePrecisionCoeff(const SUMOVehicle& v, const OptionsCont& oc);
    static double getSpeedPrecisionCoeff(const SUMOVehicle& v, const OptionsCont& oc);
    static double getOptimalPerceptionSpeed(const SUMOVehicle& v, const OptionsCont& oc);
    static double getParam1(const SUMOVehicle& v, const OptionsCont& oc);
    static double getParam2(const SUMOVehicle& v, const OptionsCont& oc);
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
        double param2);

    /// @brief Initializeses the per ivan parameters
    void initPerIvan();

private:
    /// @brief The holder vehicle casted to MSVehicle*
    MSVehicle* myHolderMS;

    /// @name Temporary to hold perivan parameters until initialization.
    /// @note Invalid after call to initPerIvan().
    /// @{
    double myTimeCorrelationWindow;
    double myPerceptionDelay;
    double myMinDistanceError;
    double myOptimalPerceptionDistance;
    double myMaximalPerceptionDistance;
    double myMaxDistanceError;
    double myDistanceErrorShape;
    double myMinDistancePrecision;
    double myMinSpeedPrecision;
    double myDistancePrecisionCoeff;
    double mySpeedPrecisionCoeff;
    double myOptimalPerceptionSpeed;
    double myParam1;
    double myParam2;
    /// @}

    /// @brief The per ivan of the holder.
    std::shared_ptr<MSSimplePerIvan> myPerIvan;

private:
    /// @brief Invalidated copy constructor.
    MSDevice_PerIvan(const MSDevice_PerIvan&);

    /// @brief Invalidated assignment operator.
    MSDevice_PerIvan& operator=(const MSDevice_PerIvan&);

};
