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
/// @file    MSPerIvan.h
/// @author  Ivan Postigo
/// @date    Tue, 21 Apr 2015
///
// A class representing a vehicle driver's current perception performance
/****************************************************************************/

/// @todo: check parameter admissibility in setter methods

#pragma once
#include <config.h>
#include <memory>
#include <utils/common/SUMOTime.h>
#include <utils/xml/SUMOXMLDefinitions.h>

// ===========================================================================
// class definitions
// ===========================================================================
/// @class WienerProcess
/// @brief An Ornstein-Uhlenbeck stochastic process
class WienerProcess {
public:
    /// @brief constructor
    WienerProcess(double initialState, double timeScale);
    /// @brief destructor
    ~WienerProcess();
    /// @brief evolve for a time step of length dt.
    void step(double dt);
    /// @brief static version of the step()
    static double step(double state, double dt, double timeScale);
    /// @brief set the process' timescale to a new value
    void setTimeScale(double timeScale) {
        myTimeScale = timeScale;
    };
    /// @brief set the process' state to a new value
    void setState(double state) {
        myState = state;
    };
    inline double getTimeScale() const {
        return myTimeScale;
    };
    /// @brief Obtain the current state of the process
    double getState() const;

    static SumoRNG* getRNG() {
        return &myRNG;
    }

private:
    /// @brief The current state of the process
    double myState;
    /// @brief The time scale of the process
    double myTimeScale;
    /// @brief Random generator for OUProcessIVes
    static SumoRNG myRNG;
};

/// @class MSSimplePerIvan
/// @brief Provides an interface to a perception error 
class MSSimplePerIvan {
public:
    MSSimplePerIvan(MSVehicle* veh);
    virtual ~MSSimplePerIvan() {};
    /// @name Getter methods
    ///@{
    inline double getTimeCorrelationWindow() const {
        return myTimeCorrelationWindow;
    }
    inline double getPerceptionDelay() const {
        return myPerceptionDelay;
    }
    inline double getErrorTimeScale() const {
        return myWienerProcess.getTimeScale();
    }
    inline double getMinDistanceError() const {
        return myMinDistanceError;
    }
    inline double getOptimalPerceptionDistance() const {
        return myOptimalPerceptionDistance;
    }  
    inline double getMaximalPerceptionDistance() const {
        return myMaximalPerceptionDistance;
    }   
    inline double getMaxDistanceError() const {
        return myMaxDistanceError;
    }
    inline double getDistanceErrorShape() const {
        return myDistanceErrorShape;
    }
    inline double getMinDistancePrecision() const {
        return myMinDistancePrecision;
    }
    inline double getMinSpeedPrecision() const {
        return myMinSpeedPrecision;
    }
    inline double getDistancePrecisionCoeff() const {
        return myDistancePrecisionCoeff;
    }
    inline double getSpeedPrecisionCoeff() const {
        return mySpeedPrecisionCoeff;
    }
    inline double getOptimalPerceptionSpeed() const {
        return myOptimalPerceptionSpeed;
    }
    inline double getParam1() const {
        return myParam1;
    }
    inline double getParam2() const {
        return myParam2;
    }
    inline double getOriginalReactionTime() const {
        return myOriginalReactionTime;
    }
    inline double getActionStepLength() const {
        return myActionStepLength;
    }
    inline double getWienerProcessState() const {
        return myWienerProcess.getState();
    };
    ///@}

    /// @name Setter methods
    ///@{
    inline void setTimeCorrelationWindow(const double value) {
        myTimeCorrelationWindow = value;
    }
    inline void setPerceptionDelay(const double value) {
        myPerceptionDelay = value;
    }
    inline void setMinDistanceError(const double value) {
        myMinDistanceError = value;
    }
    inline void setOptimalPerceptionDistance(const double value) {
        myOptimalPerceptionDistance = value;
    }
    inline void setMaximalPerceptionDistance(const double value) {
        myMaximalPerceptionDistance = value;
    }
    inline void setMaxDistanceError(const double value) {
        myMaxDistanceError = value;
    }    
    inline void setDistanceErrorShape(const double value) {
        myDistanceErrorShape = value;
    }
    inline void setMinDistancePrecision(const double value) {
        myMinDistancePrecision = value;
    }
    inline void setMinSpeedPrecision(const double value) {
        myMinSpeedPrecision = value;
    }
    inline void setDistancePrecisionCoeff(const double value) {
        myDistancePrecisionCoeff = value;
    }
    inline void setSpeedPrecisionCoeff(const double value) {
        mySpeedPrecisionCoeff = value;
    }
    inline void setOptimalPerceptionSpeed(const double value) {
        myOptimalPerceptionSpeed = value;
    }
    inline void setParam1(const double value) {
        myParam1 = value;
    }
    inline void setParam2(const double value) {
        myParam2 = value;
    }
    inline void setOriginalReactionTime(const double value) {
        myOriginalReactionTime = value;
        updateReactionTime();
    }
    inline void setWienerProcessState(const double state) {
        myWienerProcess.setState(state);
    };
    inline void setErrorTimeScale(const double value) {
        myWienerProcess.setTimeScale(value);
    }
    ///@}

    /// @brief Trigger updates for the wiener process and reaction + delay
    void update();

    /// @brief Computes the distance error and returns the perceived distance, also computes the perceived relative speed
    double getPerceivedSpeedDifference(const double trueSpeedDifference, const void* objID);
    /// @brief Returns the perceived relative speed if is not far off from the true speed difference.
    double getPerceivedDistance(const double trueDistance, const double ownSpeed, const void* objID = nullptr);

    inline void lockDebug() {
        myDebugLock = true;
    }
    inline void unlockDebug() {
        myDebugLock = false;
    }
    inline bool debugLocked() const {
        return myDebugLock;
    }

private:
    // @brief Update the current step duration
    void updateStepDuration();
    // Update the Wiener process
    void updateWienerProcess();
    // Update the reaction time (actionStepLength)
    void updateReactionTime();

private:
    /// @brief Vehicle corresponding to this perivan device
    MSVehicle* myVehicle;
    /// @brief Vehicle's WienerProcess
    WienerProcess myWienerProcess;
    /// @brief Timewindow in which the error estimation is correlated. [s]
    double myTimeCorrelationWindow;
    /// @brief Perception delay
    double myPerceptionDelay;
    /// @brief Min. (best) distance accuracy error
    double myMinDistanceError;
    /// @brief Perception distance range without increase in accuracy error
    double myOptimalPerceptionDistance;
    /// @brief Max. perception distance range
    double myMaximalPerceptionDistance;
    /// @brief Accuracy error at max. perception distance range.
    double myMaxDistanceError;
    /// @brief Decay of accuracy (linear, quadratic..) 
    double myDistanceErrorShape;
    /// @brief Min. (best) precision due to distance
    double myMinDistancePrecision;
    /// @brief Min. (best) precision due to speed
    double myMinSpeedPrecision;
    /// @brief Precision decay rate due to distance
    double myDistancePrecisionCoeff;
    /// @brief Precision decay rate due to speed
    double mySpeedPrecisionCoeff;
    /// @brief Speed range without decrease in precision
    double myOptimalPerceptionSpeed;
    /// @brief Auxiliary parameter 1
    double myParam1;
    /// @brief Auxiliary parameter 2
    double myParam2;
    /// @brief Action step length (reaction time+perception delay)
    double myActionStepLength;
    /// @brief Maximal reaction time (value set for the actionStepLength at awareness=1)
    double myOriginalReactionTime;

    /// @name Variables for tracking update instants
    /// @see updateStepDuration()
    /// @{
    /// @brief Elapsed time since the last state update in seconds
    double myStepDuration;
    /// @brief Time point of the last state update
    double myLastUpdateTime;

    /// @brief The perceived distances to different objects
    /// @todo: update each step if possible (?)
    std::map<const void*, double> myPerceivedDistances;
    /// @brief The last perceived speed differences to the corresponding objects
    std::map<const void*, double> myPerceivedSpeedDifference;
    /// @}

    /// @brief Used to prevent infinite loops in debugging outputs, @see followSpeed() and stopSpeed() (of MSCFModel_Krauss, e.g.)
    bool myDebugLock;
};

/// @brief Default values for the MSPerIvan parameters
struct PerIvanDefaults {
    static double timeCorrelationWindow;
    static double perceptionDelay;
    static double minDistanceError;
    static double optimalPerceptionDistance;
    static double maximalPerceptionDistance;
    static double maxDistanceError;
    static double distanceErrorShape;
    static double minDistancePrecision;
    static double minSpeedPrecision;
    static double distancePrecisionCoeff;
    static double speedPrecisionCoeff;
    static double optimalPerceptionSpeed;
    static double param1;
    static double param2;
};
