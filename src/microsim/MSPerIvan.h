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
/// @author  Michael Behrisch
/// @date    Tue, 21 Apr 2015
///
// A class representing a vehicle driver's current mental state
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
/// @class OUProcessIV
/// @brief An Ornstein-Uhlenbeck stochastic process
class OUProcessIV {
public:
    /// @brief constructor
    OUProcessIV(double initialState, double timeScale, double noiseIntensity);
    /// @brief destructor
    ~OUProcessIV();

    /// @brief evolve for a time step of length dt.
    void step(double dt);
    /// @brief static version of the step()
    static double step(double state, double dt, double timeScale, double noiseIntensity);

    /// @brief set the process' timescale to a new value
    void setTimeScale(double timeScale) {
        myTimeScale = timeScale;
    };

    /// @brief set the process' noise intensity to a new value
    void setNoiseIntensity(double noiseIntensity) {
        myNoiseIntensity = noiseIntensity;
    };

    /// @brief set the process' state to a new value
    void setState(double state) {
        myState = state;
    };

    inline double getNoiseIntensity() const {
        return myNoiseIntensity;
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
    /** @brief The current state of the process
     */
    double myState;

    /** @brief The time scale of the process
     */
    double myTimeScale;

    /** @brief The noise intensity of the process
     */
    double myNoiseIntensity;

    /// @brief Random generator for OUProcessIVes
    static SumoRNG myRNG;
};


/// @class MSSimplePerIvan
/// @brief Provides an interface to an error whose fluctuation is controlled
///        via the driver's 'awareness', which can be controlled externally, @see MSDevice_ToC
class MSSimplePerIvan {

public:
    MSSimplePerIvan(MSVehicle* veh);
    virtual ~MSSimplePerIvan() {};


    /// @name Getter methods
    ///@{
    inline double getMinAwareness() const {
        return myMinAwareness;
    }

    inline double getInitialAwareness() const {
        return myInitialAwareness;
    }

    inline double getErrorTimeScaleCoefficient() const {
        return myErrorTimeScaleCoefficient;
    }

    inline double getErrorNoiseIntensityCoefficient() const {
        return myErrorNoiseIntensityCoefficient;
    }

    inline double getErrorTimeScale() const {
        return myError.getTimeScale();
    }

    inline double getErrorNoiseIntensity() const {
        return myError.getNoiseIntensity();
    }

    inline double getSpeedDifferenceErrorCoefficient() const {
        return mySpeedDifferenceErrorCoefficient;
    }

    inline double getHeadwayErrorCoefficient() const {
        return myHeadwayErrorCoefficient;
    }
    inline double getPersistentHeadwayError() const {
        return myPersistentHeadwayError;
    }
    inline double getOptimalPerceptionRange() const {
        return myOptimalPerceptionRange;
    }  
    inline double getMaximalPerceptionRange() const {
        return myMaximalPerceptionRange;
    }   
    inline double getMaxHeadwayError() const {
        return myMaxHeadwayError;
    }
    inline double getHeadwayErrorShape() const {
        return myHeadwayErrorShape;
    }
    inline double getMinDistanceNoiseHeadway() const {
        return myMinDistanceNoiseHeadway;
    }
    inline double getMinSpeedNoiseHeadway() const {
        return myMinSpeedNoiseHeadway;
    }
    inline double getDistanceNoiseHeadwayCoeff() const {
        return myDistanceNoiseHeadwayCoeff;
    }
    inline double getSpeedNoiseHeadwayCoeff() const {
        return mySpeedNoiseHeadwayCoeff;
    }
    inline double getOptimalSpeedRange() const {
        return myOptimalSpeedRange;
    }
    inline double getPersistentDeltaVError() const {
        return myPersistentDeltaVError;
    }
    inline double getMaxDeltaVError() const {
        return myMaxDeltaVError;
    }
    inline double getDeltaVErrorShape() const {
        return myDeltaVErrorShape;
    }
    inline double getMinDistanceNoiseDeltaV() const {
        return myMinDistanceNoiseDeltaV;
    }
    inline double getMinSpeedNoiseDeltaV() const {
        return myMinSpeedNoiseDeltaV;
    }
    inline double getDistanceNoiseDeltaVCoeff() const {
        return myDistanceNoiseDeltaVCoeff;
    }
    inline double getSpeedNoiseDeltaVCoeff() const {
        return mySpeedNoiseDeltaVCoeff;
    }
    inline double getParam1() const {
        return myParam1;
    }
    inline double getFreeSpeedErrorCoefficient() const {
        return myFreeSpeedErrorCoefficient;
    }

    inline double getSpeedDifferenceChangePerceptionThreshold() const {
        return mySpeedDifferenceChangePerceptionThreshold;
    }

    inline double getHeadwayChangePerceptionThreshold() const {
        return myHeadwayChangePerceptionThreshold;
    }

    inline double getAwareness() const {
        return myAwareness;
    }

    inline double getMaximalReactionTime() const {
        return myMaximalReactionTime;
    }

    inline double getOriginalReactionTime() const {
        return myOriginalReactionTime;
    }

    inline double getActionStepLength() const {
        return myActionStepLength;
    }

    inline double getErrorState() const {
        return myError.getState();
    };
    ///@}


    /// @name Setter methods
    ///@{
    inline void setMinAwareness(const double value) {
        myMinAwareness = value;
    }

    inline void setInitialAwareness(const double value) {
        myInitialAwareness = value;
    }

    inline void setErrorTimeScaleCoefficient(const double value) {
        myErrorTimeScaleCoefficient = value;
    }

    inline void setErrorNoiseIntensityCoefficient(const double value) {
        myErrorNoiseIntensityCoefficient = value;
    }

    inline void setSpeedDifferenceErrorCoefficient(const double value) {
        mySpeedDifferenceErrorCoefficient = value;
    }

    inline void setHeadwayErrorCoefficient(const double value) {
        myHeadwayErrorCoefficient = value;
    }
    inline void setPersistentHeadwayError(const double value) {
        myPersistentHeadwayError = value;
    }
    inline void setOptimalPerceptionRange(const double value) {
        myOptimalPerceptionRange = value;
    }
    inline void setMaximalPerceptionRange(const double value) {
        myMaximalPerceptionRange = value;
    }
    inline void setMaxHeadwayError(const double value) {
        myMaxHeadwayError = value;
    }    
    inline void setHeadwayErrorShape(const double value) {
        myHeadwayErrorShape = value;
    }
    inline void setMinDistanceNoiseHeadway(const double value) {
        myMinDistanceNoiseHeadway = value;
    }
    inline void setMinSpeedNoiseHeadway(const double value) {
        myMinSpeedNoiseHeadway = value;
    }
    inline void setDistanceNoiseHeadwayCoeff(const double value) {
        myDistanceNoiseHeadwayCoeff = value;
    }
    inline void setSpeedNoiseHeadwayCoeff(const double value) {
        mySpeedNoiseHeadwayCoeff = value;
    }
    inline void setOptimalSpeedRange(const double value) {
        myOptimalSpeedRange = value;
    }
    inline void setPersistentDeltaVError(const double value) {
        myPersistentDeltaVError = value;
    }
    inline void setMaxDeltaVError(const double value) {
        myMaxDeltaVError = value;
    }
    inline void setDeltaVErrorShape(const double value) {
        myDeltaVErrorShape = value;
    }
    inline void setMinDistanceNoiseDeltaV(const double value) {
        myMinDistanceNoiseDeltaV = value;
    }
    inline void setMinSpeedNoiseDeltaV(const double value) {
        myMinSpeedNoiseDeltaV = value;
    }
    inline void setDistanceNoiseDeltaVCoeff(const double value) {
        myDistanceNoiseDeltaVCoeff = value;
    }
    inline void setSpeedNoiseDeltaVCoeff(const double value) {
        mySpeedNoiseDeltaVCoeff = value;
    }
    inline void setParam1(const double value) {
        myParam1 = value;
    }
    inline void setFreeSpeedErrorCoefficient(const double value) {
        myFreeSpeedErrorCoefficient = value;
    }
    inline void setSpeedDifferenceChangePerceptionThreshold(const double value) {
        mySpeedDifferenceChangePerceptionThreshold = value;
    }
    inline void setHeadwayChangePerceptionThreshold(const double value) {
        myHeadwayChangePerceptionThreshold = value;
    }
    inline void setMaximalReactionTime(const double value) {
        myMaximalReactionTime = value;
        updateReactionTime();
    }
    inline void setOriginalReactionTime(const double value) {
        myOriginalReactionTime = value;
        updateReactionTime();
    }

    void setAwareness(const double value);

    inline void setErrorState(const double state) {
        myError.setState(state);
    };

    inline void setErrorTimeScale(const double value) {
        myError.setTimeScale(value);
    }

    inline void setErrorNoiseIntensity(const double value) {
        myError.setNoiseIntensity(value);
    }
    ///@}

    /// @brief Trigger updates for the errorProcess, assumed gaps, etc
    void update();


    /// @brief Update the assumed gaps to the known objects according to
    ///        the corresponding perceived speed differences.
    void updateAssumedGaps();

    /// @name Methods to obtain the current error quantities to be used by the car-following model
    /// @see TCIModel
    /// @{
//    /// @see myAccelerationError
//    inline double getAppliedAcceleration(double desiredAccel) {
//        return desiredAccel + myError.getState();
//    };

    /// @brief apply perception error to own speed
    double getPerceivedOwnSpeed(double speed);

    /// @brief This method checks whether the errorneous speed difference that would be perceived for this step
    ///        differs sufficiently from the previously perceived to be actually perceived. If so, it sets the
    ///        flag myReactionFlag[objID]=true, which should be checked just after the call to this method because
    ///        it will be overwritten by subsequent calls.
    double getPerceivedSpeedDifference(const double trueSpeedDifference, const double trueGap, const double speed, const void* objID = nullptr);
    /// @see myHeadwayPerceptionError
    double getPerceivedHeadway(const double trueGap, const double speed, const void* objID = nullptr);
    /// @}

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
    // Update the error process
    void updateError();
    // Update the reaction time (actionStepLength)
    void updateReactionTime();

private:

    /// @brief Vehicle corresponding to this per ivan
    MSVehicle* myVehicle;

    /// @brief Driver's 'awareness' \in [0,1]
    double myAwareness;
    /// @brief Minimal value for 'awareness' \in [0,1]
    double myMinAwareness;
    /// @brief Initial value for 'awareness' \in [0,1]
    double myInitialAwareness;
    /// @brief Driver's 'error', @see TCI_Model
    OUProcessIV myError;
    /// @brief Coefficient controlling the impact of awareness on the time scale of the error process
    double myErrorTimeScaleCoefficient;
    /// @brief Coefficient controlling the impact of awareness on the noise intensity of the error process
    double myErrorNoiseIntensityCoefficient;

    /// @brief Scaling coefficients for the magnitude of errors
    double mySpeedDifferenceErrorCoefficient;
    double myHeadwayErrorCoefficient;
    double myFreeSpeedErrorCoefficient;

    /// @brief Persistent headway error
    double myPersistentHeadwayError;
    /// @brief Perception range without increase in errors
    double myOptimalPerceptionRange;
    /// @brief Max. perception range
    double myMaximalPerceptionRange;
    /// @brief Headway error at max. range
    double myMaxHeadwayError;
    /// @brief Headway error function shape    
    double myHeadwayErrorShape;
    /// @brief Min. distance noise on headway   
    double myMinDistanceNoiseHeadway;
    /// @brief Min. speed noise on headway
    double myMinSpeedNoiseHeadway;
    /// @brief Distance noise rate headway coefficient
    double myDistanceNoiseHeadwayCoeff;
    /// @brief Speed noise rate headway coefficient
    double mySpeedNoiseHeadwayCoeff;
    /// @brief Speed range without increase in errors
    double myOptimalSpeedRange;
    /// @brief Persistent deltaV error
    double myPersistentDeltaVError;
    /// @brief DeltaV error at max. range
    double myMaxDeltaVError;
    /// @brief Delta V error function shape
    double myDeltaVErrorShape;
    /// @brief Min. distance noise on deltaV
    double myMinDistanceNoiseDeltaV;
    /// @brief Min. speed noise on deltaV
    double myMinSpeedNoiseDeltaV;
    /// @brief Distance noise rate deltaV coefficient
    double myDistanceNoiseDeltaVCoeff;
    /// @brief Speed noise rate deltaV coefficient
    double mySpeedNoiseDeltaVCoeff;
    /// @brief Calibration parameter scale precision
    double myParam1;

    /// @brief Thresholds above a change in the corresponding quantity is perceived.
    /// @note  In the comparison, we multiply the actual change amount by the current
    ///       gap to the object to reflect a more precise perception if the object is closer.
    double myHeadwayChangePerceptionThreshold;
    double mySpeedDifferenceChangePerceptionThreshold;
    //    // @brief if a perception threshold is passed for some object, a flag is set to induce a reaction to the object
    //    std::map<void*, bool> myReactionFlag;

        /// @brief Action step length (~current maximal reaction time) induced by awareness level
        /// @note  This interpolates linearly from myOriginalReactionTime for awareness==1
        ///        to myMaximalReactionTime for awareness==myMinAwareness
    double myActionStepLength;
    /// @brief Maximal reaction time (value set for the actionStepLength at awareness=1)
    double myOriginalReactionTime;
    /// @brief Maximal reaction time (value set for the actionStepLength at awareness=myMinAwareness)
    double myMaximalReactionTime;

    /// @name Variables for tracking update instants
    /// @see updateStepDuration()
    /// @{
    /// @brief Elapsed time since the last state update
    double myStepDuration;
    /// @brief Time point of the last state update
    double myLastUpdateTime;

    /// @brief The assumed gaps to different objects
    /// @todo: update each step to incorporate the assumed change given a specific speed difference
    std::map<const void*, double> myAssumedGap;
    /// @brief The last perceived speed differences to the corresponding objects
    std::map<const void*, double> myLastPerceivedSpeedDifference;
    /// @}

    /// @brief Used to prevent infinite loops in debugging outputs, @see followSpeed() and stopSpeed() (of MSCFModel_Krauss, e.g.)
    bool myDebugLock;
};


/// @brief Default values for the MSPerIvan parameters
struct PerIvanDefaults {
    static double minAwareness;
    static double initialAwareness;
    static double errorTimeScaleCoefficient;
    static double errorNoiseIntensityCoefficient;
    static double speedDifferenceErrorCoefficient;
    static double speedDifferenceChangePerceptionThreshold;
    static double headwayChangePerceptionThreshold;
    static double headwayErrorCoefficient;
    static double persistentHeadwayError;
    static double optimalPerceptionRange;
    static double maximalPerceptionRange;
    static double maxHeadwayError;
    static double headwayErrorShape;
    static double minDistanceNoiseHeadway;
    static double minSpeedNoiseHeadway;
    static double distanceNoiseHeadwayCoeff;
    static double speedNoiseHeadwayCoeff;
    static double optimalSpeedRange;
    static double persistentDeltaVError;
    static double maxDeltaVError;
    static double deltaVErrorShape;
    static double minDistanceNoiseDeltaV;
    static double minSpeedNoiseDeltaV;
    static double distanceNoiseDeltaVCoeff;
    static double speedNoiseDeltaVCoeff;
    static double param1;
    static double freeSpeedErrorCoefficient;
    static double maximalReactionTimeFactor;
};
