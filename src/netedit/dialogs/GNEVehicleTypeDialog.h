/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2019 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    GNEVehicleTypeDialog.h
/// @author  Pablo Alvarez Lopez
/// @date    Jan 2019
/// @version $Id$
///
// Dialog for edit vehicleTypes
/****************************************************************************/
#ifndef GNEVehicleTypeDialog_h
#define GNEVehicleTypeDialog_h

// ===========================================================================
// included modules
// ===========================================================================

#include <config.h>

#include "GNEDemandElementDialog.h"

// ===========================================================================
// class declarations
// ===========================================================================

class GNEVehicleType;

// ===========================================================================
// class definitions
// ===========================================================================

/**
 * @class GNEVehicleTypeDialog
 * @brief Dialog for edit rerouter intervals
 */

class GNEVehicleTypeDialog : public GNEDemandElementDialog {

public:
    /// @brief class for VClasses
    class VTypeCommonAtributes : protected FXGroupBox {
        /// @brief FOX-declaration
        FXDECLARE(GNEVehicleTypeDialog::VTypeCommonAtributes)

    public:
        /// @brief class for VClasses
        class VClassRow : protected FXHorizontalFrame {

        public:
            /// @brief constructor
            VClassRow(VTypeCommonAtributes* VTypeCommonAtributesParent, FXVerticalFrame* column);

            /// @brief set variables
            void setVariable();

            /// @brief update values
            void updateValues();

        protected:
            /// @brief set VClass texture
            void setVClassLabelImage();

            /// @brief FXComboBox for VClass
            FXComboBox* myComboBoxVClass;
        
            /// @brief label with image of VClass
            FXLabel* myComboBoxVClassLabelImage;

        private:
            /// @brief pointer to VTypeCommonAtributes parent
            VTypeCommonAtributes* myVTypeCommonAtributesParent;
        };

        /// @brief class for VShapeRow
        class VShapeRow : protected FXHorizontalFrame {
        
        public:
            /// @brief constructor
            VShapeRow(VTypeCommonAtributes* VTypeCommonAtributesParent, FXVerticalFrame* column);

            /// @brief set variables
            void setVariable();

            /// @brief update values
            void updateValues();

        protected:
            /// @brief set VShape texture
            void setVShapeLabelImage();

            /// @brief FXComboBox for Shape
            FXComboBox* myComboBoxShape;

            /// @brief label with image of Shape
            FXLabel* myComboBoxShapeLabelImage;

        private:
            /// @brief pointer to VTypeCommonAtributes parent
            VTypeCommonAtributes* myVTypeCommonAtributesParent;
        };

        /// @brief constructor
        VTypeCommonAtributes(GNEVehicleTypeDialog* vehicleTypeDialog, FXHorizontalFrame* column);

        /// @brief build commmon attributes (A)
        void buildCommonAttributesA(FXVerticalFrame* column);

        /// @brief build commmon attributes (B)
        void buildCommonAttributesB(FXVerticalFrame* column);

        /// @brief update values
        void updateValues();

        /// @name FOX-callbacks
        /// @{
        /// @event called after change a Vehicle Type parameter
        long onCmdSetVariable(FXObject*, FXSelector, void*);

        /// @event called after change a Vehicle Type color
        long onCmdSetColor(FXObject*, FXSelector, void*);
        /// @}

    protected:
        /// @brief fox need this
        VTypeCommonAtributes() {}

        /// @brief build row int
        FXTextField* buildRowInt(FXPacker* column, SumoXMLAttr tag);

        /// @brief build row float
        FXTextField* buildRowFloat(FXPacker* column, SumoXMLAttr tag);

        /// @brief build row
        FXTextField* buildRowString(FXPacker* column, SumoXMLAttr tag);

        /// @brief FXTextfield for vehicleTypeID
        FXTextField* myTextFieldVehicleTypeID;

        /// @brief vehicle class row
        VClassRow* myVClassRow;

        /// @brief FXButton for Color
        FXButton* myButtonColor;

        /// @brief FXTextfield for Color
        FXTextField* myTextFieldColor;

        /// @brief FXTextfield for Length
        FXTextField* myTextFieldLength;

        /// @brief FXTextfield for MinGap
        FXTextField* myTextFieldMinGap;

        /// @brief FXTextfield for MaxSpeed
        FXTextField* myTextFieldMaxSpeed;

        /// @brief FXTextfield for SpeedFactor
        FXTextField* myTextFieldSpeedFactor;

        /// @brief FXTextfield for SpeedDev
        FXTextField* myTextFieldSpeedDev;

        /// @brief FXTextfield for EmissionClass
        FXTextField* myTextFieldEmissionClass;

        /// @brief vehicle shape row
        VShapeRow* myVShapeRow;

        /// @brief FXTextfield for Width
        FXTextField* myTextFieldWidth;

        /// @brief FXTextfield for Filename
        FXTextField* myTextFieldFilename;

        /// @brief FXTextfield for Impatience
        FXTextField* myTextFieldImpatience;

        /// @brief ComboBox for LaneChangeModel
        FXComboBox* myComboBoxLaneChangeModel;

        /// @brief FXTextfield for PersonCapacity
        FXTextField* myTextFieldPersonCapacity;

        /// @brief FXTextfield for ContainerCapacity
        FXTextField* myTextFieldContainerCapacity;

        /// @brief FXTextfield for BoardingDuration
        FXTextField* myTextFieldBoardingDuration;

        /// @brief FXTextfield for LoadingDuration
        FXTextField* myTextFieldLoadingDuration;

        /// @brief FXComboBox for LatAlignment
        FXComboBox* myComboBoxLatAlignment;

        /// @brief FXTextfield for MinGapLat
        FXTextField* myTextFieldMinGapLat;

        /// @brief FXTextfield for MaxSpeedLat
        FXTextField* myTextFieldMaxSpeedLat;

        /// @brief FXTextfield for ActionStepLenght
        FXTextField* myTextFieldActionStepLenght;

    private:
        /// @brief pointer to Vehicle Type dialog parent
        GNEVehicleTypeDialog* myVehicleTypeDialog;
    };

    /// @brief class for CarFollowingModel
    class CarFollowingModelParameters : public FXGroupBox {
        /// @brief FOX-declaration
        FXDECLARE(GNEVehicleTypeDialog::CarFollowingModelParameters)

    public:
        /// @brief constructor
        CarFollowingModelParameters(GNEVehicleTypeDialog* vehicleTypeDialog, FXHorizontalFrame* column);

        /// @brief refresh Car Following Model Fields
        void refreshCFMFields();

        /// @brief update values
        void updateValues();

        /// @name FOX-callbacks
        /// @{
        /// @event called after change a CFM variable
        long onCmdSetVariable(FXObject*, FXSelector, void*);
        /// @}
        
    protected:
        /// @brief fox need this
        CarFollowingModelParameters() {}

        /// @brief class used for represent rows with Car Following Model parameters
        class CarFollowingModelRow : public FXHorizontalFrame {
        public:
            /// @brief constructor
            CarFollowingModelRow(CarFollowingModelParameters *carFollowingModelParametersParent, FXVerticalFrame* verticalFrame, SumoXMLAttr attr);
            
            /// @brief set Variablen in VehicleType
            void setVariable();

            /// @brief update value of Vehicle Type
            void updateValue();

        private:
            /// @brief pointer to CarFollowingModelParameters parent
            CarFollowingModelParameters *myCarFollowingModelParametersParent;

            /// @brief edited attribute
            SumoXMLAttr myAttr;

            /// @brief text field
            FXTextField* textField;

            /// @label Label with the Row attribute
            FXLabel *myLabel;
        };
        
    private:
        /// @brief pointer to Vehicle Type dialog parent
        GNEVehicleTypeDialog* myVehicleTypeDialog;
                
        /// @brief vector with the Car Following Model Row
        std::vector<CarFollowingModelRow*> myRows;

        /// @brief Row for CarFollowModel
        FXComboBox* myComboBoxCarFollowModel;

        /// @brief Vertical Frame for CarFollowingModelRow
        FXVerticalFrame *myVerticalFrameRows;

        /// @brief Row for Accel
        CarFollowingModelRow* myAccelRow;

        /// @brief Row for Decel
        CarFollowingModelRow* myDecelRow;

        /// @brief Row for aparent Decel
        CarFollowingModelRow* myApparentDecelRow;
    
        /// @brief Row for emergency Decel
        CarFollowingModelRow* myEmergencyDecelRow;

        /// @brief Row for Sigma
        CarFollowingModelRow* mySigmaRow;

        /// @brief Row for Tau
        CarFollowingModelRow* myTauRow;

        /// @brief Row for MinGapFactor
        CarFollowingModelRow* myMinGapFactorRow;

        /// @brief Row for MinGap (only for Kerner)
        CarFollowingModelRow* myKRow;

        /// @brief Row for MinGap (only for Kerner)
        CarFollowingModelRow* myPhiRow;

        /// @brief Row for MinGap (only for IDM)
        CarFollowingModelRow* myDeltaRow;

        /// @brief Row for MinGap(only for IDM)
        CarFollowingModelRow* mySteppingRow;

        /// @brief Row for Security (only for Wiedemann)
        CarFollowingModelRow* mySecurityRow;

        /// @brief Row for Estimation (only for Wiedemann)
        CarFollowingModelRow* myEstimationRow;

        /// @brief Row for TMP1
        CarFollowingModelRow* myTmp1Row;

        /// @brief Row for TMP2
        CarFollowingModelRow* myTmp2Row;

        /// @brief Row for TMP3
        CarFollowingModelRow* myTmp3Row;

        /// @brief Row for TMP4
        CarFollowingModelRow* myTmp4Row;

        /// @brief Row for TMP5
        CarFollowingModelRow* myTmp5Row;

        /// @brief Row for TrainType
        CarFollowingModelRow* myTrainTypeRow;

        /// @brief Row for TauLast
        CarFollowingModelRow* myTrauLastRow;

        /// @brief Row for Aprob
        CarFollowingModelRow* myAprobRow;

        /// @brief Row for Adapt Factor
        CarFollowingModelRow* myAdaptFactorRow;

        /// @brief Row for Adapt Time
        CarFollowingModelRow* myAdaptTimeRow;

        /// @brief temporal label for incomplete attributes
        FXLabel *myLabelIncompleteAttribute;
    };

    /// @brief constructor
    GNEVehicleTypeDialog(GNEDemandElement* editedVehicleType, bool updatingElement);

    /// @brief destructor
    ~GNEVehicleTypeDialog();

    /// @name FOX-callbacks
    /// @{
    /// @brief event after press accept button
    long onCmdAccept(FXObject*, FXSelector, void*);

    /// @brief event after press cancel button
    long onCmdCancel(FXObject*, FXSelector, void*);

    /// @brief event after press reset button
    long onCmdReset(FXObject*, FXSelector, void*);

    /// @event after change a variable of vehicle type
    long onCmdSetVariable(FXObject*, FXSelector, void*);
    /// @}

protected:
    /// @brief update data fields
    void updateVehicleTypeValues();

private:
    /// @brief flag to check if current vehicleType is valid
    bool myVehicleTypeValid;

    /// @brief current sumo attribute invalid
    SumoXMLAttr myInvalidAttr;

    /// @brief Vehicle Type Common Attributes
    VTypeCommonAtributes* myVTypeCommonAtributes;

    /// @brief Car Following model parameters
    CarFollowingModelParameters* myCarFollowingModelParameters;

    /// @brief Invalidated copy constructor.
    GNEVehicleTypeDialog(const GNEVehicleTypeDialog&) = delete;

    /// @brief Invalidated assignment operator.
    GNEVehicleTypeDialog& operator=(const GNEVehicleTypeDialog&) = delete;
};

#endif
