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
/// @file    GNEToolDialog.cpp
/// @author  Pablo Alvarez Lopez
/// @date    Jun 2022
///
// Abstract dialog for tools
/****************************************************************************/

#include <netedit/GNEApplicationWindow.h>
#include <utils/gui/div/GUIDesigns.h>
#include <utils/handlers/TemplateHandler.h>

#include "GNEToolDialog.h"


#define EXTRAMARGING 4

// ===========================================================================
// FOX callback mapping
// ===========================================================================

FXDEFMAP(GNEToolDialog) GNEToolDialogMap[] = {
    FXMAPFUNC(SEL_CLOSE,    0,                      GNEToolDialog::onCmdCancel),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_BUTTON_RUN,     GNEToolDialog::onCmdRun),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_BUTTON_CANCEL,  GNEToolDialog::onCmdCancel),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_BUTTON_RESET,   GNEToolDialog::onCmdReset)
};

// Object implementation
FXIMPLEMENT(GNEToolDialog, FXTopWindow, GNEToolDialogMap, ARRAYNUMBER(GNEToolDialogMap))

// ============================================-===============================
// member method definitions
// ===========================================================================

GNEToolDialog::GNEToolDialog(GNEApplicationWindow* GNEApp, const std::string& name, const std::string& templateToolStr) :
    FXTopWindow(GNEApp->getApp(), name.c_str(), GUIIconSubSys::getIcon(GUIIcon::EMPTY), 
        GUIIconSubSys::getIcon(GUIIcon::EMPTY), GUIDesignDialogBoxTools),
    myGNEApp(GNEApp) {
    // parse tool options
    TemplateHandler::parseTemplate(myToolsOptions, templateToolStr);
    // create main frame
    FXVerticalFrame* mainFrame = new FXVerticalFrame(this, GUIDesignAuxiliarFrame);
    // build horizontalFrame for content
    myContentFrame = new FXVerticalFrame(mainFrame, GUIDesignAuxiliarFrame);
    // build arguments
    buildArguments();
    // add separator
    new FXSeparator(mainFrame);
    // create buttons centered
    FXHorizontalFrame* buttonsFrame = new FXHorizontalFrame(mainFrame, GUIDesignHorizontalFrame);
    new FXHorizontalFrame(buttonsFrame, GUIDesignAuxiliarHorizontalFrame);
    new FXButton(buttonsFrame, (TL("Run") + std::string("\t\t") + TL("close accepting changes")).c_str(),  GUIIconSubSys::getIcon(GUIIcon::ACCEPT), this, MID_GNE_BUTTON_RUN, GUIDesignButtonAccept);
    new FXButton(buttonsFrame, (TL("Cancel") + std::string("\t\t") + TL("close discarding changes")).c_str(), GUIIconSubSys::getIcon(GUIIcon::CANCEL), this, MID_GNE_BUTTON_CANCEL, GUIDesignButtonCancel);
    new FXButton(buttonsFrame, (TL("Reset") + std::string("\t\t") + TL("reset to previous values")).c_str(),  GUIIconSubSys::getIcon(GUIIcon::RESET),  this, MID_GNE_BUTTON_RESET,  GUIDesignButtonReset);
    new FXHorizontalFrame(buttonsFrame, GUIDesignAuxiliarHorizontalFrame);
}


GNEToolDialog::~GNEToolDialog() {}


void
GNEToolDialog::show() {
    // show dialog
    FXTopWindow::show(PLACEMENT_SCREEN);
    // refresh APP
    getApp()->refresh();
    // open as modal dialog (will block all windows until stop() or stopModal() is called)
    myGNEApp->getApp()->runModalFor(this);
}


void
GNEToolDialog::hide() {
    // stop modal
    myGNEApp->getApp()->stopModal(this);
    // hide dialog
    FXTopWindow::hide();
}


FXVerticalFrame*
GNEToolDialog::getContentFrame() const {
    return myContentFrame;
}


long
GNEToolDialog::onCmdRun(FXObject*, FXSelector, void*) {
    // RUN

    // hide tool dialog
    hide();
    return 1;
}


long
GNEToolDialog::onCmdCancel(FXObject*, FXSelector, void*) {
    // just hide tool dialog
    hide();
    return 1;
}


long
GNEToolDialog::onCmdReset(FXObject*, FXSelector, void*) {
    // iterate over all arguments and reset values
    for (const auto& argument : myArguments) {
        argument->reset();
    }
    return 1;
}


void
GNEToolDialog::buildArguments() {
    int maxColumnWidth = 0;
    // iterate over options
    for (const auto &option : myToolsOptions) {
        if (option.second->isInteger()) {
            myArguments.push_back(new GNEToolDialogElements::IntArgument(this, option.first, option.second));
        } else if (option.second->isFloat()) {
            myArguments.push_back(new GNEToolDialogElements::FloatArgument(this, option.first, option.second));
        } else if (option.second->isBool()) {
            //myArguments.push_back(new GNEToolDialogElements::FileNameArgument(this, option.first, option.second));
        } else if (option.second->isFileName()) {
            myArguments.push_back(new GNEToolDialogElements::FileNameArgument(this, option.first, option.second));       
        } else {
            myArguments.push_back(new GNEToolDialogElements::StringArgument(this, option.first, option.second));
        }
        // check column width
        int columnWidth = myArguments.back()->getNameLabel()->getFont()->getTextWidth(option.first.c_str(), option.first.size() + EXTRAMARGING);
        if (columnWidth > maxColumnWidth) {
            maxColumnWidth = columnWidth;
        }
    }
    // set content frame size
    myContentFrame->setHeight(GUIDesignHeight * (int)myArguments.size());
    // adjust parameter label
    for (auto &argument : myArguments) {
        argument->getNameLabel()->setWidth(maxColumnWidth);
    }

    // USE MFXLabelTooltip
}

/****************************************************************************/
