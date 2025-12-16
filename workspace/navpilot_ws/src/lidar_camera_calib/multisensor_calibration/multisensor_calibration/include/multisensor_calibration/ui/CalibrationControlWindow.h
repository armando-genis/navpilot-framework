// Copyright (c) 2024 - 2025 Fraunhofer IOSB and contributors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Fraunhofer IOSB nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef MULTISENSORCALIBRATION_UI_CALIBRATIONCONTROLWINDOW_H
#define MULTISENSORCALIBRATION_UI_CALIBRATIONCONTROLWINDOW_H

// Qt
#include <QMainWindow>
#include <QPushButton>
#include <Qt>

// ROS
#include <rcl_interfaces/msg/log.hpp>

// Multisensor calibration
#include "multisensor_calibration/ui/AboutDialog.h"

namespace multisensor_calibration
{

namespace Ui
{
class CalibrationControlWindow;
}

/**
 * @ingroup ui
 * @brief Main control window of all calibration GUIs.
 *
 * This holds the push buttons to add or remove an observation, to finalize and visualize the
 * calibration. It also hold useful meta information to the calibration, as well as the text box
 * in which the ROS log message appear.
 *
 */
class CalibrationControlWindow : public QMainWindow
{
    Q_OBJECT

    //--- METHOD DECLARATION ---//

  public:
    /**
     * @brief Constructor
     */
    CalibrationControlWindow(QWidget* parent = nullptr);

    /**
     * @brief Destructor
     *
     */
    ~CalibrationControlWindow();

    /**
     * @brief Method to attach placement guidance dialog to the window and connect the corresponding
     * display buttons to the dialog visibility state.
     *
     * @param[in] pDialog Pointer to the dialog object that is to be attached.
     * @param[in] buttonInitialCheckState Initial check state of push button.
     */
    void attachPlacementGuidanceDialog(QDialog* pDialog,
                                       Qt::CheckState buttonInitialCheckState = Qt::Checked);

    /**
     * @brief Method to attach dialog for the reference data to the window and
     * connect the corresponding display buttons to the dialog visibility state.
     *
     * @param[in] pDialog Pointer to the dialog object that is to be attached.
     * @param[in] buttonInitialCheckState Initial check state of push button.
     */
    void attachReferenceDialog(QDialog* pDialog,
                               Qt::CheckState buttonInitialCheckState = Qt::Checked);

    /**
     * @brief Method to attach dialog for the source data to the window and connect the
     * corresponding display buttons to the dialog visibility state.
     *
     * @param[in] pDialog Pointer to the dialog object that is to be attached.
     * @param[in] buttonInitialCheckState Initial check state of push button.
     */
    void attachSourceDialog(QDialog* pDialog,
                            Qt::CheckState buttonInitialCheckState = Qt::Checked);

    /**
     * @brief Method to clear log messages
     *
     */
    void clearLogMessages();

    /** @overload
     */
    void closeEvent(QCloseEvent* closeEvent) override;

    /**
     */
    void showEvent(QShowEvent* showEvent) override;

    /**
     * @brief Get pointer to action to reset calibration.
     */
    QAction* actionResetCalibrationPtr();

    /**
     * @brief Get pointer to action to open preferences.
     */
    QAction* actionOpenPreferencesPtr();

    /**
     * @brief Get pointer to action to open robot workspace.
     */
    QAction* actionOpenCalibWsPtr();

    /**
     * @brief Get pointer to action to open robot workspace.
     */
    QAction* actionOpenRobotWsPtr();

    /**
     * @brief Get pointer to action to add observation.
     */
    QAction* actionAddObservationPtr();

    /**
     * @brief Get pointer to import to add observation.
     */
    QAction* actionImportObservationPtr();

    /**
     * @brief Get pointer to push button to trigger capturing of target.
     */
    QPushButton* pbCaptureTargetPtr();

    /**
     * @brief Get pointer to push button to finalize calibration.
     */
    QPushButton* pbFinalizeCalibrationPtr();

    /**
     * @brief Get pointer to push button to remove observation.
     */
    QPushButton* pbRemoveObservationPtr();

    /**
     * @brief Get pointer to push button to handle calibration visualization.
     */
    QPushButton* pbVisCalibrationPtr();

    /**
     * @brief Callback method to print log message in log text edit.
     *
     * @param[in] pLogMsg Pointer to log message.
     */
    void printLogMessage(const rcl_interfaces::msg::Log::ConstSharedPtr pLogMsg);

  signals:
    /**
     * @brief Signal emitted when control window is closed.
     */
    void closed();

    void newLogMessage(QString newMsg);

  private:
    /**
     * @brief Private function to connect a view dialog to a push button.
     *
     * @param[in, out] pDialog Pointer to the dialog object that is to be attached.
     * @param[in, out] pButton Pointer to the Button object.
     * @param[in] buttonInitialCheckState Initial check state of push button.
     */
    void attachDialogToPushButton(QDialog*& pDialog,
                                  QPushButton*& pButton,
                                  Qt::CheckState buttonInitialCheckState = Qt::Checked) const;

    /**
     * @brief Handle triggering of documentation action
     */
    void onActionDocumentationTriggered() const;

    /**
     * @brief Handle triggering of about action
     */
    void onActionAboutTriggered() const;

    //--- MEMBER DECLARATION ---//

  private:
    Ui::CalibrationControlWindow* ui;

    // Object of the about dialog.
    AboutDialog* pAboutDialog;
};

} // namespace multisensor_calibration
#endif // MULTISENSORCALIBRATION_UI_CALIBRATIONCONTROLWINDOW_H
