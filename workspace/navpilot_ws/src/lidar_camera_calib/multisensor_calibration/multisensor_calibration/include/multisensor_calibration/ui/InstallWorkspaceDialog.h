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

#ifndef MULTISENSORCALIBRATION_UI_INSTALLWORKSPACEDIALOG_H
#define MULTISENSORCALIBRATION_UI_INSTALLWORKSPACEDIALOG_H

// Qt
#include <QDialog>
#include <QDir>
#include <Qt>

namespace multisensor_calibration
{

namespace Ui
{
class InstallWorkspaceDialog;
}

/**
 * @ingroup ui
 * @brief GUI dialog to install workspace templates.
 * This is part of the calibration configurator of the multi_sensor_calibration node.
 *
 */
class InstallWorkspaceDialog : public QDialog
{
    Q_OBJECT

    //--- METHOD DECLARATION ---//

  public:
    /**
     * @brief Construct a new Install Workspace Dialog object.
     *
     * @param[in] parent Parent Widget
     */
    InstallWorkspaceDialog(QWidget* parent = nullptr);

    /**
     * @brief Destroy the Calibration Config Dialog object.
     */
    ~InstallWorkspaceDialog();

    /**
     * @brief Set path to calibration root dir in which to install the templates.
     */
    void setCalibrationRootDirPath(const QString& iDirPath);

  public slots:

    void accept() override;

  private slots:

    /**
     * @brief Handle click to deselect all calibration workspaces
     */
    void handleDeselectAllPushButtonClicked();

    /**
     * @brief Handle change of Robot Workspace folder
     */
    void handleRobotWsChanged();

    /**
     * @brief Handle click to select all calibration workspaces
     */
    void handleSelectAllPushButtonClicked();

    //--- MEMBER DECLARATION ---//

  private:
    /// Pointer to UI
    Ui::InstallWorkspaceDialog* ui;

    /// Root directory of all calibrations, holding individual robot workspaces.
    QDir calibrationRootDir_;
};

} // namespace multisensor_calibration
#endif // MULTISENSORCALIBRATION_UI_INSTALLWORKSPACEDIALOG_H
