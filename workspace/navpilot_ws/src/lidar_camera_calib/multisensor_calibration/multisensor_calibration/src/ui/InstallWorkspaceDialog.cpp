/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/ui/InstallWorkspaceDialog.h"

// Qt
#include <QDir>
#include <QListWidgetItem>
#include <QPushButton>

// multisensor_calibration
#include "multisensor_calibration/common/common.h"
#include "ui_InstallWorkspaceDialog.h"

namespace multisensor_calibration
{

//==================================================================================================
InstallWorkspaceDialog::InstallWorkspaceDialog(QWidget* parent) :
  QDialog(parent),
  ui(new Ui::InstallWorkspaceDialog),
  calibrationRootDir_(QDir::homePath() + "/Fraunhofer-IOSB/multisensor_calibration")
{
    //--- set up UI
    ui->setupUi(this);
    this->setWindowIcon(QIcon(":/icons/icons8-sensor-100_filled.png"));

    ui->buttonBox->button(QDialogButtonBox::Ok)->setText("Install");

    //--- connect signals and slots
    connect(ui->robotComboBox,
            static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
            this, &InstallWorkspaceDialog::handleRobotWsChanged);
    connect(ui->selectAllPushButton, &QPushButton::clicked,
            this, &InstallWorkspaceDialog::handleSelectAllPushButtonClicked);
    connect(ui->deselectAllPushButton, &QPushButton::clicked,
            this, &InstallWorkspaceDialog::handleDeselectAllPushButtonClicked);

    //--- fill combo box holding different workspaces
    QDir resourceRobotWsDir = QDir(":/robot_workspaces/");
    for (QString wsDir : resourceRobotWsDir.entryList(QDir::AllDirs, QDir::Name))
    {
        ui->robotComboBox->addItem(wsDir);
    }
}

//==================================================================================================
InstallWorkspaceDialog::~InstallWorkspaceDialog()
{
    delete ui;
}

//==================================================================================================
void InstallWorkspaceDialog::setCalibrationRootDirPath(const QString& iDirPath)
{
    calibrationRootDir_.setPath(iDirPath);
}

//==================================================================================================
void InstallWorkspaceDialog::accept()
{
    bool isSuccessful = true;

    QString robotWsFolderName = ui->robotComboBox->currentText();
    QString robotWsPath       = calibrationRootDir_.absolutePath() +
                          QDir::separator() +
                          robotWsFolderName;
    isSuccessful &= calibrationRootDir_.mkpath(robotWsPath);
    isSuccessful &= QFile::copy(":/robot_workspaces/" +
                                  robotWsFolderName + QDir::separator() +
                                  QString::fromStdString(SETTINGS_FILE_NAME),
                                robotWsPath + QDir::separator() +
                                  QString::fromStdString(SETTINGS_FILE_NAME));

    isSuccessful &= QFile(robotWsPath + QDir::separator() +
                          QString::fromStdString(SETTINGS_FILE_NAME))
                      .setPermissions(
                        QFileDevice::ReadOwner | QFileDevice::WriteOwner |
                        QFileDevice::ReadGroup | QFileDevice::WriteGroup |
                        QFileDevice::ReadOther);

    for (int i = 0; i < ui->calibrationListWidget->count(); ++i)
    {
        QListWidgetItem* item = ui->calibrationListWidget->item(i);

        if (item->checkState() == Qt::Checked)
        {
            QString calibWsFolderName = item->text();
            QString calibWsPath       = robotWsPath +
                                  QDir::separator() +
                                  calibWsFolderName;
            isSuccessful &= calibrationRootDir_.mkpath(calibWsPath);
            isSuccessful &= QFile::copy(":/robot_workspaces/" +
                                          robotWsFolderName + QDir::separator() +
                                          calibWsFolderName + QDir::separator() +
                                          QString::fromStdString(SETTINGS_FILE_NAME),
                                        calibWsPath + QDir::separator() +
                                          QString::fromStdString(SETTINGS_FILE_NAME));

            isSuccessful &= QFile(calibWsPath + QDir::separator() +
                                  QString::fromStdString(SETTINGS_FILE_NAME))
                              .setPermissions(
                                QFileDevice::ReadOwner | QFileDevice::WriteOwner |
                                QFileDevice::ReadGroup | QFileDevice::WriteGroup |
                                QFileDevice::ReadOther);
        }
    }

    QDialog::accept();
}

//==================================================================================================
void InstallWorkspaceDialog::handleDeselectAllPushButtonClicked()
{
    for (int i = 0; i < ui->calibrationListWidget->count(); ++i)
    {
        QListWidgetItem* item = ui->calibrationListWidget->item(i);
        item->setCheckState(Qt::Unchecked);
    }
}

//==================================================================================================
void InstallWorkspaceDialog::handleRobotWsChanged()
{
    ui->calibrationListWidget->clear();

    // name of selected robot workspace
    QString robotName = ui->robotComboBox->currentText();

    QDir resourceCalibWsDir = QDir(":/robot_workspaces/" + robotName + "/");
    for (QString wsDir : resourceCalibWsDir.entryList(QDir::AllDirs, QDir::Name))
    {
        QListWidgetItem* item = new QListWidgetItem(ui->calibrationListWidget);
        item->setText(wsDir);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(Qt::Checked);
        ui->calibrationListWidget->addItem(item);
    }
}

//==================================================================================================
void InstallWorkspaceDialog::handleSelectAllPushButtonClicked()
{
    for (int i = 0; i < ui->calibrationListWidget->count(); ++i)
    {
        QListWidgetItem* item = ui->calibrationListWidget->item(i);
        item->setCheckState(Qt::Checked);
    }
}

} // namespace multisensor_calibration
