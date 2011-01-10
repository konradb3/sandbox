#include "propertydialog.h"
#include "ui_propertydialog.h"

PropertyDialog::PropertyDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PropertyDialog)
{
    ui->setupUi(this);

    connect(ui->acceptButton, SIGNAL(clicked()), this, SLOT(commit()));
}

PropertyDialog::~PropertyDialog()
{
    delete ui;
}

QString PropertyDialog::getControllerName()
{
  return controller;
}

QString PropertyDialog::getKinematicName()
{
  return kinematic;
}

QString PropertyDialog::getJointTopicName()
{
  return jointTopic;
}

void PropertyDialog::commit()
{
  controller = ui->controllerEdit->text();
  kinematic = ui->kinematicEdit->text();
  jointTopic = ui->jointEdit->text();
}
