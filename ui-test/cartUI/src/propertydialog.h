#ifndef PROPERTYDIALOG_H
#define PROPERTYDIALOG_H

#include <QDialog>
#include <QString>

namespace Ui {
    class PropertyDialog;
}

class PropertyDialog : public QDialog
{
    Q_OBJECT

public:
    explicit PropertyDialog(QWidget *parent = 0);
    ~PropertyDialog();

    QString getControllerName();
    QString getKinematicName();
    QString getJointTopicName();
public slots:
    void commit();
private:
    Ui::PropertyDialog *ui;
    QString controller;
    QString kinematic;
    QString jointTopic;
};

#endif // PROPERTYDIALOG_H
