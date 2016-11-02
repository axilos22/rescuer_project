#ifndef RESCUER_WIDGET_H
#define RESCUER_WIDGET_H

#include <QWidget>
#include <QVBoxLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QBoxLayout>
#include <QString>
#include <QPushButton>

class RescuerWidget : public QWidget
{
//    Q_OBJECT
public:
    explicit RescuerWidget(QWidget *parent = 0);
    void initWidget(QString name="rescuerWidget0");
protected:
    QVBoxLayout* _vLayout;
    QWidget* _cameraImg;
    QBoxLayout* _statusBox;
    QBoxLayout* _orderBox;
signals:

public slots:

};

#endif // RESCUER_WIDGET_H
