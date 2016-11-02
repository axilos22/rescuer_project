#include "rescuer_project/rescuer_widget.h"

RescuerWidget::RescuerWidget(QWidget *parent) :
    QWidget(parent)
{
}

void RescuerWidget::initWidget(QString name)
{
    this->setAccessibleName(name);
    _vLayout = new QVBoxLayout;
    _statusBox = new QBoxLayout(QBoxLayout::LeftToRight);
    _orderBox = new QBoxLayout(QBoxLayout::LeftToRight);
    _cameraImg = new QWidget();

    //test
    QPushButton *img= new QPushButton("rescuer_img");
    QPushButton *status=new QPushButton("status");
    QPushButton *order=new QPushButton("ORDERS");
    img->setParent(_cameraImg);
    _statusBox->addWidget(status);
    _orderBox->addWidget(order);

    _cameraImg->setAutoFillBackground(true);

    _vLayout->addWidget(_cameraImg);
    _vLayout->addLayout(_statusBox);
    _vLayout->addLayout(_orderBox);

    this->setLayout(_vLayout);
}
