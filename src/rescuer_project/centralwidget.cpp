#include "rescuer_project/centralwidget.h"
#include <QKeyEvent>

CentralWidget::CentralWidget(QWidget *parent) :
    QWidget(parent)
{
}

void CentralWidget::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
    case Qt::Key_Up:
        emit upPressed();
        break;
    case Qt::Key_Down:
        emit downPressed();
        break;
    case Qt::Key_Left:
        emit leftPressed();
        break;
    case Qt::Key_Right:
        emit rightPressed();
        break;
    default:
        break;
    }
}
