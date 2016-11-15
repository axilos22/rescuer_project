#include "rescuer_project/lineeditteleop.h"
#include <QKeyEvent>
LineEditTeleop::LineEditTeleop(QWidget *parent) :
    QLineEdit(parent)
{}

void LineEditTeleop::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
    case Qt::Key_PageUp:
        emit pageUpPressed();
        break;
    case Qt::Key_PageDown:
        emit pageDownPressed();
        break;
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
