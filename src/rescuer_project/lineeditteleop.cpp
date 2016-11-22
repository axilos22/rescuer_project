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
    case Qt::Key_Space:
        emit spacePressed();
        break;
    case Qt::Key_Home:
        emit homePressed();
        break;
    case Qt::Key_End:
        emit endPressed();
        break;
    case Qt::Key_W:
        emit wkeyPressed();
        break;
    case Qt::Key_X:
        emit xkeyPressed();
        break;
    case Qt::Key_A:
        emit akeyPressed();
        break;
    case Qt::Key_D:
        emit dkeyPressed();
        break;





    default:
        break;
    }
}

void LineEditTeleop::keyReleaseEvent(QKeyEvent *event)
{
    Q_UNUSED(event);
    emit keyReleased();
}
