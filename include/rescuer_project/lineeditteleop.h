#ifndef LINEEDITTELEOP_H
#define LINEEDITTELEOP_H

#include <QLineEdit>

class LineEditTeleop : public QLineEdit
{
    Q_OBJECT
public:
    explicit LineEditTeleop(QWidget *parent = 0);
protected:
    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *event);
signals:
    void pageUpPressed();
    void pageDownPressed();
    void upPressed();
    void downPressed();
    void leftPressed();
    void rightPressed();
    void spacePressed();
    void homePressed();
    void endPressed();
    void keyReleased();
public slots:
private:

};

#endif // LINEEDITTELEOP_H
