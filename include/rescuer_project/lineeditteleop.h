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
signals:
    void pageUpPressed();
    void pageDownPressed();
    void upPressed();
    void downPressed();
    void leftPressed();
    void rightPressed();
public slots:

};

#endif // LINEEDITTELEOP_H
