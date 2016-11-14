#ifndef CENTRALWIDGET_H
#define CENTRALWIDGET_H

#include <QWidget>

class CentralWidget : public QWidget
{
    Q_OBJECT
public:
    explicit CentralWidget(QWidget *parent = 0);
protected:
    void keyPressEvent(QKeyEvent *event);
signals:
    void upPressed();
    void downPressed();
    void leftPressed();
    void rightPressed();
public slots:

};

#endif // CENTRALWIDGET_H
