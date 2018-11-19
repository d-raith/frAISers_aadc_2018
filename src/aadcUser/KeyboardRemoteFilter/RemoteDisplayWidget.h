
/******************************************************************************
 * Audi Autonomous Driving Cup 2018
 * Team frAIsers
******************************************************************************/

#ifndef KEYBOARDREMOTEFILTER_REMOTEDISPLAYWIDGET_H_
#define KEYBOARDREMOTEFILTER_REMOTEDISPLAYWIDGET_H_

#include "stdafx.h"
#define KEY_MOVE_FWD 87  // W
#define KEY_MOVE_BWD 83  // S
#define KEY_STEER_LEFT 65  // S
#define KEY_STEER_RIGHT 68  // D


class RemoteDisplayWidget : public QWidget {
    Q_OBJECT

 public:
    explicit RemoteDisplayWidget(QWidget* parent);
    ~RemoteDisplayWidget() {}
    QPushButton *m_pEnableKeyboardButton;
    QPushButton *m_pDisableKeyboardButton;

 public slots:

 protected:
    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void focusOutEvent(QFocusEvent *e);

 signals:
    void keyReceived(int value);

    void keyFwdPressed();
    void keyBwdPressed();
    void keyLeftPressed();
    void keyRightPressed();
    void keyFwdReleased();
    void keyBwdReleased();
    void keyLeftReleased();
    void keyRightReleased();

    void widgetOutOfFocus();

 private:
    QWidget* m_pWidget;
    QVBoxLayout *m_mainLayout;
};
#endif
