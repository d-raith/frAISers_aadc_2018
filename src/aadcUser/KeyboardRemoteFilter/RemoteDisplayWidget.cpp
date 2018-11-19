
/******************************************************************************
 * Audi Autonomous Driving Cup 2018
 * Team frAIsers
******************************************************************************/

#include "stdafx.h"
#include "RemoteDisplayWidget.h"

RemoteDisplayWidget::RemoteDisplayWidget(QWidget* parent) : QWidget(parent) {
    m_pWidget = new QWidget(this);
    m_pWidget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    QFont font;
    font.setPointSize(20);

    QLabel *toplabel = new QLabel(this);
    toplabel->setText("Keyboard Remote Controller");
    toplabel->setAlignment(Qt::AlignCenter | Qt::AlignCenter);
    toplabel->setFont(font);

    m_pEnableKeyboardButton = new QPushButton(this);
    m_pEnableKeyboardButton->setText("Enable Keyboard Control");
    m_pEnableKeyboardButton->setFixedSize(200, 50);
    m_pEnableKeyboardButton->setFocusPolicy(Qt::NoFocus);

    m_pDisableKeyboardButton = new QPushButton(this);
    m_pDisableKeyboardButton->setText("Disable Keyboard Control");
    m_pDisableKeyboardButton->setFixedSize(200, 50);
    m_pDisableKeyboardButton->setFocusPolicy(Qt::NoFocus);

    m_mainLayout = new QVBoxLayout();
    m_mainLayout->addWidget(toplabel, 0, Qt::AlignCenter);
    m_mainLayout->addWidget(m_pEnableKeyboardButton, 0, Qt::AlignCenter);
    m_mainLayout->addWidget(m_pDisableKeyboardButton, 0, Qt::AlignCenter);
    setLayout(m_mainLayout);
    setFocusPolicy(Qt::StrongFocus);
}

void RemoteDisplayWidget::keyPressEvent(QKeyEvent* event) {
    if (!event->isAutoRepeat()) {
        emit keyReceived(static_cast<int>(event->key()));
        int key = event->key();
        if (key == KEY_MOVE_FWD) {
            emit keyFwdPressed();
        } else if (key == KEY_MOVE_BWD) {
            emit keyBwdPressed();
        } else if (key == KEY_STEER_LEFT) {
            emit keyLeftPressed();
        } else if (key == KEY_STEER_RIGHT) {
            emit keyRightPressed();
        }
    }
}

void RemoteDisplayWidget::keyReleaseEvent(QKeyEvent* event) {
    if (!event->isAutoRepeat()) {
        int key = event->key();
        if (key == KEY_MOVE_FWD) {
            emit keyFwdReleased();
        } else if (key == KEY_MOVE_BWD) {
            emit keyBwdReleased();
        } else if (key == KEY_STEER_LEFT) {
            emit keyLeftReleased();
        } else if (key == KEY_STEER_RIGHT) {
            emit keyRightReleased();
        }
    }
}

void RemoteDisplayWidget::mousePressEvent(QMouseEvent* event) {
    LOG_INFO("KeyboardRemote: regained focus");
    setFocus();
}

void RemoteDisplayWidget::focusOutEvent(QFocusEvent* e) {
    if (e->reason() == Qt::MouseFocusReason) {
        emit widgetOutOfFocus();
    }
}
