/********************************************************************************
** Form generated from reading UI file 'launcher.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LAUNCHER_H
#define UI_LAUNCHER_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Launcher
{
public:
    QHBoxLayout *horizontalLayout_4;
    QVBoxLayout *verticalLayout;
    QSpacerItem *verticalSpacer_5;
    QPushButton *kuka_button;
    QSpacerItem *verticalSpacer;
    QVBoxLayout *verticalLayout_2;
    QPushButton *coil_controller_button;
    QSpacerItem *verticalSpacer_2;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout;
    QPushButton *camera_bottom_button;
    QPushButton *camera_front_button;
    QSpacerItem *verticalSpacer_3;
    QVBoxLayout *verticalLayout_5;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *dynamixel_man_button;
    QPushButton *dynamixel_aut_button;
    QPushButton *dynamixel_calib_button;
    QSpacerItem *verticalSpacer_4;
    QVBoxLayout *verticalLayout_6;
    QHBoxLayout *horizontalLayout_3;
    QPushButton *custom_launch_button;
    QPushButton *stop_button;
    QSpacerItem *horizontalSpacer;
    QLabel *label;
    QComboBox *modes_combo_box;
    QSpacerItem *verticalSpacer_6;

    void setupUi(QWidget *Launcher)
    {
        if (Launcher->objectName().isEmpty())
            Launcher->setObjectName(QStringLiteral("Launcher"));
        Launcher->setWindowModality(Qt::NonModal);
        Launcher->resize(436, 335);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(Launcher->sizePolicy().hasHeightForWidth());
        Launcher->setSizePolicy(sizePolicy);
        Launcher->setAcceptDrops(false);
        horizontalLayout_4 = new QHBoxLayout(Launcher);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalSpacer_5 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_5);

        kuka_button = new QPushButton(Launcher);
        kuka_button->setObjectName(QStringLiteral("kuka_button"));

        verticalLayout->addWidget(kuka_button);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        coil_controller_button = new QPushButton(Launcher);
        coil_controller_button->setObjectName(QStringLiteral("coil_controller_button"));

        verticalLayout_2->addWidget(coil_controller_button);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer_2);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        camera_bottom_button = new QPushButton(Launcher);
        camera_bottom_button->setObjectName(QStringLiteral("camera_bottom_button"));

        horizontalLayout->addWidget(camera_bottom_button);

        camera_front_button = new QPushButton(Launcher);
        camera_front_button->setObjectName(QStringLiteral("camera_front_button"));

        horizontalLayout->addWidget(camera_front_button);


        verticalLayout_3->addLayout(horizontalLayout);


        verticalLayout_2->addLayout(verticalLayout_3);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer_3);

        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        dynamixel_man_button = new QPushButton(Launcher);
        dynamixel_man_button->setObjectName(QStringLiteral("dynamixel_man_button"));

        horizontalLayout_2->addWidget(dynamixel_man_button);

        dynamixel_aut_button = new QPushButton(Launcher);
        dynamixel_aut_button->setObjectName(QStringLiteral("dynamixel_aut_button"));

        horizontalLayout_2->addWidget(dynamixel_aut_button);

        dynamixel_calib_button = new QPushButton(Launcher);
        dynamixel_calib_button->setObjectName(QStringLiteral("dynamixel_calib_button"));

        horizontalLayout_2->addWidget(dynamixel_calib_button);


        verticalLayout_5->addLayout(horizontalLayout_2);

        verticalSpacer_4 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_5->addItem(verticalSpacer_4);


        verticalLayout_2->addLayout(verticalLayout_5);


        verticalLayout->addLayout(verticalLayout_2);

        verticalLayout_6 = new QVBoxLayout();
        verticalLayout_6->setObjectName(QStringLiteral("verticalLayout_6"));
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        custom_launch_button = new QPushButton(Launcher);
        custom_launch_button->setObjectName(QStringLiteral("custom_launch_button"));

        horizontalLayout_3->addWidget(custom_launch_button);

        stop_button = new QPushButton(Launcher);
        stop_button->setObjectName(QStringLiteral("stop_button"));

        horizontalLayout_3->addWidget(stop_button);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer);

        label = new QLabel(Launcher);
        label->setObjectName(QStringLiteral("label"));

        horizontalLayout_3->addWidget(label);

        modes_combo_box = new QComboBox(Launcher);
        modes_combo_box->setObjectName(QStringLiteral("modes_combo_box"));

        horizontalLayout_3->addWidget(modes_combo_box);


        verticalLayout_6->addLayout(horizontalLayout_3);

        verticalSpacer_6 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_6->addItem(verticalSpacer_6);


        verticalLayout->addLayout(verticalLayout_6);


        horizontalLayout_4->addLayout(verticalLayout);


        retranslateUi(Launcher);

        QMetaObject::connectSlotsByName(Launcher);
    } // setupUi

    void retranslateUi(QWidget *Launcher)
    {
        Launcher->setWindowTitle(QApplication::translate("Launcher", "Launcher", 0));
        kuka_button->setText(QApplication::translate("Launcher", "Kuka Launch", 0));
        coil_controller_button->setText(QApplication::translate("Launcher", "Coil Controller Launch", 0));
        camera_bottom_button->setText(QApplication::translate("Launcher", "Camera Bottom", 0));
        camera_front_button->setText(QApplication::translate("Launcher", "Camera Front", 0));
        dynamixel_man_button->setText(QApplication::translate("Launcher", "Dynamixel Man", 0));
        dynamixel_aut_button->setText(QApplication::translate("Launcher", "Dynamixel Aut", 0));
        dynamixel_calib_button->setText(QApplication::translate("Launcher", "Dynamixel Calib", 0));
        custom_launch_button->setText(QApplication::translate("Launcher", "Custom Launch", 0));
        stop_button->setText(QApplication::translate("Launcher", "Stop All", 0));
        label->setText(QApplication::translate("Launcher", "Modes:", 0));
    } // retranslateUi

};

namespace Ui {
    class Launcher: public Ui_Launcher {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LAUNCHER_H
