/********************************************************************************
** Form generated from reading UI file 'log_section.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LOG_SECTION_H
#define UI_LOG_SECTION_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_BagWidget
{
public:
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_2;
    QSpacerItem *horizontalSpacer_4;
    QLabel *label_2;
    QLineEdit *experiment_name;
    QSpacerItem *horizontalSpacer_5;
    QHBoxLayout *horizontalLayout_3;
    QSpacerItem *horizontalSpacer_6;
    QLabel *label;
    QLineEdit *liquid_number;
    QSpacerItem *horizontalSpacer_7;
    QHBoxLayout *horizontalLayout_4;
    QSpacerItem *horizontalSpacer_8;
    QLabel *label_3;
    QLineEdit *distance;
    QSpacerItem *horizontalSpacer_11;
    QHBoxLayout *horizontalLayout_5;
    QSpacerItem *horizontalSpacer_9;
    QLabel *label_4;
    QLineEdit *microrobot_number;
    QSpacerItem *horizontalSpacer_12;
    QHBoxLayout *horizontalLayout_6;
    QSpacerItem *horizontalSpacer_10;
    QLabel *label_5;
    QLineEdit *name;
    QSpacerItem *horizontalSpacer_13;
    QHBoxLayout *horizontalLayout_8;
    QSpacerItem *horizontalSpacer_14;
    QLabel *label_6;
    QLineEdit *location;
    QSpacerItem *horizontalSpacer_15;
    QHBoxLayout *horizontalLayout_7;
    QSpacerItem *horizontalSpacer_2;
    QPushButton *record_button;
    QSpacerItem *horizontalSpacer;
    QPushButton *stop_button;
    QSpacerItem *horizontalSpacer_3;

    void setupUi(QWidget *BagWidget)
    {
        if (BagWidget->objectName().isEmpty())
            BagWidget->setObjectName(QStringLiteral("BagWidget"));
        BagWidget->resize(449, 327);
        horizontalLayout = new QHBoxLayout(BagWidget);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Fixed, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_4);

        label_2 = new QLabel(BagWidget);
        label_2->setObjectName(QStringLiteral("label_2"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(142);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(label_2->sizePolicy().hasHeightForWidth());
        label_2->setSizePolicy(sizePolicy);

        horizontalLayout_2->addWidget(label_2);

        experiment_name = new QLineEdit(BagWidget);
        experiment_name->setObjectName(QStringLiteral("experiment_name"));

        horizontalLayout_2->addWidget(experiment_name);

        horizontalSpacer_5 = new QSpacerItem(40, 20, QSizePolicy::Minimum, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_5);


        verticalLayout->addLayout(horizontalLayout_2);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        horizontalSpacer_6 = new QSpacerItem(40, 20, QSizePolicy::Fixed, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_6);

        label = new QLabel(BagWidget);
        label->setObjectName(QStringLiteral("label"));
        sizePolicy.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
        label->setSizePolicy(sizePolicy);

        horizontalLayout_3->addWidget(label);

        liquid_number = new QLineEdit(BagWidget);
        liquid_number->setObjectName(QStringLiteral("liquid_number"));
        liquid_number->setInputMethodHints(Qt::ImhPreferNumbers);

        horizontalLayout_3->addWidget(liquid_number);

        horizontalSpacer_7 = new QSpacerItem(40, 20, QSizePolicy::Minimum, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_7);


        verticalLayout->addLayout(horizontalLayout_3);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        horizontalSpacer_8 = new QSpacerItem(40, 20, QSizePolicy::Fixed, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer_8);

        label_3 = new QLabel(BagWidget);
        label_3->setObjectName(QStringLiteral("label_3"));
        sizePolicy.setHeightForWidth(label_3->sizePolicy().hasHeightForWidth());
        label_3->setSizePolicy(sizePolicy);

        horizontalLayout_4->addWidget(label_3);

        distance = new QLineEdit(BagWidget);
        distance->setObjectName(QStringLiteral("distance"));
        distance->setInputMethodHints(Qt::ImhPreferNumbers);

        horizontalLayout_4->addWidget(distance);

        horizontalSpacer_11 = new QSpacerItem(40, 20, QSizePolicy::Minimum, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer_11);


        verticalLayout->addLayout(horizontalLayout_4);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        horizontalSpacer_9 = new QSpacerItem(40, 20, QSizePolicy::Fixed, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer_9);

        label_4 = new QLabel(BagWidget);
        label_4->setObjectName(QStringLiteral("label_4"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(142);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(label_4->sizePolicy().hasHeightForWidth());
        label_4->setSizePolicy(sizePolicy1);

        horizontalLayout_5->addWidget(label_4);

        microrobot_number = new QLineEdit(BagWidget);
        microrobot_number->setObjectName(QStringLiteral("microrobot_number"));
        microrobot_number->setInputMethodHints(Qt::ImhPreferNumbers);

        horizontalLayout_5->addWidget(microrobot_number);

        horizontalSpacer_12 = new QSpacerItem(40, 20, QSizePolicy::Minimum, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer_12);


        verticalLayout->addLayout(horizontalLayout_5);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
        horizontalSpacer_10 = new QSpacerItem(40, 20, QSizePolicy::Fixed, QSizePolicy::Minimum);

        horizontalLayout_6->addItem(horizontalSpacer_10);

        label_5 = new QLabel(BagWidget);
        label_5->setObjectName(QStringLiteral("label_5"));
        sizePolicy.setHeightForWidth(label_5->sizePolicy().hasHeightForWidth());
        label_5->setSizePolicy(sizePolicy);

        horizontalLayout_6->addWidget(label_5);

        name = new QLineEdit(BagWidget);
        name->setObjectName(QStringLiteral("name"));

        horizontalLayout_6->addWidget(name);

        horizontalSpacer_13 = new QSpacerItem(40, 20, QSizePolicy::Minimum, QSizePolicy::Minimum);

        horizontalLayout_6->addItem(horizontalSpacer_13);


        verticalLayout->addLayout(horizontalLayout_6);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setObjectName(QStringLiteral("horizontalLayout_8"));
        horizontalSpacer_14 = new QSpacerItem(40, 20, QSizePolicy::Fixed, QSizePolicy::Minimum);

        horizontalLayout_8->addItem(horizontalSpacer_14);

        label_6 = new QLabel(BagWidget);
        label_6->setObjectName(QStringLiteral("label_6"));
        sizePolicy.setHeightForWidth(label_6->sizePolicy().hasHeightForWidth());
        label_6->setSizePolicy(sizePolicy);

        horizontalLayout_8->addWidget(label_6);

        location = new QLineEdit(BagWidget);
        location->setObjectName(QStringLiteral("location"));

        horizontalLayout_8->addWidget(location);

        horizontalSpacer_15 = new QSpacerItem(40, 20, QSizePolicy::Minimum, QSizePolicy::Minimum);

        horizontalLayout_8->addItem(horizontalSpacer_15);


        verticalLayout->addLayout(horizontalLayout_8);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName(QStringLiteral("horizontalLayout_7"));
        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::MinimumExpanding, QSizePolicy::Minimum);

        horizontalLayout_7->addItem(horizontalSpacer_2);

        record_button = new QPushButton(BagWidget);
        record_button->setObjectName(QStringLiteral("record_button"));

        horizontalLayout_7->addWidget(record_button);

        horizontalSpacer = new QSpacerItem(10, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_7->addItem(horizontalSpacer);

        stop_button = new QPushButton(BagWidget);
        stop_button->setObjectName(QStringLiteral("stop_button"));

        horizontalLayout_7->addWidget(stop_button);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_7->addItem(horizontalSpacer_3);


        verticalLayout->addLayout(horizontalLayout_7);


        horizontalLayout->addLayout(verticalLayout);


        retranslateUi(BagWidget);

        QMetaObject::connectSlotsByName(BagWidget);
    } // setupUi

    void retranslateUi(QWidget *BagWidget)
    {
        BagWidget->setWindowTitle(QApplication::translate("BagWidget", "Log Section", 0));
        label_2->setText(QApplication::translate("BagWidget", "Experiment Name:", 0));
        label->setText(QApplication::translate("BagWidget", "Liquid Number:", 0));
        label_3->setText(QApplication::translate("BagWidget", "Distance Betw. Coils:", 0));
        label_4->setText(QApplication::translate("BagWidget", "Microrobot Number:", 0));
        label_5->setText(QApplication::translate("BagWidget", "Name - Surname:", 0));
        label_6->setText(QApplication::translate("BagWidget", "Location:", 0));
        record_button->setText(QApplication::translate("BagWidget", "Record", 0));
        stop_button->setText(QApplication::translate("BagWidget", "Stop", 0));
    } // retranslateUi

};

namespace Ui {
    class BagWidget: public Ui_BagWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LOG_SECTION_H
