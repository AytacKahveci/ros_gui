#ifndef PROGRAM_MODE_H_
#define PROGRAM_MODE_H_

#include <stdlib.h>
#include <iostream>
#include <string>

#include <rqt_gui_cpp/plugin.h>
#include <rqt_program_mode/ui_launcher.h>

#include <QWidget>
#include <QString>

namespace program_mode
{
    class ProgramMode : public rqt_gui_cpp::Plugin
    {
        Q_OBJECT

    public:
        ProgramMode();

        ~ProgramMode();

        virtual void initPlugin(qt_gui_cpp::PluginContext& context);

        virtual void shutdownPlugin();

        virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;

        virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

    protected slots:
        virtual void kukaButton();

        virtual void coilControllerButton();

        virtual void cameraBottomButton();

        virtual void cameraFrontButton();

        virtual void dynamixelManButton();

        virtual void dynamixelAutButton();

        virtual void dynamixelCalibButton();

        virtual void customLaunchButton();

        virtual void closeButton();

        virtual void modeComboBox(int);

    protected:
        QWidget* widget_;
        Ui::Launcher ui_;

    private:
        FILE* file_;
        FILE* file2_;
        FILE* file_kill_;
        char buffer_[100];

        std::string kuka_command = "roslaunch kuka_hw_cart_vel test.launch";
        std::string coil_command = "roslaunch coil_controller coilPidControl.launch";
        std::string camera_bottom = "roslaunch image_processing camera_node.launch";
        std::string camera_front = "roslaunch image_processing camera_front.launch";
        std::string dynamixel_man = "roslaunch dynamixel_custom_controller commonControl.launch; rosrun dynamixel_custom_controller keyboardControl";
        std::string dynamixel_aut = "roslaunch dynamixel_custom_controller commonControl.launch";
        std::string dynamixel_calib = "roslaunch dynamixel_custom_controller dynamixel_limits.launch; rosrun dynamixel_custom_controller dynamixel_limits_node";

        std::string combo_box_command_;
    };
} // namespace program_mode

#endif