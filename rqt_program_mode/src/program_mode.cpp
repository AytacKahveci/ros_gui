/* @author Aytac Kahveci */
#include <rqt_program_mode/program_mode.h>

#include <pluginlib/class_list_macros.h>

namespace program_mode
{
    ProgramMode::ProgramMode() : rqt_gui_cpp::Plugin(), widget_(0)
    {
        setObjectName("rqt_program_mode");
    }

    ProgramMode::~ProgramMode()
    {
        if(file_)
        {
            pclose(file_);
            delete file_;
        }
    }

    void ProgramMode::initPlugin(qt_gui_cpp::PluginContext& context)
    {
        widget_ = new QWidget();
        ui_.setupUi(widget_);

        if (context.serialNumber() > 1)
        {
            widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
        }
        context.addWidget(widget_);
        connect(ui_.kuka_button, SIGNAL(clicked()), this, SLOT(kukaButton()) );
        connect(ui_.coil_controller_button, SIGNAL(clicked()), this, SLOT(coilControllerButton()) );
        connect(ui_.camera_bottom_button, SIGNAL(clicked()), this, SLOT(cameraBottomButton()) );
        connect(ui_.camera_front_button, SIGNAL(clicked()), this, SLOT(cameraFrontButton()) );
        connect(ui_.dynamixel_man_button, SIGNAL(clicked()), this, SLOT(dynamixelManButton()) );
        connect(ui_.dynamixel_aut_button, SIGNAL(clicked()), this, SLOT(dynamixelAutButton()) );
        connect(ui_.dynamixel_calib_button, SIGNAL(clicked()), this, SLOT(dynamixelCalibButton()) );
        connect(ui_.custom_launch_button, SIGNAL(clicked()), this, SLOT(customLaunchButton()) );
        connect(ui_.stop_button, SIGNAL(clicked()), this, SLOT(closeButton()) );

        ui_.modes_combo_box->addItem("Dynamixel Calibration");
        ui_.modes_combo_box->addItem("2D Positioning");
        ui_.modes_combo_box->addItem("3D Positioning");
        ui_.modes_combo_box->addItem("2D Polygon Path-Uncontrolled");
        ui_.modes_combo_box->addItem("Fixed Force");
        ui_.modes_combo_box->addItem("Helix Uncontrolled");

        connect(ui_.modes_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(modeComboBox(int)));
    }

    void ProgramMode::kukaButton()
    {
        
    }

    void ProgramMode::coilControllerButton()
    {

    }

    void ProgramMode::cameraBottomButton()
    {

    }

    void ProgramMode::cameraFrontButton()
    {

    }

    void ProgramMode::dynamixelManButton()
    {

    }

    void ProgramMode::dynamixelAutButton()
    {

    }

    void ProgramMode::dynamixelCalibButton()
    {

    }

    void ProgramMode::customLaunchButton()
    {
        try 
        {
            file_ = popen(combo_box_command_.c_str(), "w");
        }
        catch(...)
        {
            printf("Error in customLaunchButton. Command: %s\n", combo_box_command_.c_str());
        }
    }

    void ProgramMode::modeComboBox(int index)
    {
        switch(index)
        {
            case 0: // Dynamixel Calibration
            {
                combo_box_command_ = "rosrun rqt_program_mode dynamixel_calibration.sh";                
                break;
            }
            case 1: // 2D positioning
            {
                combo_box_command_ = "rosrun rqt_program_mode positioning_2d.sh";
                break;
            }
            case 2: // 3D positioning
            {
                combo_box_command_ = "rosrun rqt_program_mode positioning_3d.sh";
                break;
            }
            case 3: // 2D Polygon Path-Uncontrolled
            {

                break;
            }
            case 4: // Fixed Force
            {

                break;
            }
            case 5: //Helix Uncontrolled
            {

                break;
            }
            case -1:
                break;
            default:
                break;
        }
    }

    void ProgramMode::closeButton()
    {
        try 
        {
            file_kill_ = popen("rosnode kill --all", "w");
        }
        catch(...)
        {
            printf("Error in closeButton.\n");
        }
    }

    void ProgramMode::shutdownPlugin()
    {}

    void ProgramMode::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
    {}

    void ProgramMode::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
    {}

} // namespace program_mode

PLUGINLIB_EXPORT_CLASS(program_mode::ProgramMode, rqt_gui_cpp::Plugin)
