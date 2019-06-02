/* @author Aytac Kahveci */
#include <teme_bag/teme_bag.h>

#include <pluginlib/class_list_macros.h>

namespace record_bag
{
    RecordBag::RecordBag() 
    : rqt_gui_cpp::Plugin(), widget_(0)
    {
        setObjectName("LogSection");
    }

    RecordBag::~RecordBag()
    {
        if(file_)
            delete file_;

        if(file2_)
            delete file2_;

        if(file_kill_)
            delete file_kill_;
    }

    void RecordBag::initPlugin(qt_gui_cpp::PluginContext& context)
    {
        widget_ = new QWidget();
        ui_.setupUi(widget_);

        if (context.serialNumber() > 1)
        {
            widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
        }
        context.addWidget(widget_);
        connect(ui_.record_button, SIGNAL(clicked()), this, SLOT(recordButton()) );
        connect(ui_.stop_button, SIGNAL(clicked()), this, SLOT(stopButton()) );

        ui_.microrobot_number->setValidator(new QIntValidator(0, 100, this));
        connect(ui_.microrobot_number, SIGNAL(textChanged(const QString&)), this, SLOT(microrobotNumber(const QString&)) );

        ui_.liquid_number->setValidator(new QIntValidator(0, 100, this));
        connect(ui_.liquid_number, SIGNAL(textChanged(const QString&)), this, SLOT(liquidNumber(const QString&)) );

        ui_.distance->setValidator(new QDoubleValidator(0, 100, 2, this));
        connect(ui_.distance, SIGNAL(textChanged(const QString&)), this,SLOT(distance(const QString&)) );

        connect(ui_.experiment_name, SIGNAL(textChanged(const QString&)), this, SLOT(experimentName(const QString&)) );
        connect(ui_.name, SIGNAL(textChanged(const QString&)), this, SLOT(nameHandler(const QString&)) );
        connect(ui_.location, SIGNAL(textChanged(const QString&)), this, SLOT(locationHandler(const QString&)) );
    }

    void RecordBag::microrobotNumber(const QString&)
    {
        microrobot_num_ = ui_.microrobot_number->text().toInt();
    }

    void RecordBag::liquidNumber(const QString&)
    {
        liquid_num_ = ui_.liquid_number->text().toInt();
    }

    void RecordBag::distance(const QString&)
    {
        distance_between_coils_ = ui_.distance->text().toDouble();
    }

    void RecordBag::experimentName(const QString&)
    {
        experiment_name_ = ui_.experiment_name->text().toStdString();
    }

    void RecordBag::nameHandler(const QString&)
    {
        name_ = ui_.name->text().toStdString();
    }

    void RecordBag::locationHandler(const QString&)
    {
        location_ = ui_.location->text().toStdString();
    }

    void RecordBag::recordButton()
    {
        // Create MetaData
        std::string topics[] = {"/camera_node_bottom/positionDist", "/camera_node_front/positionDist", "/joint_states", "/dynamixel/joint_states", "/coil_pid_controller/coil_states", "/microrobot/pose", "/microrobot/pose_conj", "/goal/visualizer"};
        int i = 1;
        std::string path = location_ + "/" + std::to_string(i);
        for( ; folderExists(path); )
        {
            i++;
            path = location_ + "/" + std::to_string(i);
        }
        boost::filesystem::create_directory(path);
        
        try
        {
            std::string text_file = path + "/meta_data.txt";
            std::ofstream fd(text_file.c_str(), std::ios_base::out);
            if(fd.good())
            {
                char buffer[100];
                snprintf(buffer, 100, "Experiment name: %s \n", experiment_name_.c_str());
                fd << buffer;

                snprintf(buffer, 100, "Name: %s \n", name_.c_str());
                fd << buffer;

                snprintf(buffer, 100, "Liquid Num: %d \n", liquid_num_);
                fd << buffer;

                snprintf(buffer, 100, "Microrobot Num: %d \n", microrobot_num_);
                fd << buffer;

                snprintf(buffer, 100, "Distance Between Coils: %f \n", distance_between_coils_);
                fd << buffer;

                fd.close();
            }

            int arr_size = sizeof(topics) / sizeof(topics[0]);

            std::string sub_topics = "";
            for(int i = 0; i < arr_size; i++)
            {
                sub_topics += topics[i] + " ";
            }
            
            std::string bag_name = "Ex";
            std::string command = "cd "+ path + ";" + "rosbag record " + sub_topics + "-o " + bag_name; 
            file_ = popen(command.c_str(), "w");

            command = "roslaunch image_processing record.launch filename1:=" + path + "/outputRawront " + "filename2:=" + path + "/outputRawFrontProcessed " + "filename3:=" + path + "/outputRawBottom " +
                        "filename4:=" + path + "/outputRawBottomProcessed";
            file2_ = popen(command.c_str(), "w");
        }
        catch(...)
        {
            printf("Error in rosbag command.\n");
        }
    }

    void RecordBag::stopButton()
    {
        try 
        {
            std::string command = "rosnode list";
            file_ = popen(command.c_str(), "r");
            if(!file_)
            {
                printf("Couldn't start command");
                throw("Couldn't start command");
            }
            while(fgets(buffer_, 100, file_) != NULL)
            {
                //printf("%s \n", buffer_);
                std::string temp(buffer_);
                if(temp.find("/record", 0) != std::string::npos)
                {
                    printf("Pattern is found, %s \n", buffer_);
                    std::string kill_command = "rosnode kill " + temp;
                    file_kill_ = popen(kill_command.c_str(), "w");
                    if(!file_kill_)
                    {
                        printf("erorr killing node \n");
                    }
                }
                if(temp.find("video_recorder", 0) != std::string::npos)
                {
                    printf("Pattern is found, %s \n", buffer_);
                    std::string kill_command = "rosnode kill " + temp;
                    file_kill_ = popen(kill_command.c_str(), "w");
                    if(!file_kill_)
                    {
                        printf("erorr killing node \n");
                    }
                }
            }
        }
        catch(...)
        {
            printf("Error in rosnode list command.\n");
        }
    }

    void RecordBag::shutdownPlugin()
    {}

    void RecordBag::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
    {}

    void RecordBag::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
    {}

    bool RecordBag::folderExists(const std::string& path)
    {
        boost::filesystem::path dir(path);
        return boost::filesystem::exists(dir);
    }
}

PLUGINLIB_EXPORT_CLASS(record_bag::RecordBag, rqt_gui_cpp::Plugin)
