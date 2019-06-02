/* @author Aytac Kahveci */
#include <stdio.h>
#include <string>
#include <unistd.h>
#include <vector>

#include <iostream>
#include <fstream>
#include <exception>
#include <sys/stat.h>

#include <rqt_gui_cpp/plugin.h>
#include <teme_bag/ui_log_section.h>

#include <QWidget>
#include <QString>
#include <QKeyEvent>

#include <boost/filesystem.hpp>

namespace record_bag
{
    class RecordBag : public rqt_gui_cpp::Plugin
    {
        Q_OBJECT 

    public:
        RecordBag();

        ~RecordBag();

        virtual void initPlugin(qt_gui_cpp::PluginContext& context);

        virtual void shutdownPlugin();

        virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;

        virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

    protected slots:
        virtual void recordButton();

        virtual void stopButton();

        virtual void microrobotNumber(const QString& );

        virtual void distance(const QString& );

        virtual void liquidNumber(const QString& );

        virtual void experimentName(const QString& );

        virtual void nameHandler(const QString& );

        virtual void locationHandler(const QString& );

    protected:
        QWidget* widget_;
        Ui::BagWidget ui_;

    private:
        FILE* file_;
        FILE* file2_;
        FILE* file_kill_;
        char buffer_[100];

        std::string location_ = "/home/aytac/Desktop";
        std::string bag_name_ = "exp";
        std::string command_ = "rosnode list";
        std::string experiment_name_ = "Sabit kuvvet deneyi";
        std::string name_ = "Aytac Kahveci";
        int liquid_num_ = 1;
        int microrobot_num_ = 1;
        double distance_between_coils_ = 15.0;

        bool folderExists(const std::string& path);
    };
}
