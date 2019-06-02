#include <ros/ros.h>
#include <rosbag.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream.h>
#include <vector>
#include <string>

namespace csv_to_bag
{
    class CsvToBag
    {
    public:
        CsvToBag();

        ~CsvToBag();

        void parse()
        {
            std::string tokens;
            std::getline(file, tokens);

        }
    private:
        ros::NodeHandle nh;
        std::string file_name;

    };
}

int main(int argc, char *argv[])
{


    return 0;
}
