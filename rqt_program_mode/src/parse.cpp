#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <rqt_program_mode/csv.h>
#include <rosbag/bag.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/ros.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "parser");
    ros::NodeHandle nh;
    rosbag::Bag bag;
    bag.open("/home/aytac/ros_gui_ws/src/rqt_program_mode/src/ex.bag", rosbag::bagmode::Write);

    std_msgs::Float64MultiArray cartPos_;
    std_msgs::Float64MultiArray dynamixel_;
    std_msgs::Float64MultiArray coil_;

    std::vector<double> X_val;
    std::vector<double> Y_val;
    std::vector<double> Z_val;
    std::vector<double> A_val;
    std::vector<double> B_val;
    std::vector<double> C_val;
    std::vector<double> E_val;
    std::vector<double> Coil1_val;
    std::vector<double> Coil2_val;
    std::vector<double> Timestamp_val;


    io::CSVReader<10, io::trim_chars<' '>, io::no_quote_escape<','>, io::single_and_empty_line_comment<'#'> > in("/home/aytac/ros_gui_ws/src/rqt_program_mode/src/untitled.csv");
    io::CSVReader<3> inCoil("/home/aytac/ros_gui_ws/src/rqt_program_mode/src/untitled.csv");
    in.read_header(io::ignore_extra_column, "x", "y", "z", "a", "b", "c", "e", "coil1", "coil2", "timestamp");
    inCoil.read_header(io::ignore_extra_column, "coil1", "coil2", "timestamp");
    double x,y,z,a,b,c,e,coil1,coil2,timestamp;
    int dataSize = 0;
    while(in.read_row(x,y,z,a,b,c,e,coil1,coil2,timestamp))
    {
    // do stuff with the data
        X_val.push_back(x);
        Y_val.push_back(y);
        Z_val.push_back(z);
        A_val.push_back(a);
        B_val.push_back(b);
        C_val.push_back(c);
        E_val.push_back(e);
        Coil1_val.push_back(coil1);
        Coil2_val.push_back(coil2);
        Timestamp_val.push_back(timestamp);
        ROS_INFO("coil1:%f, coil2:%f, timestamp:%f", coil1, coil2, timestamp);
        dataSize++;
    }

    cartPos_.data.resize(6);
    dynamixel_.data.resize(4);
    coil_.data.resize(2);

    for(int i=0; i<dataSize; i++)
    {
        int tempTime = Timestamp_val[i] * 1e4;
        uint32_t sec, nsec;
        sec = tempTime / 1e4;
        nsec = tempTime % 10000;
        ROS_INFO("sec:%d, nsec:%d", sec, nsec);
        cartPos_.data[0] = X_val[i];
        cartPos_.data[1] = Y_val[i];
        cartPos_.data[2] = Z_val[i];
        cartPos_.data[3] = A_val[i];
        cartPos_.data[4] = B_val[i];
        cartPos_.data[5] = C_val[i];
        bag.write("/joint_position_controller/command", ros::Time(sec, nsec*1e5), cartPos_);

        coil_.data[0] = Coil1_val[i];
        coil_.data[1] = Coil2_val[i];
        bag.write("/coil_pid_controller/CoilController/command", ros::Time(sec, nsec*1e5), coil_);
    }

    bag.close();

    return 0;
}
