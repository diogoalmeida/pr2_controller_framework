#include <ros/ros.h>
#include <utils/DataLogger.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "data_logger_test_node");
  ros::NodeHandle n;
  pr2_cartesian_clients::DataLogger logger;
  ros::Rate r(0.1);
  std::string bag_name;
  int count = 1;
  logger.addRecordTopic("/test_bed/feedback");

  ROS_INFO("Data logger test ready to start."
           " Before pressing enter to continue, set the log level to Debug in"
           " rqt_logger_level");

  std::cin.get();


  while (ros::ok())
  {
    ROS_INFO("Press a key to start logging data");
    bag_name = std::string("logger_test_") + std::to_string(count);
    count ++;

    logger.startRecording(bag_name, 1);
    std::cin.get();
    logger.saveData();
    logger.stopRecording();

    // ROS_INFO("Continuing");
    // r.sleep();
  }

  return 0;
}
