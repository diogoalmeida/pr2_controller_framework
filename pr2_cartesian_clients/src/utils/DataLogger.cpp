#include <utils/DataLogger.hpp>

namespace pr2_cartesian_clients{

  DataLogger::DataLogger()
  {
    options_.append_date = false;
    options_.trigger = false;
    #if ROS_VERSION_MINIMUM(1, 12, 6) // min_space does not exist in hydro. This checks for indigo
      options_.min_space = 0;
    #endif
    options_.verbose = false;
    options_.split = false;
  }

  DataLogger::DataLogger(const rosbag::RecorderOptions &options)
  {
    options_ = options;
  }

  DataLogger::~DataLogger()
  {
    // Bye!
  }

  void DataLogger::addRecordTopic(const std::string &topic_name)
  {
    options_.topics.push_back(topic_name);
  }

  bool DataLogger::startRecording(const std::string &bag_name, const double time)
  {
    std::lock_guard<std::mutex> lock(record_mutex_);
    if (is_recording_) // DataLogger supports only one recording at a time
    {
      return false;
    }

    record_thread_.join();
    options_.max_duration = ros::Duration(time);
    record_thread_ = std::thread(&DataLogger::runRecorder, this);

    return true;
  }

  bool DataLogger::isRecording()
  {
    std::lock_guard<std::mutex> lock(record_mutex_);
    return is_recording_;
  }

  int DataLogger::runRecorder()
  {
    {
      std::lock_guard<std::mutex>lock(record_mutex_);
      is_recording_ = true;
    }

    rosbag::Recorder recorder(options_);
    recorder.run();

    {
      std::lock_guard<std::mutex>lock(record_mutex_);
      is_recording_ = false;
    }

    return 1;
  }
}
