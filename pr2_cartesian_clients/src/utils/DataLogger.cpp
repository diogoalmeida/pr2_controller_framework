#include <utils/DataLogger.hpp>

namespace pr2_cartesian_clients{

  DataLogger::DataLogger()
  {
    ROS_DEBUG("Creating a data logger!");
    options_.append_date = false;
    options_.trigger = false;
    #if ROS_VERSION_MINIMUM(1, 12, 6) // min_space does not exist in hydro. This checks for indigo
      options_.min_space = 0;
    #endif
    options_.verbose = true;
    options_.split = false;
    options_.regex = false;
    recorder_ = new rosbag::Recorder(options_);
    ROS_DEBUG("Data logger created successfully!");
  }

  DataLogger::DataLogger(const rosbag::RecorderOptions &options)
  {
    options_ = options;
    recorder_ = new rosbag::Recorder(options);
  }

  DataLogger::~DataLogger()
  {
    if (record_thread_.joinable())
    {
      ROS_DEBUG("Joining previous thread!");
      record_thread_.join();
    }

    delete recorder_;
  }

  void DataLogger::addRecordTopic(const std::string &topic_name)
  {
    ROS_DEBUG("Adding record topic: %s", topic_name.c_str());
    recorder_->subscribe(topic_name);
  }

  bool DataLogger::startRecording(const std::string &bag_name, const double time)
  {
    ROS_DEBUG("Starting recording in name %s for %.2f seconds!", bag_name.c_str(), time);
    {
      std::lock_guard<std::mutex> lock(record_mutex_);
      if (is_recording_) // DataLogger supports only one recording at a time
      {
        return false;
      }
    }

    ROS_DEBUG("Topic list size: %d", (int) options_.topics.size());

    if (record_thread_.joinable())
    {
      ROS_DEBUG("Joining previous thread!");
      record_thread_.join();
    }

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
    ROS_DEBUG("Recorder thread is active!");
    {
      std::lock_guard<std::mutex>lock(record_mutex_);
      is_recording_ = true;
    }

    ROS_DEBUG("Running the recorder");
    recorder_->run();

    {
      std::lock_guard<std::mutex>lock(record_mutex_);
      is_recording_ = false;
    }

    ROS_DEBUG("Recorder thread is finished!");
    return 1;
  }
}
