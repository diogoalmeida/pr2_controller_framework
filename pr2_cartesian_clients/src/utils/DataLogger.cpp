#include <utils/DataLogger.hpp>

namespace pr2_cartesian_clients{

  DataLogger::DataLogger()
  {
    ROS_DEBUG("Creating a data logger!");
    options_.append_date = false;
    options_.trigger = false;
    options_.snapshot = true;
    #if ROS_VERSION_MINIMUM(1, 11, 9) // min_space does not exist in hydro. This checks for indigo
      options_.min_space = 0;
    #endif
    options_.verbose = false;
    options_.split = true;
    options_.regex = false;
    is_recording_ = false;
    recorder_ = 0;
    ROS_DEBUG("Data logger created successfully!");
  }

  DataLogger::DataLogger(const rosbag::RecorderOptions &options)
  {
    is_recording_ = false;
    recorder_ = 0;
    options_ = options;
  }

  DataLogger::~DataLogger()
  {
    // if (record_thread_.joinable())
    // {
    //   ROS_DEBUG("Joining previous thread!");
    //   record_thread_.join();
    // }

    if (recorder_)
    {
      delete recorder_;
    }
  }

  void DataLogger::addRecordTopic(const std::string &topic_name)
  {
    ROS_DEBUG("Adding record topic: %s", topic_name.c_str());
    options_.topics.push_back(topic_name);
  }

  bool DataLogger::startRecording(const std::string &bag_name, const double time)
  {
    ROS_DEBUG("Starting recording in name %s for %.2f seconds!", bag_name.c_str(), time);
    options_.prefix = bag_name;
    options_.max_duration = ros::Duration(time);
    {
      // std::lock_guard<std::mutex> lock(record_mutex_);
      if (is_recording_) // DataLogger supports only one recording at a time
      {
        ROS_WARN("Previous record is not finished");
        return false;
      }
    }

    ROS_DEBUG("Topic list size: %d", (int) options_.topics.size());

    if (record_thread_.joinable())
    {
      ROS_DEBUG("Joining previous thread!");
      // record_thread_.join();
      record_thread_.detach(); // :(
    }

    ROS_DEBUG("Running the recorder");
    is_recording_ = true;
    record_thread_ = std::thread(&DataLogger::runRecorder, this);

    return true;
  }

  void DataLogger::stopRecording()
  {
    ROS_DEBUG("Stopping recording!");
    // std::lock_guard<std::mutex> lock(record_mutex_);
    if (is_recording_) // DataLogger supports only one recording at a time
    {
      ROS_DEBUG("Current recorder is recording");
      if (recorder_)
      {
        ROS_DEBUG("Current recorder is alive");
        try
        {
          delete recorder_;
        }
        catch(...)
        {
          ROS_ERROR("omg");
        }
        recorder_ = 0;
      }

      is_recording_ = false;
    }

    ROS_DEBUG("Checking thread");

    if (record_thread_.joinable())
    {
      ROS_DEBUG("Joining previous thread!");
      // record_thread_.join();
      record_thread_.detach(); // :(
    }
  }

  bool DataLogger::saveData()
  {
    if (is_recording_)
    {
      recorder_->doTrigger();
      return true;
    }

    return false;
  }

  bool DataLogger::isRecording()
  {
    // std::lock_guard<std::mutex> lock(record_mutex_);
    return is_recording_;
  }

  int DataLogger::runRecorder()
  {
    ROS_DEBUG("Recorder thread is active!");
    // {
      // std::lock_guard<std::mutex>lock(record_mutex_);
    // }

    is_recording_ = true;
    ROS_DEBUG("Getting new recorder");
    recorder_ = new rosbag::Recorder(options_);
    ROS_DEBUG("run()");
    recorder_->run();
    is_recording_ = false;
    // if (recorder_)
    // {
    //   delete recorder_;
    //   recorder_ = 0;
    // }

    // {
    //   std::lock_guard<std::mutex>lock(record_mutex_);
    //   is_recording_ = false;
    // }

    ROS_DEBUG("Recorder thread is finished!");
    return 1;
  }
}
