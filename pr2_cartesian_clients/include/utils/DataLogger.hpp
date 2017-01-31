#ifndef __DATA_LOGGER__
#define __DATA_LOGGER__

#include <ros/ros.h>
#include <rosbag/recorder.h>
#include <thread>
#include <mutex>
#include <chrono>

namespace pr2_cartesian_clients{
  /**
    Utility class designed to log data from one experiment.

    It encapsulates the rosbag::recorder class such that
    the recorder will run in a separate thread
  */
  class DataLogger
  {
  public:
    DataLogger();
    /**
      This constructor overrides all the default recording options.

      @param options The new recording options to be used by the data logger
    */
    DataLogger(const rosbag::RecorderOptions &options);
    ~DataLogger();

    /**
      Starts recording a new bag for a pre-defined amount of time.

      @param bag_name The name of the resulting bag file.
      @param time Maximum amount of time for the recording.
      @return True if recording was started, false if a previous record hasn't finished.
    */
    bool startRecording(const std::string &bag_name, const double time);

    /**
      Check if a previous bag is being recorded.

      @return True if an active recording is happening.
    */
    bool isRecording();

    /**
      Add a topic to be recorded.

      @param topic_name Name of the topic
    */
    void addRecordTopic(const std::string &topic_name);

  private:
    /**
      The recorder thread method.

      @return 1, if finished successfully.
    */
    int runRecorder();

  private:
    rosbag::RecorderOptions options_;
    rosbag::Recorder *recorder_;
    std::thread record_thread_;
    std::mutex record_mutex_;
    bool is_recording_;
  };
}
#endif
