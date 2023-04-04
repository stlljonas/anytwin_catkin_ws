/*
 * BagWriter.hpp
 *
 *  Created on: Apr 5, 2016
 *      Author: gabrielhottiger
 */

#include <rosbag/bag.h>
#include <std_msgs/Header.h>

#pragma once

namespace bageditor {

class BagWriter {

 public:
    // Only construction with file path allowed
    BagWriter() = delete;

    // Constructor
    BagWriter(std::string path, rosbag::bagmode::BagMode mode = rosbag::bagmode::BagMode::Write):
        path_{path},
        bag_()
    {
        // open bagfile
        try{
            bag_.open(path_, mode);
        }
        catch(rosbag::BagException& e) {
            throw rosbag::BagException("[BagWriter]: Could not open bagfile for writing!");
        }
    }

    ~BagWriter() {
        // close bagfile
        bag_.close();
    }

    template<typename MessageType_>
    void writeTimedMessagesToTopic(const std::string & topic,
                                   const std::vector< std::pair<ros::Time, MessageType_> > & timedData);

    template<typename MessageType_>
    void writeTimedMessageToTopic(const std::string & topic,
                                  const ros::Time& time , const MessageType_ & data);

    template<typename MessageType_>
    void writeStampedMessagesToTopic(const std::string & topic,
                                     const std::vector<MessageType_> & messages);
    template<typename MessageType_>
    void writeStampedMessageToTopic(const std::string & topic,
                                     const MessageType_ & message);

 private:
    std::string path_;
    rosbag::Bag bag_;
};

}

#include <bageditor/BagWriter.tpp>
