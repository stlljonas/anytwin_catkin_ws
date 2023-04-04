/*
 * BagLoader.hpp
 *
 *  Created on: Apr 5, 2016
 *      Author: gabrielhottiger
 */

#include <rosbag/bag.h>
#include <std_msgs/Header.h>

#pragma once

namespace bageditor {

class BagLoader {

 public:
    // Only construction with file path allowed
    BagLoader() = delete;

    // Constructor
    BagLoader(std::string path):
        path_{path},
        bag_()
    {
        // open bagfile
        try{
            bag_.open(path_, rosbag::bagmode::Read);
        }
        catch(rosbag::BagException& e) {
            throw rosbag::BagException("[BagLoader]: Could not load bagfile!");
        }
    }

    ~BagLoader() {
        // close bagfile
        bag_.close();
    }

    //! This function gets a vector of DataType_ extracted from a topic and satisfying a condition.
    //  The string "topic" defines the name under which the topic was recorded on to the bagfile.
    //  The function "condition" filters the incoming topics based on a boolean expression. It only writes
    //  msgs into the datavector  when the condition returns true for that message.
    //  The function "getData" specifies which field of a message shall be written into the data vector.
    template<typename MessageType_, typename DataType_>
    std::vector<DataType_> getDataFromTopic(const std::string & topic,
                                            const std::function<bool(MessageType_)> & condition,
                                            const std::function<DataType_(MessageType_)> & getData);

    // The following functions provide overloaded implementations of getDataFromTopic
    template<typename MessageType_>
    std::vector<MessageType_> getDataFromTopic(const std::string & topic,
                                               const std::function<bool(MessageType_)> & condition);

    template<typename MessageType_, typename DataType_>
    std::vector<DataType_> getDataFromTopic(const std::string & topic,
                                            const std::function<DataType_(MessageType_)> & getData);

    template<typename MessageType_>
    std::vector<MessageType_> getDataFromTopic(const std::string & topic);

    template<typename MessageType_, typename DataType_>
    std::vector< std::pair<ros::Time, DataType_> > getTimedDataFromTopic(const std::string & topic,
                                                                         const std::function<bool(MessageType_)> & condition,
                                                                         const std::function<DataType_(MessageType_)> & getData);

    template<typename MessageType_>
    std::vector< std::pair<ros::Time, MessageType_> > getTimedDataFromTopic(const std::string & topic,
                                                                            const std::function<bool(MessageType_)> & condition);

    template<typename MessageType_, typename DataType_>
    std::vector< std::pair<ros::Time, DataType_> > getTimedDataFromTopic(const std::string & topic,
                                                                         const std::function<DataType_(MessageType_)> & getData);

    template<typename MessageType_>
    std::vector< std::pair<ros::Time, MessageType_> > getTimedDataFromTopic(const std::string & topic);

    // This function uses a sequence number range as condition
    template<typename MessageType_, typename DataType_>
    std::vector<DataType_> getDataFromTopicInSeqRange(const std::string & topic,
                                                      const std::function<DataType_(MessageType_)> & getData,
                                                      const std::function<std_msgs::Header(MessageType_)> & getHeader,
                                                      const int startSeq,
                                                      const int stopSeq);

    template<typename MessageType_>
    std::vector<MessageType_> getDataFromTopicInSeqRange(const std::string & topic,
                                                         const std::function<std_msgs::Header(MessageType_)> & getHeader,
                                                         const int startSeq,
                                                         const int stopSeq);

    // This function uses a time range as condition
    template<typename MessageType_, typename DataType_>
    std::vector<DataType_> getDataFromTopicInTimeRange(const std::string & topic,
                                                       const std::function<DataType_(MessageType_)> & getData,
                                                       const std::function<std_msgs::Header(MessageType_)> & getHeader,
                                                       const double startTimeInSec,
                                                       const double stopTimeInSec);

    template<typename MessageType_>
    std::vector<MessageType_> getDataFromTopicInTimeRange(const std::string & topic,
                                                          const std::function<std_msgs::Header(MessageType_)> & getHeader,
                                                          const double startTimeInSec,
                                                          const double stopTimeInSec);

    template<typename MessageType_>
    ros::Time getFirstMessageTime(const std::string & topic);

    template<typename MessageType_>
    ros::Time getLastMessageTime(const std::string & topic);

 private:
    std::string path_;
    rosbag::Bag bag_;
};

}

#include <bageditor/BagLoader.tpp>
