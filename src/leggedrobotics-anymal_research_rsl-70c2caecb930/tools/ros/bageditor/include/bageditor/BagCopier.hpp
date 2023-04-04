/*
 * BagLoader.hpp
 *
 *  Created on: Apr 5, 2016
 *      Author: Gabriel Hottiger
 */

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

#pragma once

namespace bageditor {

class BagCopier {

 public:
    // Only construction with file path allowed
    BagCopier() = delete;

    // Constructor
    BagCopier(std::string inputPath, std::string outputPath, rosbag::bagmode::BagMode mode = rosbag::bagmode::BagMode::Write):
        inputPath_{inputPath},
        outputPath_(outputPath),
        inputBag_(),
        outputBag_()
    {
        // open bagfile
        try{
            inputBag_.open(inputPath_, rosbag::bagmode::Read);
            outputBag_.open(outputPath_, mode);
        }
        catch(rosbag::BagException& e) {
            throw rosbag::BagException("[BagLoader]: Could not open bagfiles!");
        }
    }

    ~BagCopier() {
        // close bagfile
        inputBag_.close();
        outputBag_.close();
    }

    template < typename  MessageType_ >
    void copyTopic(const std::string& topic) {
        std::vector<std::string> topics = { topic };
        copyTopics<MessageType_>(topics);
    }

    template < typename MessageType_ >
    void copyTopics(const std::vector<std::string>& topics)
    {
        typedef boost::shared_ptr<MessageType_ const> MessageTypeConstPtr;

        rosbag::View view(inputBag_, rosbag::TopicQuery(topics));

        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            MessageTypeConstPtr msg = m.instantiate<MessageType_>();
            if (msg != nullptr)
            {
                outputBag_.write(m.getTopic(), m.getTime(), *msg);
            }
        }
    }

 private:
    std::string inputPath_;
    std::string outputPath_;
    rosbag::Bag inputBag_;
    rosbag::Bag outputBag_;
};

}
