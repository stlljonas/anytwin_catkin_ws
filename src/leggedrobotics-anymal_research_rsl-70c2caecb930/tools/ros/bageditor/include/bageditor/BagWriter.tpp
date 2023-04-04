namespace bageditor {

template<typename MessageType_>
void BagWriter::writeTimedMessagesToTopic(const std::string & topic,
                              const std::vector< std::pair<ros::Time, MessageType_> > & timedData)
{
    for(const auto & td : timedData) {
        writeTimedMessageToTopic(topic, td.first, td.second);
    }
}

template<typename MessageType_>
void BagWriter::writeTimedMessageToTopic(const std::string & topic,
                              			 const ros::Time& time , const MessageType_ & data)
{
	try{
		bag_.write(topic, time, data);
    }
    catch(rosbag::BagException& e) {
        ROS_WARN_STREAM("Could not write stamped message to topic. "<< e.what() );
    }
}

template<typename MessageType_>
void BagWriter::writeStampedMessagesToTopic(const std::string & topic,
                                const std::vector<MessageType_> & messages)
{
    for(const auto & m : messages) {
        writeStampedMessageToTopic(topic, m);
    }
}

template<typename MessageType_>
void BagWriter::writeStampedMessageToTopic(const std::string & topic,
                                const MessageType_ & message)
{
    try{
		bag_.write(topic, message.header.stamp, message);
    }
    catch(rosbag::BagException& e) {
        ROS_WARN_STREAM("Could not write stamped message to topic. "<< e.what() );
    }
}

}
