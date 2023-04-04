#include <boost/foreach.hpp>
#include <rosbag/view.h>

namespace bageditor {

template<typename MessageType_, typename DataType_>
std::vector<DataType_> BagLoader::getDataFromTopic(const std::string & topic,
                                                   const std::function<bool(MessageType_)> & condition,
                                                   const std::function<DataType_(MessageType_)> & getData)
{

    typedef boost::shared_ptr<MessageType_ const> MessageTypeConstPtr;

    std::vector<DataType_> data;

    rosbag::View view(bag_, rosbag::TopicQuery(topic));

    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        MessageTypeConstPtr msg = m.instantiate<MessageType_>();
        if (msg != nullptr)
        {
            if(condition(*msg)) {
                data.push_back(getData(*msg));
            }
        }
    }

    return data;
}

template<typename MessageType_>
std::vector<MessageType_> BagLoader::getDataFromTopic(const std::string & topic,
                                                      const std::function<bool(MessageType_)> & condition)
{
    return getDataFromTopic<MessageType_, MessageType_>(topic, condition, [](MessageType_ msg) { return msg; });
}

template<typename MessageType_, typename DataType_>
std::vector<DataType_> BagLoader::getDataFromTopic(const std::string & topic,
                                                   const std::function<DataType_(MessageType_)> & getData)
{
    return getDataFromTopic<MessageType_, DataType_>(topic, [](MessageType_ msg){ return true; }, getData);
}

template<typename MessageType_>
std::vector<MessageType_> BagLoader::getDataFromTopic(const std::string & topic)
{
    return getDataFromTopic<MessageType_, MessageType_>(topic, [](MessageType_ msg){ return true; }, [](MessageType_ msg) { return msg; });
}

template<typename MessageType_, typename DataType_>
std::vector<DataType_> BagLoader::getDataFromTopicInSeqRange(const std::string & topic,
                                                             const std::function<DataType_(MessageType_)> & getData,
                                                             const std::function<std_msgs::Header(MessageType_)> & getHeader,
                                                             const int startSeq,
                                                             const int stopSeq)
{
    return getDataFromTopic<MessageType_, DataType_>(topic, [&](MessageType_ msg){ return getHeader(msg).seq > startSeq && getHeader(msg).seq < stopSeq; }, getData);
}

template<typename MessageType_>
std::vector<MessageType_> BagLoader::getDataFromTopicInSeqRange(const std::string & topic,
                                                                const std::function<std_msgs::Header(MessageType_)> & getHeader,
                                                                const int startSeq,
                                                                const int stopSeq)
{
    return getDataFromTopic<MessageType_, MessageType_>(topic, [&](MessageType_ msg){ return getHeader(msg).seq > startSeq && getHeader(msg).seq < stopSeq; }, [](MessageType_ msg) { return msg; });
}

template<typename MessageType_, typename DataType_>
std::vector<DataType_> BagLoader::getDataFromTopicInTimeRange(const std::string & topic,
                                                              const std::function<DataType_(MessageType_)> & getData,
                                                              const std::function<std_msgs::Header(MessageType_)> & getHeader,
                                                              const double startTimeInSec,
                                                              const double stopTimeInSec)
{
    return getDataFromTopic<MessageType_, DataType_>(topic, [&](MessageType_ msg){ return getHeader(msg).stamp.toSec() > startTimeInSec && getHeader(msg).stamp.toSec() < stopTimeInSec; }, getData);
}

template<typename MessageType_>
std::vector<MessageType_> BagLoader::getDataFromTopicInTimeRange(const std::string & topic,
                                                                 const std::function<std_msgs::Header(MessageType_)> & getHeader,
                                                                 const double startTimeInSec,
                                                                 const double stopTimeInSec)
{
    return getDataFromTopic<MessageType_, MessageType_>(topic, [&](MessageType_ msg){ return getHeader(msg).stamp.toSec() > startTimeInSec && getHeader(msg).stamp.toSec() < stopTimeInSec; }, [](MessageType_ msg) { return msg; });
}


template<typename MessageType_, typename DataType_>
std::vector< std::pair<ros::Time, DataType_> > BagLoader::getTimedDataFromTopic(const std::string & topic,
                                                                           const std::function<bool(MessageType_)> & condition,
                                                                           const std::function<DataType_(MessageType_)> & getData)
{
    typedef boost::shared_ptr<MessageType_ const> MessageTypeConstPtr;

    std::vector< std::pair<ros::Time, DataType_> > data;

    rosbag::View view(bag_, rosbag::TopicQuery(topic));

    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        MessageTypeConstPtr msg = m.instantiate<MessageType_>();
        if (msg != nullptr)
        {
            if(condition(*msg)) {
                data.push_back(std::pair<ros::Time, DataType_>(m.getTime(), getData(*msg)));
            }
        }
    }

    return data;
}

template<typename MessageType_>
std::vector< std::pair<ros::Time, MessageType_> > BagLoader::getTimedDataFromTopic(const std::string & topic,
                                                                        const std::function<bool(MessageType_)> & condition)
{
    return getTimedDataFromTopic<MessageType_, MessageType_>(topic, condition, [](MessageType_ msg) { return msg; });
}

template<typename MessageType_, typename DataType_>
std::vector< std::pair<ros::Time, DataType_> > BagLoader::getTimedDataFromTopic(const std::string & topic,
                                                                     const std::function<DataType_(MessageType_)> & getData)
{
    return getTimedDataFromTopic<MessageType_, DataType_>(topic, [](MessageType_ msg){ return true; }, getData);
}

template<typename MessageType_>
std::vector< std::pair<ros::Time, MessageType_> > BagLoader::getTimedDataFromTopic(const std::string & topic)
{
    return getTimedDataFromTopic<MessageType_, MessageType_>(topic, [](MessageType_ msg){ return true; }, [](MessageType_ msg) { return msg; });
}


template<typename MessageType_>
ros::Time BagLoader::getFirstMessageTime(const std::string & topic)
{
    typedef boost::shared_ptr<MessageType_ const> MessageTypeConstPtr;

    rosbag::View view(bag_, rosbag::TopicQuery(topic));

    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        MessageTypeConstPtr msg = m.instantiate<MessageType_>();
        if (msg != nullptr)
        {
			return m.getTime();
        }
    }

    return ros::Time(0);
}

template<typename MessageType_>
ros::Time BagLoader::getLastMessageTime(const std::string & topic)
{
	ros::Time lastTime;
    typedef boost::shared_ptr<MessageType_ const> MessageTypeConstPtr;

    rosbag::View view(bag_, rosbag::TopicQuery(topic));

    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        MessageTypeConstPtr msg = m.instantiate<MessageType_>();
        if (msg != nullptr)
        {
			lastTime = m.getTime();
        }
    }

    return lastTime;
}

}
