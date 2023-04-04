## BagLoader

### Description
Allows loading of a topics data to std::vector containers. Messages can be ignored based on a condition function. The data/msg that shall be extracted can be define by a getData function. An implementation of time and sequence number conditions is already provided.

### Usage
```cpp
// Construct a bagloader object with the bagfile path
BagLoader bag("myBag.bag");

// Define a lambda that accesses the data(e.g data field of std_msgs::Float64)
auto f = [](std_msgs::Float64 msg) { return msg.data; };

// Define a lambda that filters the data (only data greater than 5 is added to the vector)
auto selector = [](std_msgs::Float64 msg) { return msg.data > 5.0; };

// Get data as a vector
std::vector<double> = bag.getDataFromTopic<std_msgs::Float64, double>("/my_topic", selector ,f);

```
