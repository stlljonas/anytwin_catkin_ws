//
// Created by pleemann on 25.05.17.
//

#ifndef COSMO_DEBUG_LEVEL1
#define COSMO_DEBUG_LEVEL1
#define COSMO_DEBUG_LEVEL2
#endif

#include "cosmo/cosmo.hpp"

#include "message_logger/message_logger.hpp"

#include "cosmo_example/TestMessage.hpp"

#include <sys/time.h>
#include <atomic>
#include <csignal>

std::atomic<bool> g_running{true};
cosmo::SubscriberPtr<TestMessage> g_subscriber;

void signalCallback(int) {
  g_running = false;
}

void callback(const TestMessage& msg) {
  // Note that the reference points to shared memory, so you should copy the data to be thread safe and not e.g. save a pointer to it
  TestMessage localMsg = msg;

  // if you want to execute complex calculations in the callback, you should unlock the Subscriber's internal mutex before doing them
  g_subscriber->unlockMutex();
  MELO_INFO_STREAM("Subscriber received message on /test4: a:" << localMsg.a_ << "\n  b: " << localMsg.b_[0] << ", " << localMsg.b_[1]
                                                               << ", " << localMsg.b_[2] << ", " << localMsg.b_[3] << ", "
                                                               << localMsg.b_[4]);
  usleep(500000);
}

class MyClass {
 public:
  void callback0(const TestMessage& msg) {
    // Note that the reference points to shared memory, so you should copy the data to be thread safe and not e.g. save a pointer to it
    MELO_INFO_STREAM("Subscriber received message on /test0: a:" << msg.a_ << "\n  b: " << msg.b_[0] << ", " << msg.b_[1] << ", "
                                                                 << msg.b_[2] << ", " << msg.b_[3] << ", " << msg.b_[4]);
  }
  void callback1(const TestMessage& msg) {
    // Note that the reference points to shared memory, so you should copy the data to be thread safe and not e.g. save a pointer to it
    MELO_INFO_STREAM("Subscriber received message on /test1: a:" << msg.a_ << "\n  b: " << msg.b_[0] << ", " << msg.b_[1] << ", "
                                                                 << msg.b_[2] << ", " << msg.b_[3] << ", " << msg.b_[4]);
  }
  void callback2(const TestMessage& msg) {
    // Note that the reference points to shared memory, so you should copy the data to be thread safe and not e.g. save a pointer to it
    MELO_INFO_STREAM("Subscriber received message on /test2: a:" << msg.a_ << "\n  b: " << msg.b_[0] << ", " << msg.b_[1] << ", "
                                                                 << msg.b_[2] << ", " << msg.b_[3] << ", " << msg.b_[4]);
  }
  void callback3(const TestMessage& msg) {
    // Note that the reference points to shared memory, so you should copy the data to be thread safe and not e.g. save a pointer to it
    MELO_INFO_STREAM("Subscriber received message on /test3: a:" << msg.a_ << "\n  b: " << msg.b_[0] << ", " << msg.b_[1] << ", "
                                                                 << msg.b_[2] << ", " << msg.b_[3] << ", " << msg.b_[4]);
  }
};

int main() {
  signal(SIGINT, signalCallback);

  MyClass instance;

  std::array<cosmo::SubscriberPtr<TestMessage>, 5> subscribers;

  // use helper function nr1
  subscribers[0] = cosmo::subscribeShm("/test0", &MyClass::callback0, &instance);

  // use helper function nr2
  {
    cosmo::SubscriberOptionsPtr<TestMessage> options(
        new cosmo::SubscriberOptions<TestMessage>("/test1", std::bind(&MyClass::callback1, &instance, std::placeholders::_1)));
    subscribers[1] = cosmo::subscribeShm<TestMessage>(options);
  }
  // use helper function nr2 and subscribe "latched", receiving last published message upon subscribing
  {
    cosmo::SubscriberOptionsPtr<TestMessage> options(
        new cosmo::SubscriberOptions<TestMessage>("/test2", std::bind(&MyClass::callback2, &instance, std::placeholders::_1)));
    options->latch_ = true;
    subscribers[2] = cosmo::subscribeShm<TestMessage>(options);
  }

  // directly create subscriber, without helper function. Requires to call start()
  {
    cosmo::SubscriberOptionsPtr<TestMessage> options(
        new cosmo::SubscriberOptions<TestMessage>("/test3", std::bind(&MyClass::callback3, &instance, std::placeholders::_1)));
    subscribers[3].reset(new cosmo::Subscriber<TestMessage>(options));
    subscribers[3]->start();
  }

  // we can also use global functions as callbacks
  subscribers[4] = cosmo::subscribeShm<TestMessage>(
      std::make_shared<cosmo::SubscriberOptions<TestMessage>>("/test4", std::bind(callback, std::placeholders::_1)));

  g_subscriber = subscribers[4];

  while (g_running) {
    MELO_INFO("=========================================");
    // use /test4 topic as "clock", because this is the topic the publisher program publishes last
    // wait for an amount of time which is higher than the publish timestep
    // (you could also wait for a shorter amount, but then you will have receive(..) calls without effect)
    subscribers[4]->receive(std::chrono::microseconds{1200000});

    // the other publishers already have new data, no need for a timeout
    for (unsigned int i = 0; i < 4; ++i) {
      subscribers[i]->receive(std::chrono::microseconds{0});
    }
  }

  return 0;
}
