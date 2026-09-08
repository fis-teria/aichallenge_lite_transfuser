// Serialized message and publisher GID from the SAME rmw take. No publishers.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <chrono>
#include <iostream>
#include <thread>
#include <ctime>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

static std::string hex(const uint8_t *p, size_t n) {
  static constexpr char digits[]="0123456789abcdef";
  std::string s; s.reserve(2*n);
  for(size_t i=0;i<n;++i){s+=digits[p[i]>>4];s+=digits[p[i]&15];}
  return s;
}
static void send_line(const std::string &s) {
  // A stalled Python consumer must not leave this child blocked in stdout.
  auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(200);
  size_t offset=0;
  while(offset<s.size()) {
    if(std::chrono::steady_clock::now()>=deadline) throw std::runtime_error("IPC_BACKPRESSURE");
    pollfd fd{STDOUT_FILENO,POLLOUT,0};
    if(poll(&fd,1,10)<0) throw std::runtime_error("IPC_POLL");
    if(!(fd.revents&POLLOUT)) continue;
    auto n=write(STDOUT_FILENO,s.data()+offset,s.size()-offset);
    if(n>0) offset+=static_cast<size_t>(n);
    else if(errno!=EAGAIN && errno!=EINTR) throw std::runtime_error("IPC_WRITE");
  }
}
int main(int argc,char **argv) {
  // Fixed finite worker: 25 wall seconds / 2000 messages / 1 MiB per message.
  if(argc<4 || (argc-1)%3 || argc>22) return 2;
  try {
    rclcpp::init(0,nullptr);
    auto options=rclcpp::NodeOptions().enable_rosout(false).start_parameter_services(false)
      .start_parameter_event_publisher(false).use_global_arguments(false);
    auto node=std::make_shared<rclcpp::Node>("v4_gid_receiver",options);
    std::vector<std::string> roles;
    std::vector<rclcpp::GenericSubscription::SharedPtr> subs;
    for(int i=1;i<argc;i+=3) {
      std::string role=argv[i];
      if(role.empty() || role.find_first_not_of("abcdefghijklmnopqrstuvwxyz_")!=std::string::npos || argv[i+1][0]!='/')
        throw std::runtime_error("INVALID_BINDING");
      roles.push_back(role);
      subs.push_back(node->create_generic_subscription(argv[i+1],argv[i+2],rclcpp::SensorDataQoS(),
                     [](std::shared_ptr<rclcpp::SerializedMessage>){}));
    }
    if(fcntl(STDOUT_FILENO,F_SETFL,fcntl(STDOUT_FILENO,F_GETFL)|O_NONBLOCK)<0) throw std::runtime_error("IPC_FLAGS");
    auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(25);
    size_t count=0;
    while(rclcpp::ok() && std::chrono::steady_clock::now()<deadline && count<2000) {
      for(size_t i=0;i<subs.size() && count<2000;++i) {
        rclcpp::SerializedMessage msg; rclcpp::MessageInfo info;
        if(!subs[i]->take_serialized(msg,info)) continue;
        timespec stamp{}; if(clock_gettime(CLOCK_MONOTONIC,&stamp)) throw std::runtime_error("CLOCK");
        const auto &raw=msg.get_rcl_serialized_message();
        if(raw.buffer_length>1048576) throw std::runtime_error("MESSAGE_SIZE_LIMIT");
        const auto &gid=info.get_rmw_message_info().publisher_gid;
        send_line(roles[i]+" "+hex(gid.data,RMW_GID_STORAGE_SIZE)+" "+
          std::to_string(uint64_t(stamp.tv_sec)*1000000000ULL+stamp.tv_nsec)+" "+
          hex(raw.buffer,raw.buffer_length)+"\n");
        ++count;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    rclcpp::shutdown(); return 0;
  } catch(const std::exception &e) {std::cerr<<e.what()<<std::endl;return 1;}
}
