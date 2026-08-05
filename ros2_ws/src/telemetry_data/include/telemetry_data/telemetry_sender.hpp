#pragma once 
#include <boost/asio.hpp>
#include <thread>
#include <string>
#include <iostream>

class TelemetrySender{
    // using Udp = boost::asio::ip::udp;
    using stream_protocol = boost::asio::local::stream_protocol;
    
    public:
        // TelemetrySender(const std::string& ip, const int port);
        TelemetrySender();
        ~TelemetrySender();

        void send_metric(const std::string& line_protocol_string);

    private:
        inline static constexpr const char* TELEGRAF_SOCKET_PATH = "/tmp/sockets/telegraf.sock";
        //from boost.asio include
        boost::asio::io_context io_context_; 
        // Udp::socket socket_;
        // Udp::endpoint endpoint_;

        stream_protocol::socket socket_;

        boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;

        //from std include
        std::thread background_thread_;
};