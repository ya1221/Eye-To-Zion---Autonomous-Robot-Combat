#include "robot_description/telemetry_sender.hpp"
#include <filesystem>

// TelemetrySender::TelemetrySender(const std::string& ip, const int port) : socket_(io_context_), work_guard_(boost::asio::make_work_guard(io_context_)){
//     try {
//         socket_.open(Udp::v4());        
//         Udp::resolver resolver(io_context_);
//         Udp::resolver::results_type endpoints = resolver.resolve(Udp::v4(), ip, std::to_string(port));
//         endpoint_ = *endpoints.begin();

//         background_thread_ = std::thread([this](){
//             io_context_.run();
//         });
//     } catch (const std::exception& e) {
//         // RCLCPP_INFO(this->get_logger(), "[TelegrafSender] Initialization Error:", e.what())
//     }
// }

TelemetrySender::TelemetrySender() : socket_(io_context_), work_guard_(boost::asio::make_work_guard(io_context_)){
    if(!std::filesystem::exists(TelemetrySender::TELEGRAF_SOCKET_PATH)){
        std::cerr << "[TelemetrySender] Telegraf socket not found at " << TelemetrySender::TELEGRAF_SOCKET_PATH << "\n";
        return;
    }
    stream_protocol::endpoint endpoint(TelemetrySender::TELEGRAF_SOCKET_PATH);

    try {
        socket_.connect(endpoint);
        std::cerr << "[TelemetrySender] Connected to Telegraf socket at " << TelemetrySender::TELEGRAF_SOCKET_PATH << "\n";
    } catch (std::exception& e) {
        std::cerr << "[TelemetrySender] Failed to connect to Telegraf socket: " << e.what() << "\n";
        return;
    }

    // Start the background thread to run the io_context event loop.
    // Without this, boost::asio::post() calls queue work but nothing ever executes it.
    background_thread_ = std::thread([this](){
        io_context_.run();
    });
}

TelemetrySender::~TelemetrySender(){
    work_guard_.reset();
    if(background_thread_.joinable()){
        background_thread_.join();
    }
    if(socket_.is_open()) {
        socket_.close();
    }

    std::cout << "Telemetry sender object is out of life\n";
}

// void TelemetrySender::send_metric(const std::string& line_protocol) {
//     boost::asio::post(io_context_, [this, line_protocol]() {
//         try {
//             socket_.send_to(boost::asio::buffer(line_protocol), endpoint_);
//         } catch (const std::exception& e) {
//         }
//     });
// }

void TelemetrySender::send_metric(const std::string& line_protocol) {
    boost::asio::post(io_context_, [this, line_protocol](){
        boost::asio::async_write(socket_, boost::asio::buffer(line_protocol),
            [line_protocol](const boost::system::error_code& ec, std::size_t bytes_sent){
                if (ec) {
                    std::cerr << "Error: " << ec.message() << " (Code: " << ec.value() << " Category: " << ec.category().name() << ")\n";
                } else {
                    // std::cerr << "[TelemetrySender] Sent " << bytes_sent << " bytes: " << line_protocol << "\n";
                }
        });
    });
}
