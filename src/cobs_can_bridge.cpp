/**
 * @file cobs_can_bridge.cpp
 * @author Stew (you@domain.com)
 * @brief cobs_serialを使ったcobs_can_bridge.
 * @version 0.1
 * @date 2023-02-15
 * 
 * @todo ライフサイクルをちゃんとやる
 * @copyright Copyright (c) 2023
 * 
 */

#include <cstring>
#include <optional>
#include <utility>
#include <chrono>
#include <future>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <cobs_can_frame/msg/frame.hpp>

#include <cobs_can/cobs_serial.hpp>
#include <cobs_can/reporter.hpp>

namespace cobs_can_bridge
{
	using namespace CRSLib::IntegerTypes;

	class CobsCanBridge final : public rclcpp::Node
	{
		ErrorReporter error_reporter;
		std::optional<CRSLib::Usb::CobsSerial<ErrorReporter, InfoReporter>> cobs_serial;

		rclcpp::Publisher<cobs_can_frame::msg::Frame>::SharedPtr can_rx_pub;
		rclcpp::Subscription<cobs_can_frame::msg::Frame>::SharedPtr can_tx_sub;

	public:
		CobsCanBridge(const rclcpp::NodeOptions & options):
			rclcpp::Node("cobs_can_bridge", options),
			error_reporter{this->get_logger()},
			cobs_serial{std::in_place, "/dev/usbcan", 115'200, ErrorReporter(this->get_logger()), InfoReporter(this->get_logger())}
		{
			using namespace std::chrono_literals;

			rclcpp::on_shutdown([this](){on_shutdown();});

			rclcpp::WallRate(1s).sleep();

			volatile int loop_well_formalize = 0;
			while(handshake())
			{
				// このsleepは内部でvolatile領域にアクセスするとかioの関数を呼んでいるんだろうか...一応volatile入れておく。
				rclcpp::WallRate(1s).sleep();
				loop_well_formalize = 0;
			}

			start_listen_can();

			/// @todo canXXXへの送信。現在はとりあえずcan_rx, can_txへ送受信する
			can_rx_pub = this->create_publisher<cobs_can_frame::msg::Frame>("can_rx", 1000);
			can_tx_sub = this->create_subscription<cobs_can_frame::msg::Frame>("can_tx", 1000, std::bind(&CobsCanBridge::can_tx_callback, this, std::placeholders::_1));
		}

	private:
		void on_shutdown() noexcept
		{
			cobs_serial = std::nullopt;
		}

		void can_tx_callback(const cobs_can_frame::msg::Frame::SharedPtr frame)
		{
			/// @todo idをトピックから取得
			cobs_serial->async_write(convert(frame, 100));
		}

		static std::vector<u8> convert(const cobs_can_frame::msg::Frame::SharedPtr& frame, const u32 id)
		{
			std::vector<u8> buffer(5 + frame->dlc);

			// id (11 or 29 bit)
			for(int i = 0; i < 4; ++i)
			{
				buffer[i] = id >> i & 0xFF;
			}

			// dlc
			buffer[4] = frame->dlc;

			std::memcpy(buffer.data() + 5, frame->data.data(), frame->dlc);

			return buffer;
		}

		static std::tuple<cobs_can_frame::msg::Frame, u32> deconvert(const std::vector<u8>& buffer)
		{
			u32 id = 0;
			for(int i = 0; i < 4; ++i)
			{
				id |= buffer[i] << i;
			}

			cobs_can_frame::msg::Frame frame{};

			frame.dlc = buffer[4];

			std::memcpy(frame.data.data(), buffer.data() + 5, frame.dlc);

			return {frame, id};
		}

		void can_rx_handler(const std::vector<u8>& buffer) noexcept
		{
			try
			{
				auto [frame, id] = deconvert(buffer);

				try
				{
					can_rx_pub->publish(std::move(frame));
				}
				catch(...)
				{
					error_reporter("cobs_can_bridge: can_rx_handler: something wrong happened while publishing.");
				}
			}
			catch(...)
			{
				error_reporter("cobs_can_bridge: can_rx_handler: something wrong happened while deconverting.");
			}

			cobs_serial->async_read([this](auto&& buf) noexcept {can_rx_handler(buf);});
		}

		void start_listen_can()
		{
			cobs_serial->async_read([this](auto&& buf) noexcept {can_rx_handler(buf);});
		}

		bool handshake()
		{
			std::vector<u8> hello_usbcan{};
			for(const auto c : "helloUSBCAN")
			{
				hello_usbcan.push_back(c);
			}

			std::promise<bool> result_prm{};
			std::future<bool> result_ftr = result_prm.get_future();

			cobs_serial->async_write(hello_usbcan);
			cobs_serial->async_read
			(
				[result_prm = std::move(result_prm)](const std::vector<u8>& data) noexcept
				{
					bool result{true};
					
					constexpr auto hello_cobscan = std::to_array("helloCobsCAN");

					for(int i = 0; i < std::min(data.size(), hello_cobscan.size()); ++i)
					{
						if(data[i] != hello_cobscan[i])
						{
							result = false;
							break;
						}
					}

					result_prm.set_value(result);
				}
			);

			return result_ftr.get();
		}
	};
}