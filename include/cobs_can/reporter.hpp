#pragma once

#include <rclcpp/rclcpp.hpp>

#include <CRSLibtmp/reporter.hpp>

namespace cobs_can_bridge
{
	struct ErrorReporter final
	{
		rclcpp::Logger logger;

		ErrorReporter(rclcpp::Logger&& logger) noexcept:
			logger{std::move(logger)}
		{}

		ErrorReporter(ErrorReporter&& other) noexcept:
			logger{std::move(other.logger)}
		{}

		void operator()(const char *const message) noexcept
		{
			RCLCPP_ERROR(logger, message);
		}
	};

	struct InfoReporter final
	{
		rclcpp::Logger logger;

		InfoReporter(rclcpp::Logger&& logger) noexcept:
			logger{std::move(logger)}
		{}

		InfoReporter(ErrorReporter&& other) noexcept:
			logger{std::move(other.logger)}
		{}

		void operator()(const char *const message) noexcept
		{
			RCLCPP_INFO(logger, message);
		}
	};
}