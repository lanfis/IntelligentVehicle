#pragma once
#ifndef _ROS_CONSOLE_FORMAT_H_
#define _ROS_CONSOLE_FORMAT_H_

#include <string>
#include <ros/console.h>

#define EXTENDED_OUTPUT 1

#if EXTENDED_OUTPUT

#define NO_COLOR     "\033[0m"
#define BLACK        "\033[30m"
#define RED          "\033[31m"
#define GREEN        "\033[32m"
#define YELLOW       "\033[33m"
#define BLUE         "\033[34m"
#define MAGENTA      "\033[35m"
#define CYAN         "\033[36m"
#define LIGHTGRAY    "\033[37m"



//const std::string getFunctionName(const std::string &name);
//__PRETTY_FUNCTION__
#define OUT_AUX(FUNC_COLOR, MSG_COLOR, STREAM, NODE, MSG) STREAM(FUNC_COLOR "[" << NODE << "] " MSG_COLOR << MSG << NO_COLOR)

#define OUT_DEBUG(node, msg) OUT_AUX(BLUE, NO_COLOR, ROS_DEBUG_STREAM, node, msg)
#define OUT_INFO(node, msg) OUT_AUX(GREEN, NO_COLOR, ROS_INFO_STREAM, node, msg)
#define OUT_WARN(node, msg) OUT_AUX(YELLOW, YELLOW, ROS_WARN_STREAM, node, msg)
#define OUT_ERROR(node, msg) OUT_AUX(RED, RED, ROS_ERROR_STREAM, node, msg)

#else

#define NO_COLOR     ""
#define BLACK        ""
#define RED          ""
#define GREEN        ""
#define YELLOW       ""
#define BLUE         ""
#define MAGENTA      ""
#define CYAN         ""
#define LIGHTGRAY    ""

#define OUT_DEBUG(msg) ROS_DEBUG_STREAM(msg)
#define OUT_INFO(msg) ROS_INFO_STREAM(msg)
#define OUT_WARN(msg) ROS_WARN_STREAM(msg)
#define OUT_ERROR(msg) ROS_WARN_STREAM(msg)

#endif

#endif