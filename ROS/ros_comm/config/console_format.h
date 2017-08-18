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


#endif
