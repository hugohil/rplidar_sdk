/*
 *  SLAMTEC LIDAR
 *  Data Printer Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2020 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>

#include <iostream>
#include <fstream>

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"
#ifndef _countof
  #define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
  #include <Windows.h>
  #define delay(x)::Sleep(x)
#else
  #include <unistd.h>
  static inline void delay(sl_word_size_t ms){
    while (ms>=1000){
      usleep(1000*1000);
      ms-=1000;
    };
    if (ms!=0) usleep(ms*1000);
  }
#endif

using namespace sl;

void print_usage (int argc, const char * argv[]) {
  printf("Custom LIDAR data grabber for SLAMTEC LIDAR.\n"
    "Version: %s \n"
    "Usage:\n"
    "%s --port <serial port> [baudrate] # to print\n"
    "%s --port <serial port> [baudrate] | ./other-app # to use datas somewhere else\n"
    "The baudrate is 115200 (for A2) or 256000 (for A3). Default is 115200.\n"
    "SL_LIDAR_SDK_VERSION", argv[0], argv[0]);
}

bool checkSLAMTECLIDARHealth (ILidarDriver * drv) {
  sl_result op_result;
  sl_lidar_response_device_health_t healthinfo;

  op_result = drv->getHealth(healthinfo);
  // the macro IS_OK is the preperred way to judge whether the operation is succeed.
  if (SL_IS_OK(op_result)) {
    printf("SLAMTEC Lidar health status : %d\n", healthinfo.status);
    if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
      fprintf(stderr, "Error, slamtec lidar internal error detected. Please reboot the device to retry.\n");
      // enable the following code if you want slamtec lidar to be reboot by software
      // drv->reset();
      return false;
    } else {
      return true;
    }
  } else {
    fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
    return false;
  }
}

bool ctrl_c_pressed;
void ctrlc (int) {
  ctrl_c_pressed = true;
}

int main(int argc, const char * argv[]) {
  const char * opt1 = NULL;
  const char * opt_port = NULL;
  sl_u32 opt_baudrate = 0;
  sl_result op_result;

  IChannel* _channel;

  printf("LIDAR data printer for SLAMTEC LIDAR.\n"
    "Version: %s\n", "SL_LIDAR_SDK_VERSION");

  if (argc > 1) {
    opt1 = argv[1];
  } else {
    print_usage(argc, argv);
    return -1;
  }

  sl_u32 baudrateArray[2] = { 115200, 256000 };

  const char * default_port = "/dev/ttyUSB0";
  #ifdef _WIN32
    default_port = "\\\\.\\com3";
  #elif __APPLE__
    default_port = "/dev/tty.SLAB_USBtoUART";
  #endif

  if (strcmp(opt1, "--port") == 0) {
    opt_port = (argc > 2) ? argv[2] : default_port;
    opt_baudrate = (argc > 3) ? strtoul(argv[3], NULL, 10) : baudrateArray[0];
  } else {
    print_usage(argc, argv);
    return -1;
  }

  // create the driver instance
	ILidarDriver * drv = *createLidarDriver();

  if (!drv) {
    fprintf(stderr, "insufficent memory, exit\n");
    exit(-2);
  }

  sl_lidar_response_device_info_t devinfo;
  bool connectSuccess = false;

  _channel = (*createSerialPortChannel(opt_port, opt_baudrate));
  if (SL_IS_OK((drv)->connect(_channel))) {
    op_result = drv->getDeviceInfo(devinfo);
    if (SL_IS_OK(op_result)) {
      connectSuccess = true;
    } else {
      delete drv;
      drv = NULL;
    }
  }

  if (!connectSuccess) {
    fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n", opt_port);
    goto on_finished;
  }

  // print out the device serial number, firmware and hardware version number..
  printf("SLAMTEC LIDAR S/N: ");
  for (int pos = 0; pos < 16 ;++pos) {
    printf("%02X", devinfo.serialnum[pos]);
  }

  printf("\n"
    "Firmware Ver: %d.%02d\n"
    "Hardware Rev: %d\n",
    devinfo.firmware_version >> 8,
    devinfo.firmware_version & 0xFF,
    (int) devinfo.hardware_version);

  // check health...
  if (!checkSLAMTECLIDARHealth(drv)) {
    goto on_finished;
  }

  signal(SIGINT, ctrlc);

  drv->setMotorSpeed();
  drv->startScan(0,1);
  printf("starting scan...\n");

  // fetch result and print it out...
  while (1) {
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = _countof(nodes);

    op_result = drv->grabScanDataHq(nodes, count);

    if (SL_IS_OK(op_result)) {
      drv->ascendScanData(nodes, count);
      std::cout << "S;";
      for (int pos = 0; pos < (int)count ; ++pos) {
        std::cout << std::to_string((nodes[pos].angle_z_q14 * 90.f) / 16384.f) << ";";
        std::cout << std::to_string(nodes[pos].dist_mm_q2 / 4.0f) << ";";
      }
      std::cout << "E;" << std::endl;
    }
    if (ctrl_c_pressed) {
      break;
    }
    usleep(16 * 1000);
  }

  printf("\nbye.\n");
  drv->stop();
  delay(200);
  drv->setMotorSpeed(0);

on_finished:
  if(drv) {
    delete drv;
    drv = NULL;
  }
  return 0;
}

