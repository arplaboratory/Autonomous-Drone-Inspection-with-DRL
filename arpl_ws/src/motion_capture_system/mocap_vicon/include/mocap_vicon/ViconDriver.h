/*
 * Copyright [2015]
 * [Kartik Mohta <kartikmohta@gmail.com>]
 * [Ke Sun <sunke.polyu@gmail.com>]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef VICON_DRIVER_H
#define VICON_DRIVER_H

#include <cmath>
#include <string>
#include <set>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <mocap_base/MoCapDriverBase.h>
#include "ViconDataStreamSDK_CPP/DataStreamClient.h" // From Vicon's SDK


namespace mocap{

class ViconDriver: public MoCapDriverBase {

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*
     * @brief Constructor
     * @param nh Ros node
     */
    ViconDriver(const ros::NodeHandle& n):
      MoCapDriverBase   (n),
      client            (new ViconDataStreamSDK::CPP::Client()),
      max_accel         (10.0),
      frame_interval    (0.01),
      process_noise     (Eigen::Matrix<double, 12, 12>::Zero()),
      measurement_noise (Eigen::Matrix<double, 6, 6>::Zero()) {
      return;
    }

    /*
     * @brief Destructor
     */
    ~ViconDriver() {
      disconnect();
      delete client;
      return;
    }

    /*
     * @brief init Initialize the object
     * @return True if successfully initialized
     */
    bool init();

    /*
     * @brief run Start acquiring data from the server
     */
    void run();

    /*
     * @brief disconnect Disconnect to the server
     * @Note The function is called automatically when the
     *  destructor is called.
     */
    void disconnect();

  private:
    // Disable the copy constructor and assign operator
    ViconDriver(const ViconDriver& );
    ViconDriver& operator=(const ViconDriver& );

    // Handle a frame which contains the info of all subjects
    void handleFrame();

    // Handle a the info of a single subject
    void handleSubject(const int& sub_idx);

    // Portal to communicate with the server
    ViconDataStreamSDK::CPP::Client* client;

    // Max acceleration
    double max_accel;

    // Average time interval between two frames
    double frame_interval;

    // A set to hold the model names
    std::set<std::string> model_set;

    // Convariance matrices for initializing kalman filters
    Eigen::Matrix<double, 12, 12> process_noise;
    Eigen::Matrix<double,  6,  6> measurement_noise;

    // For multi-threading
    boost::shared_mutex mtx;

};
}


#endif
