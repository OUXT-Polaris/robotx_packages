//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef NMEA_GPS_PLUGIN_H_INCLUDED
#define NMEA_GPS_PLUGIN_H_INCLUDED

#include <gazebo/common/Plugin.hh>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <hector_gazebo_plugins/sensor_model.h>
#include <hector_gazebo_plugins/update_timer.h>

#include <dynamic_reconfigure/server.h>
#include <hector_gazebo_plugins/GNSSConfig.h>
#include <nmea_msgs/Sentence.h>

namespace gazebo
{

  class nmea_gps_sensor_plugin : public ModelPlugin
  {
  public:
    nmea_gps_sensor_plugin();
    virtual ~nmea_gps_sensor_plugin();

  protected:
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void Reset();
    virtual void Update();

    typedef hector_gazebo_plugins::GNSSConfig GNSSConfig;
    void dynamicReconfigureCallback(GNSSConfig &config, uint32_t level);

  private:
    //members for building nmea sentence
    nmea_msgs::Sentence build_GPGGA_sentence();
    nmea_msgs::Sentence build_GPRMC_sentence();
    std::string get_nmea_checksum(std::string sentence);
    /// \brief The parent World
    physics::WorldPtr world;

    /// \brief The link referred to by this plugin
    physics::LinkPtr link;

    ros::NodeHandle* node_handle_;
    ros::Publisher nmea_sentence_publisher_;
    //ros::Publisher fix_publisher_;
    //ros::Publisher velocity_publisher_;

    sensor_msgs::NavSatFix fix_;
    geometry_msgs::Vector3Stamped velocity_;

    std::string namespace_;
    std::string link_name_;
    std::string frame_id_;
    std::string nmea_sentence_topic_;

    double reference_latitude_;
    double reference_longitude_;
    double reference_heading_;
    double reference_altitude_;

    double radius_north_;
    double radius_east_;

    SensorModel3 position_error_model_;
    SensorModel3 velocity_error_model_;

    UpdateTimer updateTimer;
    event::ConnectionPtr updateConnection;
  };

} // namespace gazebo

#endif // NMEA_GPS_PLUGIN_H_INCLUDED
