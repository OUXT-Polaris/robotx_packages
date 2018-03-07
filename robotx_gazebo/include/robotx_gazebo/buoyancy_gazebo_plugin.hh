/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef GAZEBO_PLUGINS_BUOYANCYPLUGIN_HH_
#define GAZEBO_PLUGINS_BUOYANCYPLUGIN_HH_

#include <map>
//#include <ignition/math/Vector3.hh>

#include <gazebo/common/common.hh>
#include "gazebo/common/Event.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"

namespace gazebo
{
  /// \brief A class for storing the volume properties of a link.
  class VolumeProperties
  {
    /// \brief Default constructor.
  public: VolumeProperties() : area(0), height(0) {}
    
    /// \brief Center of volume in the link frame.
    //public: ignition::math::Vector3d cov;
    //public: math::Vector3 cov;
  public: math::Vector3 cov;
    /// \brief Volume of this link.
    //public: double volume;
    /// \brief Horizontal area of this link
  public: double area;
    /// \brief Vertical height for this link
  public: double height;
  };

  /// \brief A plugin that simulates buoyancy of an object immersed in fluid.
  /// All SDF parameters are optional.
  /// <fluid_density> sets the density of the fluid that surrounds the buoyant
  /// object.
  /// <link> elements describe the volume properties of individual links in the
  /// model. For example:
  /// <link name="body">
  ///   <center_of_volume>1 2 3</center_of_volume>
  ///   <volume>50</volume>
  /// </link>
  /// <center_of_volume> A point representing the volumetric center of the
  /// link in the link frame. This is where the buoyancy force will be applied.
  /// <volume> The volume of the link in kg/m^3.
  /// If center of volume and volume are not specified, the plugin will attempt
  /// to compute these properties from the link collision shapes. This
  /// computation will not be accurate if the object is not composed of simple
  /// collision shapes.
  //  class GAZEBO_VISIBLE BuoyancyPlugin : public ModelPlugin
  class BuoyancyPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: BuoyancyPlugin();

    /// \brief Read the model SDF to compute volume and center of volume for
    /// each link, and store those properties in volPropsMap.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation inherited
    public: virtual void Init();

    /// \brief Callback for World Update events.
    protected: virtual void OnUpdate();

    /// \brief Connection to World Update events.
    protected: event::ConnectionPtr updateConnection;

    /// \brief Pointer to model containing the plugin.
    protected: physics::ModelPtr model;

    /// \brief Pointer to the plugin SDF.
    protected: sdf::ElementPtr sdf;

    /// \brief The density of the fluid in which the object is submerged in
    /// kg/m^3. Defaults to 1000, the fluid density of water.
    protected: double fluidDensity;

    // \brief The height of the fluid/air interface [m].  Defaults to 0
  protected: double fluidLevel;

    // \brief Quadratic drag generally applied to Z velocity
  protected: double fluidDrag;
    

    /// \brief Map of <link ID, point> pairs mapping link IDs to the CoV (center
    /// of volume) and volume of the link.
    protected: std::map<int, VolumeProperties> volPropsMap;
    
    // \brief Vector of links in the model for which we will apply buoyancy
    // forces.
  protected: physics::Link_V buoyLinks;
  };
}

#endif
