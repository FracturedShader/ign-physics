/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef IGNITION_PHYSICS_TPE_LIB_SRC_COLLISION_HH_
#define IGNITION_PHYSICS_TPE_LIB_SRC_COLLISION_HH_

#include <ignition/math/AxisAlignedBox.hh>

#include "ignition/physics/tpelib/Export.hh"

#include "Entity.hh"
#include "Shape.hh"

namespace ignition {
namespace physics {
namespace tpelib {

// Forward declartion
class CollisionPrivate;

/// \brief Collision class
class IGNITION_PHYSICS_TPELIB_VISIBLE Collision : public Entity
{
  /// \brief Constructor
  public: Collision();

  /// \brief Constructor
  /// \param[in] _id Collision id
  public: explicit Collision(std::size_t _id);

  /// \brief Copy Constructor
  /// \param[in] _other The other collision to copy from
  public: Collision(const Collision &_other);

  /// \brief Destructor
  public: ~Collision();

  /// \brief Assignment operator
  /// \param[in] _other collision
  /// \return collision
  public: Collision &operator=(const Collision &_other);

  /// \brief Set Shape
  /// \param[in] _shape shape
  public: void SetShape(const Shape &_shape);

  /// \brief Get Shape
  /// \return shape of collision
  public: Shape *GetShape() const;

  // Documentation inherited
  public: math::AxisAlignedBox GetBoundingBox(bool _force) override;

  /// \brief Update the pose of the entity
  /// \param[in] _timeStep current world timestep
  /// \param[in] _linearVelocity linear velocity
  /// \param[in] _angularVelocity angular velocity
  public: virtual void UpdatePose(
    const double _timeStep,
    const math::Vector3d _linearVelocity,
    const math::Vector3d _angularVelocity);

  /// \brief Private data pointer class
  private: CollisionPrivate *dataPtr = nullptr;
};

}
}
}

#endif
