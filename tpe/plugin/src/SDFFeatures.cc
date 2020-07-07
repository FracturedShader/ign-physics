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

#include <cmath>

#include <ignition/common/Console.hh>
#include <ignition/math/eigen3/Conversions.hh>

#include <sdf/Box.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Sphere.hh>
#include <sdf/Geometry.hh>

#include "SDFFeatures.hh"

using namespace ignition;
using namespace physics;
using namespace tpeplugin;

/////////////////////////////////////////////////
/// \brief Resolve the pose of an SDF DOM object with respect to its relative_to
/// frame. If that fails, return the raw pose
static Eigen::Isometry3d ResolveSdfPose(const ::sdf::SemanticPose &_semPose)
{
  math::Pose3d pose;
  ::sdf::Errors errors = _semPose.Resolve(pose);
  if (!errors.empty())
  {
    if (!_semPose.RelativeTo().empty())
    {
      ignerr << "There was an error in SemanticPose::Resolve\n";
      for (const auto &err : errors)
      {
        ignerr << err.Message() << std::endl;
      }
      ignerr << "There is no optimal fallback since the relative_to attribute["
             << _semPose.RelativeTo() << "] of the pose is not empty. "
             << "Falling back to using the raw Pose.\n";
    }
    pose = _semPose.RawPose();
  }

  return math::eigen3::convert(pose);
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfWorld(
    const Identity &_engine,
    const ::sdf::World &_sdfWorld)
{
  const Identity worldID = this->ConstructEmptyWorld(_engine, _sdfWorld.Name());

  // construct models
  for (std::size_t i = 0; i < _sdfWorld.ModelCount(); ++i)
  {
    this->ConstructSdfModel(worldID, *_sdfWorld.ModelByIndex(i));
  }

  return worldID;
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfModel(
  const Identity &_worldID,
  const ::sdf::Model &_sdfModel)
{
  // Read sdf params
  const std::string name = _sdfModel.Name();
  const auto pose = _sdfModel.RawPose();

  auto it = this->worlds.find(_worldID.id);
  if (it == this->worlds.end())
  {
    ignwarn << "World [" << _worldID.id << "] is not found." << std::endl;
    return this->GenerateInvalidId();
  }
  auto world = it->second->world;
  if (world == nullptr)
  {
    ignwarn << "World is a nullptr" << std::endl;
    return this->GenerateInvalidId();
  }
  tpelib::Entity &ent = world->AddModel();
  tpelib::Model *model = static_cast<tpelib::Model *>(&ent);
  model->SetName(name);
  model->SetPose(pose);
  const auto modelIdentity = this->AddModel(world->GetId(), *model);

  // construct links
  for (std::size_t i = 0; i < _sdfModel.LinkCount(); ++i)
  {
    this->ConstructSdfLink(modelIdentity, *_sdfModel.LinkByIndex(i));
  }

  return modelIdentity;
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfLink(
    const Identity &_modelID,
    const ::sdf::Link &_sdfLink)
{
  // Read sdf params
  const std::string name = _sdfLink.Name();

  const auto modelInfo = this->ReferenceInterface<ModelInfo>(_modelID);
  auto *model = modelInfo->model;
  if (model == nullptr)
  {
    ignwarn << "Model is a nullptr" << std::endl;
    return this->GenerateInvalidId();
  }

  // compute link pose based on offset
  const Eigen::Isometry3d linkTf =
    math::eigen3::convert(model->GetPose()) * ResolveSdfPose(_sdfLink.SemanticPose());
  math::Pose3d pose = math::eigen3::convert(linkTf);

  // construct link
  tpelib::Entity &ent = model->AddLink();
  tpelib::Link *link = static_cast<tpelib::Link *>(&ent);
  link->SetName(name);
  link->SetPose(pose);
  const auto linkIdentity = this->AddLink(model->GetId(), *link);

  // construct collisions
  for (std::size_t i = 0; i < _sdfLink.CollisionCount(); ++i)
  {
    this->ConstructSdfCollision(linkIdentity, *_sdfLink.CollisionByIndex(i));
  }

  return linkIdentity;
}

/////////////////////////////////////////////////
Identity SDFFeatures::ConstructSdfCollision(
    const Identity &_linkID,
    const ::sdf::Collision &_sdfCollision)
{
  // Read sdf params
  const std::string name = _sdfCollision.Name();
  // const auto pose = _sdfCollision.RawPose();
  const auto geom = _sdfCollision.Geom();

  const auto linkInfo = this->ReferenceInterface<LinkInfo>(_linkID);
  auto *link = linkInfo->link;
  if (link == nullptr)
  {
    ignwarn << "Link is a nullptr" << std::endl;
    return this->GenerateInvalidId();
  }

  // construct collision
  tpelib::Entity &ent = link->AddCollision();
  tpelib::Collision *collision = static_cast<tpelib::Collision *>(&ent);
  collision->SetName(name);
  if (geom->Type() == ::sdf::GeometryType::BOX)
  {
    const auto boxSdf = geom->BoxShape();
    tpelib::BoxShape shape;
    shape.SetSize(boxSdf->Size());
    collision->SetShape(shape);
  }
  else if (geom->Type() == ::sdf::GeometryType::CYLINDER)
  {
    const auto cylinderSdf = geom->CylinderShape();
    tpelib::CylinderShape shape;
    shape.SetRadius(cylinderSdf->Radius());
    shape.SetLength(cylinderSdf->Length());
    collision->SetShape(shape);
  }
  else if (geom->Type() == ::sdf::GeometryType::SPHERE)
  {
    const auto sphereSdf = geom->SphereShape();
    tpelib::SphereShape shape;
    shape.SetRadius(sphereSdf->Radius());
    collision->SetShape(shape);
  }

  // compute collision pose based on offset
  const Eigen::Isometry3d collisionTf =
    math::eigen3::convert(link->GetPose()) * ResolveSdfPose(_sdfCollision.SemanticPose());
  math::Pose3d pose = math::eigen3::convert(collisionTf);

  collision->SetPose(pose);

  // \todo(anyone) add mesh. currently mesh has to be loaded externally
  // and passed in as argument as there is no logic for searching resources
  // in ign-physics
  const auto collisionIdentity = this->AddCollision(link->GetId(), *collision);
  return collisionIdentity;
}
