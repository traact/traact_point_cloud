/*  BSD 3-Clause License
 *
 *  Copyright (c) 2020, FriederPankratz <frieder.pankratz@gmail.com>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#include <rttr/registration>
#include <rttr/type>
#include <traact/facade/Plugin.h>
#include <traact/component/facade/ApplicationAsyncSource.h>
#include <traact/component/facade/ApplicationSyncSink.h>
#include <traact/point_cloud.h>

namespace traact::component::facade {

template <typename HeaderType>
traact::pattern::Pattern::Ptr getAsyncSourcePattern() {
  pattern::spatial::SpatialPattern::Ptr spatial_ptr = std::make_shared<pattern::spatial::SpatialPattern>();
  pattern::Pattern::Ptr pattern_ptr = std::dynamic_pointer_cast<pattern::Pattern>(spatial_ptr);
  ApplicationAsyncSource<spatial::Pose6DHeader>::fillPattern(pattern_ptr);

  spatial_ptr->concurrency = unlimited;
  spatial_ptr->addCoordianteSystem("Origin", false)
      .addCoordianteSystem("Destination", false)
  .addEdge("Origin", "Destination", "output");
  return spatial_ptr;

}

template <typename HeaderType>
traact::pattern::Pattern::Ptr getSyncSinkPattern() {
  pattern::spatial::SpatialPattern::Ptr spatial_ptr = std::make_shared<pattern::spatial::SpatialPattern>();
  pattern::Pattern::Ptr pattern_ptr = std::dynamic_pointer_cast<pattern::Pattern>(spatial_ptr);
  ApplicationSyncSink<spatial::Pose6DHeader>::fillPattern(pattern_ptr);

  spatial_ptr->concurrency = serial;
  spatial_ptr->addCoordianteSystem("Origin", false)
      .addCoordianteSystem("Destination", false)
      .addEdge("Origin", "Destination", "input");
  return spatial_ptr;

}

class SpatialApplicationPlugin : public traact::facade::Plugin {
 public:

  void fillPatternNames(std::vector<std::string> &pattern_names) override {
    pattern_names.emplace_back(getAsyncSourcePattern<traact::spatial::Pose6DHeader>()->name);
    pattern_names.emplace_back(getSyncSinkPattern<traact::spatial::Pose6DHeader>()->name);

  }

  pattern::Pattern::Ptr instantiatePattern(const std::string &pattern_name) override {
    if (pattern_name == getAsyncSourcePattern<traact::spatial::Pose6DHeader>()->name)
      return getAsyncSourcePattern<traact::spatial::Pose6DHeader>();
    if (pattern_name == getSyncSinkPattern<traact::spatial::Pose6DHeader>()->name)
      return getSyncSinkPattern<traact::spatial::Pose6DHeader>();


    return nullptr;
  }
  component::Component::Ptr instantiateComponent(const std::string &pattern_name,
                                                 const std::string &new_component_name) override {

    if (pattern_name == getAsyncSourcePattern<traact::spatial::Pose6DHeader>()->name)
      return std::make_shared<component::facade::ApplicationAsyncSource<spatial::Pose6DHeader> >(new_component_name);
    if (pattern_name == getSyncSinkPattern<traact::spatial::Pose6DHeader>()->name)
      return std::make_shared<component::facade::ApplicationSyncSink<spatial::Pose6DHeader> >(new_component_name);


    return nullptr;
  }
};


}

RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

  using namespace rttr;
  registration::class_<traact::component::facade::SpatialApplicationPlugin>("SpatialApplicationPlugin").constructor<>()
      (
          policy::ctor::as_std_shared_ptr
      );
}
