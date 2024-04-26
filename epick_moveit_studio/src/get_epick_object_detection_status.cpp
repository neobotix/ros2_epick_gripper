// Copyright (c) 2023 PickNik, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <epick_moveit_studio/get_epick_object_detection_status.hpp>

#include <epick_msgs/msg/object_detection_status.hpp>
#include <moveit_studio_behavior_interface/impl/get_message_from_topic_impl.hpp>

namespace
{
inline constexpr auto kDescription = R"(
                <p>Wait for an Epick ObjectDetectionStatus message to be published on a specified ROS topic and copy it to an output data port.</p>
                <p>Given the name of a topic where <code>epick_msgs::msg::ObjectDetectionStatus</code> messages are being published, this Behavior subscribes to that topic and waits until a new message is published to the topic.</p>
                <p>When the Behavior's subscriber receives a new point cloud message, this Behavior copies it to an output data port and then finishes with a SUCCESS status.</p>
                <p>If any of the following failure states occur, this Behavior will exit with a FAILURE status code:</p>
                <ul>
                    <li>No publisher is found on the topic.</li>
                    <li>Creating a subscription on the topic does not succeed.</li>
                    <li>A publisher was found on the topic, but no message is published on the topic before a 1-second timeout duration has passed.</li>
                </ul>
                <p>This Behavior is derived from the GetMessageFromTopic class defined in the <code>moveit_studio_behavior_interface</code> package.</p>
            )";

/** @brief Maximum duration to wait for a message to be published before failing. */
constexpr auto kWaitDuration = std::chrono::seconds{ 1 };
}  // namespace

namespace epick_moveit_studio
{
GetEpickObjectDetectionStatus::GetEpickObjectDetectionStatus(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::GetMessageFromTopicBehaviorBase<epick_msgs::msg::ObjectDetectionStatus>(name, config,
                                                                                                      shared_resources)
{
}

BT::PortsList GetEpickObjectDetectionStatus::providedPorts()
{
  return BT::PortsList({
      BT::InputPort<std::string>(kPortIDTopicName, "{some_topic}",
                                 "The name of the topic which this Behavior will subscribe to and monitor for "
                                 "epick_msgs::msg::ObjectDetectionStatus messages."),
      BT::OutputPort<epick_msgs::msg::ObjectDetectionStatus>(kPortIDMessageOut, "{wrench}",
                                                    "Will contain the output epick_msgs::msg::ObjectDetectionStatus message "
                                                    "after this Behavior has finished successfully."),
  });
}

BT::KeyValueVector GetEpickObjectDetectionStatus::metadata()
{
  return { { "subcategory", "Epick" }, { "description", kDescription } };
}


tl::expected<std::chrono::duration<double>, std::string> GetEpickObjectDetectionStatus::getWaitForMessageTimeout()
{
  return kWaitDuration;
}

}  // namespace epick_moveit_studio

template class moveit_studio::behaviors::GetMessageFromTopicBehaviorBase<epick_msgs::msg::ObjectDetectionStatus>;
