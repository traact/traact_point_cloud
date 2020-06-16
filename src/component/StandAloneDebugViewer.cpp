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
#include "StandAloneDebugViewer.h"
#include <pcl/visualization/cloud_viewer.h>

#include <random>
#include <spdlog/spdlog.h>
namespace traact::component {


        StandAloneDebugViewer::StandAloneDebugViewer(std::string name, bool waitForAllInput) : m_name(name) {

            m_defaultColors.resize(5);

            m_defaultColors[0].r = 255.0 / 255;
            m_defaultColors[0].g = 225.0 / 255;
            m_defaultColors[0].b = 25.0 / 255;

            m_defaultColors[1].r = 0.0 / 255;
            m_defaultColors[1].g = 130.0 / 255;
            m_defaultColors[1].b = 200.0 / 255;

            m_defaultColors[2].r = 245.0 / 255;
            m_defaultColors[2].g = 130.0 / 255;
            m_defaultColors[2].b = 48.0 / 255;

            m_defaultColors[3].r = 250.0 / 255;
            m_defaultColors[3].g = 190.0 / 255;
            m_defaultColors[3].b = 190.0 / 255;

            m_defaultColors[4].r = 230.0 / 255;
            m_defaultColors[4].g = 190.0 / 255;
            m_defaultColors[4].b = 255.0 / 255;


        }

        void StandAloneDebugViewer::start() {

            if (m_isRunning)
                return;

            spdlog::info("StandAloneDebugViewer[{0}] starting thread", m_name);

            // is this the gui main thread ?
            // some platforms don't like running their gui mainloop in an application thread
            // can we refactor the interface so that the main application calls this method instead of while-looping forever?
            m_thread.reset(new std::thread(&StandAloneDebugViewer::threadFunction, this));
            m_isRunning = true;
        }

        void StandAloneDebugViewer::stop() {

            m_stopthread = true;
            m_thread->join();
        }

        bool StandAloneDebugViewer::isRunning() {
            return m_isRunning;

        }

        void StandAloneDebugViewer::viewerInit(pcl::visualization::PCLVisualizer &viewer) {
            viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey

            int yPox = 100;
            int yInrecment = -20;


            for (int index = 0; index < m_cloudNames.size(); ++index) {
                viewer.addText("_" + m_cloudNames[index] + "_", 10, yPox, m_defaultColors[index].r,
                               m_defaultColors[index].g,
                               m_defaultColors[index].b);
                yPox += yInrecment;


                if (m_cameraOrigin.count(m_cloudNames[index]) > 0) {


                    Eigen::Affine3f pose;
                    Eigen::Matrix4f camPose = m_cameraOrigin[m_cloudNames[index]];
                    pose.matrix() = camPose;

                    pcl::PointXYZ p;
                    p.x = camPose(0, 3);
                    p.y = camPose(1, 3);
                    p.z = camPose(2, 3);


                    viewer.addCoordinateSystem(1.0, pose, "originCoordinateSystem" + std::to_string(index), 0);
                    viewer.addSphere(p, 0.2, m_defaultColors[index].r, m_defaultColors[index].g,
                                     m_defaultColors[index].b,
                                     "origin" + std::to_string(index));

                }


            }
        }

        void
        StandAloneDebugViewer::viewerCallback(pcl::visualization::PCLVisualizer &viewer) {


            bool somethingChanged = false;
            bool somethingNew = false;
            bool allNewInput = true;

          {
            std::unique_lock<std::mutex> lk(m_protect_cloud_mutex);
            if(!body_list_.empty()) {
              viewer.removeAllShapes();
              viewer.removeAllCoordinateSystems();
              viewerInit(viewer);
              for(const auto& body : body_list_) {
                drawBody(viewer, body);
              }
            }

          }


            for (int index = 0; index < m_cloudNames.size(); ++index) {
                somethingChanged = somethingChanged || pointCloudChanged[index];
                allNewInput = allNewInput && m_newCloud[index];
                somethingNew = somethingNew || m_newCloud[index];
            }

            if (m_waitForAllInput) {
                somethingChanged = somethingChanged || allNewInput;
            } else {
                somethingChanged = somethingChanged || somethingNew;
            }


            if ((m_waitForAllInput && allNewInput) || (!m_waitForAllInput && somethingNew)) {
                std::unique_lock<std::mutex> lk(m_protect_cloud_mutex);

                m_cloudVector = m_cloudVectorBuffer;

            }

            if (!somethingChanged)
                return;


            viewer.removeAllPointClouds();


            for (int index = 0; index < m_cloudNames.size(); ++index) {

                if (showPointCloud[index]) {

                    //viewer.updatePointCloud(m_cloudVector[index], m_cloudNames[index]);

                    if (useColor) {
                        viewer.addPointCloud(m_cloudVector[index], m_cloudNames[index]);

                    } else {
                        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> colorHandler(m_cloudVector[index], m_defaultColors[index].r*255,m_defaultColors[index].g*255,m_defaultColors[index].b*255);
                        viewer.addPointCloud(m_cloudVector[index], colorHandler, m_cloudNames[index]);
                    }
                }

                pointCloudChanged[index] = false;
                m_newCloud[index] = false;


            }
        }

        void StandAloneDebugViewer::keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
                                                          void *viewer_void) {
            pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);

            spdlog::trace("keyboardEventOccurred {0} {1}", event.getKeySym(), event.isCtrlPressed());

            std::string key = event.getKeySym();

            int index = -1;
            if (event.getKeySym() == "1") {
                index = 0;
            } else if (event.getKeySym() == "2") {
                index = 1;
            } else if (event.getKeySym() == "3") {
                index = 2;
            } else if (event.getKeySym() == "4") {
                index = 3;
            } else if (event.getKeySym() == "c" && event.keyDown()) {
                useColor = !useColor;
                spdlog::trace("useColor {0} ", useColor);

            } else if (event.getKeySym() == "v" && event.keyDown()) {

            }

            if (index < 0)
                return;

            if (event.isCtrlPressed()) { // point cloud 2


            } else {

                showPointCloud[index] = !showPointCloud[index];

                pointCloudChanged[index] = true;


            }


        }


      void StandAloneDebugViewer::drawBody(const std::vector<traact::spatial::Body> &body_list,
                                         const ShapeType &shapeType,
                                         bool withCOSY) {
        std::unique_lock<std::mutex> lk(m_protect_cloud_mutex);
        body_list_ = body_list;

      }

      void StandAloneDebugViewer::drawBody(pcl::visualization::PCLVisualizer &viewer,const traact::spatial::Body &poseList, const ShapeType &shapeType, bool withCOSY) {

          int shapeID = static_cast<int>(shapeType);
        for (const auto &connection : traact::spatial::BodyUtils::JointToParent) {
          if (connection.first == connection.second)
            continue;

          const auto &childJoint = poseList.bodyJoints.at(connection.first);
          const auto &parentJoint = poseList.bodyJoints.at(connection.second);

          auto parentTrans = parentJoint.pose.translation();
          auto childTrans = childJoint.pose.translation();

          pcl::PointXYZ parentPoint(parentTrans.x(), parentTrans.y(), parentTrans.z());
          pcl::PointXYZ childPoint(childTrans.x(), childTrans.y(), childTrans.z());

          switch (shapeType) {
            case ShapeType::NONE:break;
            case ShapeType::LINE:
              // Draw a line between parent and child joint
              viewer.addLine(parentPoint, childPoint, 0.9, 0.9, 0.9, "link-" + std::to_string(shapeID));
              break;
            case ShapeType::CONE:
              // Draw a cone pointing from parent to child joint
              pcl::ModelCoefficients coeffs;
              for (int i = 0; i < 3; i++)
                coeffs.values.push_back(childPoint.data[i]);
              for (int i = 0; i < 3; i++)
                coeffs.values.push_back(parentPoint.data[i] - childPoint.data[i]);
              coeffs.values.push_back(4);
              viewer.addCone(coeffs, "cone-" + std::to_string(shapeID));
              break;
          }
          if (withCOSY) {
            const Eigen::Affine3f a = parentJoint.pose.cast<float>();

            viewer.addCoordinateSystem(0.1, a, "reference-" + std::to_string(shapeID));
          }
          shapeID++;
        }
      }

        /*
        std::shared_ptr<Component> StandAloneDebugViewer::addPointCloud(std::string cloudName) {
            m_cloudNames.emplace_back(cloudName);
            showPointCloud.emplace_back(true);
            pointCloudChanged.emplace_back(false);
            m_newCloud.emplace_back(false);


            if (m_cloudNames.size() > m_defaultColors.size()) {
                std::random_device rd;
                std::mt19937 e2(rd());
                std::uniform_real_distribution<> dist(0, 1);

                MyColor newColor;
                newColor.r = dist(e2);
                newColor.g = dist(e2);
                newColor.b = dist(e2);

                m_defaultColors.emplace_back(newColor);

            }

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr newcloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
                    new pcl::PointCloud<pcl::PointXYZRGB>());


            m_cloudVector.emplace_back(newcloud);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr newcloudBuffer = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
                    new pcl::PointCloud<pcl::PointXYZRGB>());

            m_cloudVectorBuffer.emplace_back(newcloudBuffer);

            auto newSink = std::make_shared<ApplicationPushSinkTraitComponent<datatypes::PositionVertexHeader, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> >(cloudName + "_input");
            m_cloudReceiver.emplace_back(newSink);
            size_t cloudIndex = m_cloudVector.size() - 1;

            auto callback = std::bind(&StandAloneDebugViewer::newCloudCallback, this, std::placeholders::_1,
                                      std::placeholders::_2, cloudIndex);
            newSink->setCallback(callback);

            return std::dynamic_pointer_cast<dataflow::Component>(newSink);
        }

        /*
        std::shared_ptr<Component>
        StandAloneDebugViewer::addPointCloud(std::string cloudName, const Eigen::Matrix4f &cameraOrigin) {

            m_cameraOrigin[cloudName] = cameraOrigin;
            return addPointCloud(cloudName);

        }

        void StandAloneDebugViewer::newCloudCallback(const ringbuffer::time_tag_type ts,
                                                     const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, size_t index) {
            std::unique_lock<dataflow::mutex_t> lk(m_protect_cloud_mutex);
            m_cloudVectorBuffer[index] = cloud;
            m_newCloud[index] = true;

        }*/


        void StandAloneDebugViewer::threadFunction() {

            pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

            m_isRunning = true;

            viewerInit(viewer);

            viewer.registerKeyboardCallback(&StandAloneDebugViewer::keyboardEventOccurred, *this);


            while (!m_stopthread && !viewer.wasStopped()) {

                viewerCallback(viewer);

                viewer.spinOnce(10);
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            m_isRunning = false;

        };



}

