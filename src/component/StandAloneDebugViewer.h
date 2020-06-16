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

#ifndef PCPD_STANDALONEDEBUGVIEWER_H
#define PCPD_STANDALONEDEBUGVIEWER_H


#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>
#include <thread>
#include <map>
#include <mutex>
#include <traact/spatialBody.h>

namespace traact::component {
        enum class ShapeType {
          LINE, CONE, NONE
        };

        class StandAloneDebugViewer{
        public:

            explicit StandAloneDebugViewer(std::string name, bool waitForAllInput = true);

            void start();

            void stop();

            //std::shared_ptr< Component > addPointCloud(std::string cloudName);
            //std::shared_ptr< Component > addPointCloud(std::string cloudName, const Eigen::Matrix4f& cameraOrigin);

            //void newCloudCallback(const ringbuffer::time_tag_type ts, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, size_t index);

          void drawBody(const std::vector<traact::spatial::Body> &body_list, const ShapeType &shapeType = ShapeType::LINE, bool withCOSY = false);


            bool isRunning();

        protected:

            struct MyColor {
                double r;
                double g;
                double b;
            };

            //std::vector<std::shared_ptr<ApplicationPushSinkTraitComponent<datatypes::PositionVertexHeader, pcl::PointCloud<pcl::PointXYZRGB>::Ptr > > > m_cloudReceiver;

            std::vector<MyColor> m_defaultColors;

            std::vector<std::string> m_cloudNames;
            std::map<std::string, Eigen::Matrix4f> m_cameraOrigin;
            std::mutex m_protect_cloud_mutex;
            std::vector<bool> showPointCloud;
            std::vector<bool> pointCloudChanged;
            bool m_waitForAllInput;

            bool useColor = true;
            std::string m_name;




            std::shared_ptr<std::thread> m_thread;
            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> m_cloudVector;
            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> m_cloudVectorBuffer;
            std::vector<traact::spatial::Body> body_list_;
            std::vector<bool> m_newCloud;
            bool m_stopthread{false};
            bool m_isRunning{false};


            void threadFunction();


            void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void);


            void viewerInit (pcl::visualization::PCLVisualizer& viewer);
            void viewerCallback (pcl::visualization::PCLVisualizer& viewer);


          void drawBody(pcl::visualization::PCLVisualizer &viewer,const traact::spatial::Body &body, const ShapeType &shapeType = ShapeType::LINE, bool withCOSY = false);

        };

}

#endif //PCPD_STANDALONEDEBUGVIEWER_H

