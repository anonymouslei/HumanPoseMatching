/*
 * This file is part of ArmarX.
 *
 * ArmarX is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * ArmarX is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @package    HRPExamples::ArmarXObjects::HumanPoseMatching
 * @author     Simon Ottenhaus ( simon dot ottenhaus at kit dot edu )
 * @date       2020
 * @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
 *             GNU General Public License
 */

#pragma once

//#include <SimoxUtility/EigenStdVector.h>
#include <ArmarXCore/core/Component.h>

#include <ArmarXCore/interface/observers/ObserverInterface.h>
//#include <RobotAPI/libraries/core/visualization/DebugDrawerTopic.h>
#include <RobotAPI/libraries/RobotAPIComponentPlugins/ArVizComponentPlugin.h>
#include <ArmarXGui/libraries/ArmarXGuiComponentPlugins/RemoteGuiComponentPlugin.h>
#include <ArmarXGui/libraries/RemoteGui/WidgetBuilder/LayoutWidgets.h>
#include <ArmarXCore/core/services/tasks/RunningTask.h>
#include <MMM/Motion/Legacy/LegacyMotionReaderXML.h>
#include <VirtualRobot/RobotConfig.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/Robot.h>

namespace armarx
{

    struct MotionSlice
    {
        MMM::LegacyMotionPtr motion;
        int beginIndex, endIndex, minIndex, minEndIndex;
        float minStartDifference, minEndDifference, deviationFromDistance, duration, trajectory;
    };

    /**
     * @class HumanPoseMatchingPropertyDefinitions
     * @brief
     */
    class HumanPoseMatchingPropertyDefinitions :
        public armarx::ComponentPropertyDefinitions
    {
    public:
        HumanPoseMatchingPropertyDefinitions(std::string prefix);
    };



    /**
     * @defgroup Component-HumanPoseMatching HumanPoseMatching
     * @ingroup HRPExamples-Components
     * A description of the component HumanPoseMatching.
     *
     * @class HumanPoseMatching
     * @ingroup Component-HumanPoseMatching
     * @brief Brief description of class HumanPoseMatchinstd::vector<MotionSlice> HumanPoseMatching::normalizeMotionSlices(flattenedMotionSlices)g.
     *
     * Detailed description of class HumanPoseMatching.
     */
    class HumanPoseMatching :
        virtual public armarx::plugins::ArVizComponentPluginUser,
        virtual public RemoteGuiComponentPluginUser,
        virtual public armarx::Component
    {
    public:

        /// @see armarx::ManagedIceObject::getDefaultName()
        virtual std::string getDefaultName() const override;


    protected:

        /// @see armarx::ManagedIceObject::onInitComponent()
        virtual void onInitComponent() override;

        /// @see armarx::ManagedIceObjecfloat HumanPoseMatching::getAggregatedDistanceDeviation(MotionSlice slice, std::map<string, float> segmentDistanceMap, VirtualRobot::RobotPtr r)t::onConnectComponent()
        virtual void onConnectComponent() override;

        /// @see armarx::ManagedIceObject::onDisconnectComponent()
        virtual void onDisconnectComponent() override;

        /// @see armarx::ManagedIceObject::onExitComponent()
        virtual void onExitComponent() override;

        /// @see PropertyUser::createPropertyDefinitions()
        virtual armarx::PropertyDefinitionsPtr createPropertyDefinitions() override;


    private:

        // Private methods and member variables go here.

        /// Debug observer. Used to visualize e.g. time series.
        armarx::DebugObserverInterfacePrx debugObserver;
        /// Debug drawer. Used for 3D visualization.
        //armarx::DebugDrawerTopic debugDrawer;
        void visualizeMotion(MMM::LegacyMotionPtr motion);

        void visualizeMotionSlice(MotionSlice slice, float targetHeight, int rank);

        void visualizeBestNWholeMotions(std::multimap<float, MMM::LegacyMotionPtr> motionDifferenceMap, unsigned int n);

        void visualizeNMostRelevantMotionPortions(std::multimap<float, MMM::LegacyMotionPtr> motionDifferenceMap, unsigned int n);

        float differenceToRightHandUp(MMM::LegacyMotionPtr motion);

        void createTab_Widgets();

        void createStartEndTab_Widgets();

        void taskRun();

        void checkCancelButton();

        std::map<std::string, float> getStartAngleValues(RemoteGui::TabProxy& tab);

        std::map<std::string, float> getEndAngleValues(RemoteGui::TabProxy& tab);

        std::map<std::string, Eigen::Vector3f> getStartSegmentValues(RemoteGui::TabProxy& tab);

        std::map<std::string, Eigen::Vector3f> getEndSegmentValues(RemoteGui::TabProxy& tab);

        void waitForInput();

        void visualizeNFirstMotionSlices(std::vector<MotionSlice> motionSlicesByDistance, int n, std::map<std::string, Eigen::Vector3f>& startSegmentValues, std::map<std::string, Eigen::Vector3f>& endSegmentValues);

        std::vector<float> getSegmentDifferences(MMM::LegacyMotionPtr motion, std::map<std::string, Eigen::Vector3f>& startSegmentValues, VirtualRobot::RobotPtr r, VirtualRobot::RobotNodeSetPtr robotNodeSet, float height);

        float getAggregatedDistanceDeviation(MotionSlice& slice, std::map<std::string, float>& segmentDistanceMap, VirtualRobot::RobotPtr r, VirtualRobot::RobotNodeSetPtr robotNodeSet);

        std::vector<float> getDifferences(MMM::LegacyMotionPtr motion, std::map<std::string, Eigen::Vector3f>& segmentValues, std::map<std::string, float>& jointValues, float alpha, VirtualRobot::RobotPtr r, VirtualRobot::RobotNodeSetPtr robotNodeSet, float height);

        std::vector<MotionSlice> getBestMotionSlicesStartValuesOnly(std::vector<MMM::LegacyMotionPtr> motions, std::map<std::string, float> startJointValues, std::map<std::string, Eigen::Vector3f> startSegmentValues, float alpha, float inputThreshold, float height);

        std::vector<MotionSlice> getBestMotionSlicesStartAndEndValues(std::vector<MMM::LegacyMotionPtr> motions, std::map<std::string, float> startJointValues, std::map<std::string, Eigen::Vector3f> startSegmentValues, std::map<std::string, float> endJointValues, std::map<std::string, Eigen::Vector3f> endSegmentValues, float alpha, float inputThreshold, float height);

        std::vector<MotionSlice> getBestMotionSlicesGlobalStartAndEndValues(std::vector<MMM::LegacyMotionPtr>& motions, std::map<std::string, float>& startJointValues, std::map<std::string, Eigen::Vector3f>& startSegmentValues, std::map<std::string, float>& endJointValues, std::map<std::string, Eigen::Vector3f>& endSegmentValues, float alpha, float inputThreshold, float height);

        std::vector<float> getJointDifferences(MMM::LegacyMotionPtr motion, std::map<std::string, float>& startJointValues, std::vector<std::string> jointNames);

        viz::Robot getRobotFromMotionFrame(MotionSlice slice, float targetHeight, int j, Eigen::Matrix3f rotation, Eigen::Vector3f translation);

        std::vector<float> getSegmentGlobalDistanceToPoint(MMM::LegacyMotionPtr motion, Eigen::Vector3f& targetPoint, std::string segmentName, VirtualRobot::RobotPtr r, VirtualRobot::RobotNodeSetPtr robotNodeSet, float targetHeight);

        std::vector<MotionSlice> normalizeMotionSlices(std::vector<MotionSlice>& motionSlices, std::map<std::string, Eigen::Vector3f>& startSegmentValues, std::map<std::string, Eigen::Vector3f>& endSegmentValues);

        void fillDurationForMotionSlice(MotionSlice& motionSlice);

        void fillTrajectoryForMotionSlice(MotionSlice& motionSlice, std::map<std::string, Eigen::Vector3f>& idealdirectionsMap);

        std::map<std::string, Eigen::Vector3f> getSegmentDirectionMap(std::map<std::string, Eigen::Vector3f>& startSegmentValues, std::map<std::string, Eigen::Vector3f>& endSegmentValues);

        Eigen::Vector3f getDirection(Eigen::Vector3f& startPoint, Eigen::Vector3f& endPoint);

        void drawInputVisualization();

        int getRandomNumber(int n);

    private:
        RunningTask<HumanPoseMatching>::pointer_type task;
        //SimplePeriodicTask<HumanPoseMatching>::pointer_type guiTask;
        RemoteGuiInterfacePrx remoteGui;
        int method;
        unsigned int rightShoulderXJointIndex;
        unsigned int rightShoulderYJointIndex;
        unsigned int rightElbowXJointIndex;
        std::vector<MMM::LegacyMotionPtr> motionFiles;
        bool needToReloadFiles;
        //        RemoteGui::detail::VBoxLayoutBuilder vLayout;
        VirtualRobot::RobotPtr robotModel;
        std::vector<std::string> startRowNames = std::vector<std::string>();
        std::vector<std::string> endRowNames = std::vector<std::string>();
        bool previousShowEndInput = false;
        bool previousGlobalCoordinatesInput = false;
        float previousHeight = 1.0f;
        float previousOffsetHeight = 0.0f;
        std::string mainSegment;
        std::map<std::string, std::pair<Eigen::Vector3f, Eigen::Vector3f>> startEndPairs = std::map<std::string, std::pair<Eigen::Vector3f, Eigen::Vector3f>>();
        std::map<std::string, Eigen::Vector3f> startPointsMap = std::map<std::string, Eigen::Vector3f>();
        std::map<std::string, Eigen::Vector3f> endPointsMap = std::map<std::string, Eigen::Vector3f>();
        std::vector<Eigen::Vector3f> startPoints;
        std::vector<Eigen::Vector3f> endPoints;
        std::vector<viz::Color> pairColors;
        std::vector<viz::Color> startColors;
        std::vector<viz::Color> endColors;
    };
}
