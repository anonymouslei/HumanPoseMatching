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

#include "HumanPoseMatching.h"

#include <ArmarXCore/core/time/CycleUtil.h>
#include <ArmarXCore/core/time/TimeUtil.h>
#include <ArmarXGui/libraries/RemoteGui/WidgetBuilder.h>
#include <MMM/Motion/Legacy/LegacyMotionReaderXML.h>
#include <RobotAPI/components/ArViz/Client/Client.h>
#include <RobotAPI/interface/ArViz.h>
#include <MMMSimoxTools/MMMSimoxTools.h>
#include <MMM/Model/ModelReaderXML.h>
#include <iostream>
#include <filesystem>
#include <random>
#include <Eigen/Dense>
#include <math.h>
#include <omp.h>
#include <MMM/Model/ModelProcessorWinter.h>

namespace armarx
{
    HumanPoseMatchingPropertyDefinitions::HumanPoseMatchingPropertyDefinitions(std::string prefix) :
        armarx::ComponentPropertyDefinitions(prefix)
    {
        //defineRequiredProperty<std::string>("PropertyName", "Description");
        //defineOptionalProperty<std::string>("PropertyName", "DefaultValue", "Description");

        defineOptionalProperty<std::string>("DebugObserverName", "DebugObserver", "Name of the topic the DebugObserver listens on");

        //defineOptionalProperty<std::string>("MyProxyName", "MyProxy", "Name of the proxy to be used");
    }


    std::string HumanPoseMatching::getDefaultName() const
    {
        return "HumanPoseMatching";
    }


    void HumanPoseMatching::onInitComponent()
    {
        // Register offered topices and used proxies here.
        offeringTopicFromProperty("DebugObserverName");
        // debugDrawer.offeringTopic(*this);  // Calls this->offeringTopic().

        // Using a proxy will cause the component to wait until the proxy is available.
        // usingProxyFromProperty("MyProxyName");
    }


    void HumanPoseMatching::onConnectComponent()
    {
        // Get topics and proxies here. Pass the *InterfacePrx type as template argument.
        debugObserver = getTopicFromProperty<DebugObserverInterfacePrx>("DebugObserverName");
        // debugDrawer.getTopic(*this);  // Calls this->getTopic().

        pairColors = std::vector<viz::Color>();
        startColors = std::vector<viz::Color>();
        endColors = std::vector<viz::Color>();

        for (unsigned int i = 0; i < 20; ++i)
        {
            int r = 10 * i;
            int g = 120 - 2 * i;
            int b = 255 - 5 * i;
            pairColors.push_back(viz::Color::fromRGBA(r, g, b, 160));
        }

        for (unsigned int i = 0; i < 20; ++i)
        {
            int r = 255 - 10 * i;
            int g = 10 * i;
            int b = 255 - 10 * i;
            startColors.push_back(viz::Color::fromRGBA(r, g, b, 160));
        }

        for (unsigned int i = 0; i < 20; ++i)
        {
            int r = 120 + 2 * i;
            int g = 10 * i;
            int b = 255 - 10 * i;
            endColors.push_back(viz::Color::fromRGBA(r, g, b, 160));
        }

        createStartEndTab_Widgets();
        motionFiles = std::vector<MMM::LegacyMotionPtr>();

        task = new RunningTask<HumanPoseMatching>(this, &HumanPoseMatching::taskRun);
        task->start();

        while (true)
        {
            if (task->isFinished() || task->isStopped() || !task->isRunning() || !task->isAlive())
            {
                ARMARX_IMPORTANT << "The task is dead, long live the task";
                task = nullptr;
                task = new RunningTask<HumanPoseMatching>(this, &HumanPoseMatching::taskRun);
                task->start();
            }
        }



    }


    void HumanPoseMatching::onDisconnectComponent()
    {
        task->stop();
        task = nullptr;
    }


    void HumanPoseMatching::onExitComponent()
    {

    }

    bool isSegment(std::string rowName)
    {
        return rowName.compare("BPSegment_joint") == 0
               || rowName.compare("BTSegment_joint") == 0
               || rowName.compare("BLNsegment_joint") == 0
               || rowName.compare("BUNsegment_joint") == 0
               || rowName.compare("HeadSegment_joint") == 0
               || rowName.compare("LAsegment_joint") == 0
               || rowName.compare("LEsegment_joint") == 0
               || rowName.compare("LHsegment_joint") == 0
               || rowName.compare("LKsegment_joint") == 0
               || rowName.compare("LSCsegment_joint") == 0
               || rowName.compare("LSsegment_joint") == 0
               || rowName.compare("LWsegment_joint") == 0
               || rowName.compare("LeftEyeSegmentX_joint") == 0
               || rowName.compare("LeftEyeSegmentY_joint") == 0
               || rowName.compare("LeftFootHeight_joint") == 0
               || rowName.compare("LeftHandSegment_joint") == 0
               || rowName.compare("MidHeadSegment_joint") == 0
               || rowName.compare("RAsegment_joint") == 0
               || rowName.compare("REsegment_joint") == 0
               || rowName.compare("RHsegment_joint") == 0
               || rowName.compare("RKsegment_joint") == 0
               || rowName.compare("RSCsegment_joint") == 0
               || rowName.compare("RSsegment_joint") == 0
               || rowName.compare("RWsegment_joint") == 0
               || rowName.compare("RightEyeSegmentX_joint") == 0
               || rowName.compare("RightEyeSegmentY_joint") == 0
               || rowName.compare("RightFootHeight_joint") == 0
               || rowName.compare("RightFootLength_joint") == 0
               || rowName.compare("RightHandSegment_joint") == 0
               || rowName.compare("collarSegment_joint") == 0;
    }

    std::map<std::string, float> HumanPoseMatching::getStartAngleValues(RemoteGui::TabProxy& tab)
    {
        std::map<std::string, float> startAngleValues = std::map<std::string, float>();

        for (std::string startRowName : startRowNames)
        {
            if (!isSegment(startRowName))
            {
                float angle = tab.getValue<float>(startRowName + "startJointAngleBox").get();
                startAngleValues[startRowName] = angle;
            }
        }

        return startAngleValues;
    }

    std::map<std::string, float> HumanPoseMatching::getEndAngleValues(RemoteGui::TabProxy& tab)
    {
        std::map<std::string, float> endAngleValues = std::map<std::string, float>();

        for (std::string endRowName : endRowNames)
        {
            if (!isSegment(endRowName))
            {
                float angle = tab.getValue<float>(endRowName + "endJointAngleBox").get();
                endAngleValues[endRowName] = angle;
            }
        }

        return endAngleValues;
    }

    std::map<std::string, Eigen::Vector3f> HumanPoseMatching::getStartSegmentValues(RemoteGui::TabProxy& tab1)
    {
        std::map<std::string, Eigen::Vector3f> startSegmentValues = std::map<std::string, Eigen::Vector3f>();

        RemoteGui::TabProxy tab = RemoteGui::TabProxy(getRemoteGui(), getName());

        for (std::string startRowName : startRowNames)
        {
            if (isSegment(startRowName))
            {
                float posX = HUGE_VALF;
                float posY = HUGE_VALF;
                float posZ = HUGE_VALF;

                if (tab.getValue<bool>(startRowName + "startPositionXCheck").get())
                {
                    posX = tab.getValue<float>(startRowName + "startPositionXBox").get();
                }
                if (tab.getValue<bool>(startRowName + "startPositionYCheck").get())
                {
                    posY = tab.getValue<float>(startRowName + "startPositionYBox").get();
                }
                if (tab.getValue<bool>(startRowName + "startPositionZCheck").get())
                {
                    posZ = tab.getValue<float>(startRowName + "startPositionZBox").get();
                }
                startSegmentValues[startRowName] = Eigen::Vector3f(posX, posY, posZ);
            }
        }

        return startSegmentValues;
    }

    std::map<std::string, Eigen::Vector3f> HumanPoseMatching::getEndSegmentValues(RemoteGui::TabProxy& tab)
    {
        std::map<std::string, Eigen::Vector3f> endSegmentValues = std::map<std::string, Eigen::Vector3f>();

        for (std::string endRowName : endRowNames)
        {
            if (isSegment(endRowName))
            {
                float posX = HUGE_VALF;
                float posY = HUGE_VALF;
                float posZ = HUGE_VALF;

                if (tab.getValue<bool>(endRowName + "endPositionXCheck").get())
                {
                    posX = tab.getValue<float>(endRowName + "endPositionXBox").get();
                }
                if (tab.getValue<bool>(endRowName + "endPositionYCheck").get())
                {
                    posY = tab.getValue<float>(endRowName + "endPositionYBox").get();
                }
                if (tab.getValue<bool>(endRowName + "endPositionZCheck").get())
                {
                    posZ = tab.getValue<float>(endRowName + "endPositionZBox").get();
                }

                endSegmentValues[endRowName] = Eigen::Vector3f(posX, posY, posZ);
            }
        }

        return endSegmentValues;
    }

    void HumanPoseMatching::createStartEndTab_Widgets()
    {
        using namespace RemoteGui;

        float coordinateLimit = 1000000000.f;

        auto vLayout = makeVBoxLayout("test");
        auto metaLabelsVLayout = makeVBoxLayout("metaLabelsV");
        auto metaInputVLayout = makeVBoxLayout("metaInputV");
        auto metaHLayout = makeHBoxLayout("metaH");

        WidgetPtr fileNumberBoxLabel = makeTextLabel("Number of files");
        WidgetPtr fileNumberBox = makeIntSpinBox("fileNumberBox").min(1).max(3911).value(100);

        WidgetPtr spinBoxLabel = makeTextLabel("Number of best matches");
        WidgetPtr intSpinBox = makeIntSpinBox("intSpinBox").min(1).max(50).value(5);

        WidgetPtr thresholdBoxLabel = makeTextLabel("Threshold (mm)");
        WidgetPtr thresholdSpinBox = makeFloatSpinBox("thresholdBox").min(1).max(999999).value(100);

        WidgetPtr heightBoxLabel = makeTextLabel("Height (m)");
        WidgetPtr heightSpinBox = makeFloatSpinBox("heightBox").min(1).max(5).value(previousHeight);

        WidgetPtr offsetHeightBoxLabel = makeTextLabel("offset height (m)");
        WidgetPtr offsetHeightSpinBox = makeFloatSpinBox("offsetHeightBox").min(-10).max(10).value(previousOffsetHeight);

        metaLabelsVLayout.children({fileNumberBoxLabel, spinBoxLabel, thresholdBoxLabel, heightBoxLabel, offsetHeightBoxLabel});
        metaInputVLayout.children({fileNumberBox, intSpinBox, thresholdSpinBox, heightSpinBox, offsetHeightSpinBox});
        metaHLayout.children({metaLabelsVLayout, metaInputVLayout});
        vLayout.addChild(metaHLayout);

        WidgetPtr globalCoordinatesCheckBox = makeCheckBox("Global Coordinates").value(previousGlobalCoordinatesInput);
        vLayout.addChild(globalCoordinatesCheckBox);

        WidgetPtr alphaBoxLabel = makeTextLabel("α: ");
        WidgetPtr alphaBox = makeFloatSpinBox("alphaBox").min(0).max(1).value(0.5);
        WidgetPtr alphaExplainBoxLabel = makeTextLabel("0.5 -> joint angles and position equally weighted, 1.0 -> only joint angles weighted");
        WidgetPtr alphaBoxLine = makeHBoxLayout("alphaBoxLine").children({alphaBoxLabel, alphaBox, alphaExplainBoxLabel});
        vLayout.addChild(alphaBoxLine);

        WidgetPtr startPointBoxLabel = makeTextLabel("Start Point:");
        vLayout.addChild(startPointBoxLabel);

        MMM::ModelReaderXMLPtr reader(new MMM::ModelReaderXML());
        robotModel = MMM::SimoxTools::buildModel(reader->loadModel(std::string(PROJECT_SOURCE_DIR) + "/data/MMM/mmm.xml"));
        std::vector <std::string> robotNodeNames = robotModel->getRobotNodeNames();

        WidgetPtr startRobotNodeBoxLabel = makeTextLabel("Select RobotNode").value("Select:");
        WidgetPtr startRobotNodeCombobox = makeComboBox("startRobotNodeCombobox").options(robotNodeNames);
        WidgetPtr startAddButton = makeButton("startaddButton").label("Add");
        WidgetPtr startSelectRobotNodeBoxLine = makeHBoxLayout("startSelectRobotNodeBoxLine").children({startRobotNodeBoxLabel, startRobotNodeCombobox, startAddButton});
        vLayout.addChild(startSelectRobotNodeBoxLine);

        auto startNodesVLabelsLayout = makeVBoxLayout("startNodesVLabels");
        auto startNodesVInputsLayout = makeVBoxLayout("startNodesVInputs");
        auto startNodesVButtonsLayout = makeVBoxLayout("startNodesVButtons");
        auto startNodesHLayout = makeHBoxLayout("startNodesH");

        for (std::string startRowName : startRowNames)
        {
            auto inputHLayout = makeHBoxLayout(startRowName + "startInputH");

            if (isSegment(startRowName))
            {
                WidgetPtr startPositionBoxLabel = makeTextLabel(startRowName + " Start Position: ").value(startRowName);
                WidgetPtr startPositionXCheckBox = makeCheckBox(startRowName + "startPositionXCheck").label("").value(true);
                WidgetPtr startPositionXBoxLabel = makeTextLabel(startRowName + "startPositionXBoxLabel").value("x");
                WidgetPtr startPositionXBox = makeFloatSpinBox(startRowName + "startPositionXBox").min(-coordinateLimit).max(coordinateLimit).value(0);
                WidgetPtr startPositionYCheckBox = makeCheckBox(startRowName + "startPositionYCheck").label("").value(true);
                WidgetPtr startPositionYBoxLabel = makeTextLabel(startRowName + "startPositionYBoxLabel").value("y");
                WidgetPtr startPositionYBox = makeFloatSpinBox(startRowName + "startPositionYBox").min(-coordinateLimit).max(coordinateLimit).value(0);
                WidgetPtr startPositionZCheckBox = makeCheckBox(startRowName + "startPositionZCheck").label("").value(true);
                WidgetPtr startPositionZBoxLabel = makeTextLabel(startRowName + "startPositionZBoxLabel").value("z");
                WidgetPtr startPositionZBox = makeFloatSpinBox(startRowName + "startPositionZBox").min(-coordinateLimit).max(coordinateLimit).value(0);

                inputHLayout.children({startPositionXCheckBox, startPositionXBoxLabel, startPositionXBox, startPositionYCheckBox, startPositionYBoxLabel, startPositionYBox, startPositionZCheckBox, startPositionZBoxLabel, startPositionZBox});

                WidgetPtr startDeletPositionButton = makeButton(startRowName + "startDeleteButton").label("Delete");

                startNodesVLabelsLayout.addChild(startPositionBoxLabel);
                startNodesVInputsLayout.addChild(inputHLayout);
                startNodesVButtonsLayout.addChild(startDeletPositionButton);
            }
            else
            {
                WidgetPtr startJointAngleBoxLabel = makeTextLabel(startRowName + " Start Joint Angle: ").value(startRowName);
                WidgetPtr startJointAngleBox = makeFloatSpinBox(startRowName + "startJointAngleBox").min(-3.14).max(3.14).value(0);
                WidgetPtr startJointAngleBoxRadiansLabel = makeTextLabel(startRowName + "startJointAngleBoxRadiansLabel").value("Radians");

                inputHLayout.children({startJointAngleBox, startJointAngleBoxRadiansLabel});

                WidgetPtr startDeletJointAngleButton = makeButton(startRowName + "startDeleteButton").label("Delete");

                startNodesVLabelsLayout.addChild(startJointAngleBoxLabel);
                startNodesVInputsLayout.addChild(inputHLayout);
                startNodesVButtonsLayout.addChild(startDeletJointAngleButton);
            }
        }

        startNodesHLayout.children({startNodesVLabelsLayout, startNodesVInputsLayout, startNodesVButtonsLayout});
        vLayout.addChild(startNodesHLayout);




        WidgetPtr endPointCheckBox = makeCheckBox("End Point").value(previousShowEndInput);
        vLayout.addChild(endPointCheckBox);

        auto endInputVBox = makeVBoxLayout("endInput");
        WidgetPtr endRobotNodeBoxLabel = makeTextLabel("Select RobotNode End").value("Select:");
        WidgetPtr endRobotNodeCombobox = makeComboBox("endRobotNodeCombobox").options(robotNodeNames);
        WidgetPtr endAddButton = makeButton("endaddButton").label("Add");
        WidgetPtr endSelectRobotNodeBoxLine = makeHBoxLayout().children({endRobotNodeBoxLabel, endRobotNodeCombobox, endAddButton});
        endInputVBox.addChild(endSelectRobotNodeBoxLine);

        auto endNodesVLabelsLayout = makeVBoxLayout("endNodesVLabels");
        auto endNodesVInputsLayout = makeVBoxLayout("endNodesVInputs");
        auto endNodesVButtonsLayout = makeVBoxLayout("endNodesVButtons");
        auto endNodesHLayout = makeHBoxLayout("endNodesH");

        for (std::string endRowName : endRowNames)
        {
            auto inputHLayout = makeHBoxLayout(endRowName + "endInputH");

            if (isSegment(endRowName))
            {
                WidgetPtr endPositionBoxLabel = makeTextLabel(endRowName + " end Position: ").value(endRowName);
                WidgetPtr endPositionXCheckBox = makeCheckBox(endRowName + "endPositionXCheck").label("").value(true);
                WidgetPtr endPositionXBoxLabel = makeTextLabel(endRowName + "endPositionXBoxLabel").value("x");
                WidgetPtr endPositionXBox = makeFloatSpinBox(endRowName + "endPositionXBox").min(-coordinateLimit).max(coordinateLimit).value(0);
                WidgetPtr endPositionYCheckBox = makeCheckBox(endRowName + "endPositionYCheck").label("").value(true);
                WidgetPtr endPositionYBoxLabel = makeTextLabel(endRowName + "endPositionYBoxLabel").value("y");
                WidgetPtr endPositionYBox = makeFloatSpinBox(endRowName + "endPositionYBox").min(-coordinateLimit).max(coordinateLimit).value(0);
                WidgetPtr endPositionZCheckBox = makeCheckBox(endRowName + "endPositionZCheck").label("").value(true);
                WidgetPtr endPositionZBoxLabel = makeTextLabel(endRowName + "endPositionZBoxLabel").value("z");
                WidgetPtr endPositionZBox = makeFloatSpinBox(endRowName + "endPositionZBox").min(-coordinateLimit).max(coordinateLimit).value(0);

                inputHLayout.children({endPositionXCheckBox, endPositionXBoxLabel, endPositionXBox, endPositionYCheckBox, endPositionYBoxLabel, endPositionYBox, endPositionZCheckBox, endPositionZBoxLabel, endPositionZBox});

                WidgetPtr endDeletPositionButton = makeButton(endRowName + "endDeleteButton").label("Delete");

                endNodesVLabelsLayout.addChild(endPositionBoxLabel);
                endNodesVInputsLayout.addChild(inputHLayout);
                endNodesVButtonsLayout.addChild(endDeletPositionButton);
            }
            else
            {
                WidgetPtr endJointAngleBoxLabel = makeTextLabel(endRowName + " end Joint Angle: ").value(endRowName);
                WidgetPtr endJointAngleBox = makeFloatSpinBox(endRowName + "endJointAngleBox").min(-3.14).max(3.14).value(0);
                WidgetPtr endJointAngleBoxRadiansLabel = makeTextLabel(endRowName + "endJointAngleBoxRadiansLabel").value("Radians");

                inputHLayout.children({endJointAngleBox, endJointAngleBoxRadiansLabel});

                WidgetPtr endDeletJointAngleButton = makeButton(endRowName + "endDeleteButton").label("Delete");

                endNodesVLabelsLayout.addChild(endJointAngleBoxLabel);
                endNodesVInputsLayout.addChild(inputHLayout);
                endNodesVButtonsLayout.addChild(endDeletJointAngleButton);
            }
        }

        endNodesHLayout.children({endNodesVLabelsLayout, endNodesVInputsLayout, endNodesVButtonsLayout});
        endInputVBox.hidden(!previousShowEndInput);

        endInputVBox.addChild(endNodesHLayout);

        vLayout.addChild(endInputVBox);

        WidgetPtr submitButton = makeButton("submitButton").label("Get best matches");

        WidgetPtr cancelButton = makeButton("cancelButton").label("Cancel");
        WidgetPtr buttonLine = makeHBoxLayout().children({submitButton, cancelButton});
        vLayout.addChild(buttonLine);

        WidgetPtr infoLineEdit = makeLineEdit("info");
        vLayout.addChild(infoLineEdit);

        auto gridLayout = makeSimpleGridLayout("grid").cols(5);

        gridLayout.addChild(vLayout);

        gridLayout.addChild(new HSpacer);

        getRemoteGui()->createTab(getName(), gridLayout);

    }


    armarx::PropertyDefinitionsPtr HumanPoseMatching::createPropertyDefinitions()
    {
        return armarx::PropertyDefinitionsPtr(new HumanPoseMatchingPropertyDefinitions(
                getConfigIdentifier()));
    }

    bool hasEnding(std::string const& fullString, std::string const& ending)
    {
        if (fullString.length() >= ending.length())
        {
            return (0 == fullString.compare(fullString.length() - ending.length(), ending.length(), ending));
        }
        else
        {
            return false;
        }
    }

    void HumanPoseMatching::visualizeMotion(MMM::LegacyMotionPtr motion)
    {
        viz::Layer testLayer = arviz.layer("Test");

        MMM::MotionFramePtr firstMotionFrame = motion->getMotionFrame(0);
        MMM::MotionFramePtr secondMotionFrame = motion->getMotionFrame(1);
        float timestepDiff = (secondMotionFrame->timestep - firstMotionFrame->timestep) * 1000;
        CycleUtil c(0.5 * timestepDiff);

        // get names of the involved joints
        std::vector<std::string> jointNames = motion->getJointNames();

        MMM::ModelPtr mmmModel = motion->getModel();
        //mmmModel->setHeight(1.8);
        float height = mmmModel->getHeight();

        for (unsigned int j = 0 ; j < motion->getNumFrames(); j++)
        {
            testLayer.clear();
            MMM::MotionFramePtr motionFrame = motion->getMotionFrame(j);
            Eigen::Matrix4f rootPose = motionFrame->getRootPose();
            rootPose(0, 3) = rootPose(0, 3) / height;
            rootPose(1, 3) = rootPose(1, 3) / height;
            rootPose(2, 3) = rootPose(2, 3) / height;

            viz::Robot robot = viz::Robot("mmm").pose(rootPose).file("HRPGroup3", "MMM/mmm.xml");

            Eigen::Matrix4f identityMatrix;

            identityMatrix << 1.0f, 0.0f, 0.0f, 0.0f,
                           0.0f, 1.0f, 0.0f, 0.0f,
                           0.0f, 0.0f, 1.0f, 0.0f,
                           0.0f, 0.0f, 0.0f, 1.0f;

            robot.useFullModel();

            for (unsigned int i = 0; i < jointNames.size(); ++i)
            {
                robot.joint(jointNames[i], motionFrame->joint(i));
            }


            testLayer.add(robot);

            arviz.commit({testLayer});

            c.waitForCycleDuration();

            checkCancelButton();

        }
    }

    viz::Robot HumanPoseMatching::getRobotFromMotionFrame(MotionSlice slice, float targetHeight, int j, Eigen::Matrix3f rotation, Eigen::Vector3f translation)
    {
        checkCancelButton();

        int colorBuffer = 5;

        MMM::MotionFramePtr motionFrame = slice.motion->getMotionFrame(j);

        std::vector<std::string> jointNames = slice.motion->getJointNames();

        float actualHeight = slice.motion->getModel()->getHeight();

        Eigen::Matrix4f rootPose = motionFrame->getRootPose();
        rootPose(0, 3) = rootPose(0, 3) / actualHeight * targetHeight;
        rootPose(1, 3) = rootPose(1, 3) / actualHeight * targetHeight;

        rootPose(2, 3) = rootPose(2, 3) / actualHeight;
        rootPose(2, 3) = rootPose(2, 3) * targetHeight + previousOffsetHeight * 1000;

        Eigen::Matrix3f rootRotation = rootPose.block<3, 3>(0, 0);
        Eigen::Vector3f rootTranslation = rootPose.block<3, 1>(0, 3);

        viz::Robot robot = viz::Robot("...");

        if (previousGlobalCoordinatesInput)
        {
            robot = viz::Robot("mmm").position(rotation * rootTranslation + translation).orientation(Eigen::Quaternionf(rotation * rootRotation)).file("HRPGroup3", "MMM/mmm.xml");
        }
        else
        {
            robot = viz::Robot("mmm").pose(rootPose).file("HRPGroup3", "MMM/mmm.xml");
        }


        robot = robot.scale(targetHeight);
        robot.useFullModel();

        if (slice.minEndIndex != -1)
        {
            if (j < slice.minIndex + colorBuffer && j > slice.minIndex - colorBuffer)
            {
                robot.overrideColor(viz::Color::fromRGBA(63, 74, 191, 120));
            }
            else if (j < slice.minEndIndex - colorBuffer && j >= slice.minIndex + colorBuffer)
            {
                robot.overrideColor(viz::Color::fromRGBA(72, 191, 63, 120));
            }
            else if (j < slice.minEndIndex + colorBuffer && j >= slice.minEndIndex - colorBuffer)
            {
                robot.overrideColor(viz::Color::fromRGBA(191, 63, 63, 120));
            }

        }
        else
        {
            if (j < slice.minIndex + colorBuffer && j > slice.minIndex - colorBuffer)
            {
                robot.overrideColor(viz::Color::fromRGBA(63, 74, 191, 120));
            }
        }


        for (unsigned int i = 0; i < jointNames.size(); ++i)
        {
            robot.joint(jointNames[i], motionFrame->joint(i));
        }
        return robot;
    }

    int HumanPoseMatching::getRandomNumber(int n)
    {
        std::random_device rd;
        std::mt19937 mt(rd());
        std::uniform_real_distribution<double> dist(0, n);
        return std::floor(dist(mt));
    }

    void HumanPoseMatching::visualizeMotionSlice(MotionSlice slice, float targetHeight, int rank)
    {
        checkCancelButton();
        ARMARX_IMPORTANT << "file: " << slice.motion->getMotionFileName();
        ARMARX_IMPORTANT << "min start difference: " << slice.minStartDifference << ", min end difference: " << slice.minEndDifference;
        ARMARX_IMPORTANT << "rank: " << rank << ", slice trajectory: " << slice.trajectory << ", slice duration: " << slice.duration;
        ARMARX_IMPORTANT << "deviation from distance: " << slice.deviationFromDistance;

        viz::Layer testLayer = arviz.layer("Test");

        MMM::LegacyMotionPtr motion = slice.motion;
        MMM::MotionFramePtr firstMotionFrame = motion->getMotionFrame(0);
        MMM::MotionFramePtr secondMotionFrame = motion->getMotionFrame(1);
        float timestepDiff = (secondMotionFrame->timestep - firstMotionFrame->timestep) * 1000;
        CycleUtil c(timestepDiff);

        int bufferOfFrames = (unsigned int)(500 / timestepDiff);

        Eigen::Matrix3f rotation = Eigen::Matrix3f::Zero();
        Eigen::Vector3f translation = Eigen::Vector3f::Zero();

        ARMARX_IMPORTANT << "main segment is " << mainSegment;

        if (mainSegment.size() > 0)
        {
            Eigen::Vector3f idealGlobalStartToEndDirection = getDirection(startPoints[0], endPoints[0]);

            idealGlobalStartToEndDirection[2] = 0.0f;
            idealGlobalStartToEndDirection.normalize();

            MMM::ModelPtr model = motion->getModel(false);

            MMM::ModelProcessorWinter processor = MMM::ModelProcessorWinter();
            processor.setup(previousHeight, previousHeight * model->getMass());
            model = processor.convertModel(model);

            VirtualRobot::RobotPtr r = MMM::SimoxTools::buildModel(model, false);
            VirtualRobot::RobotNodeSetPtr robotNodeSet = VirtualRobot::RobotNodeSet::createRobotNodeSet(r, "ExampleRobotNodeSet", motion->getJointNames(), "", "", true);

            MMM::MotionFramePtr minStartMotionFrame = motion->getMotionFrame(slice.minIndex);

            float actualHeight = motion->getModel()->getHeight();

            Eigen::Matrix4f rootPose = minStartMotionFrame->getRootPose();
            rootPose(0, 3) = rootPose(0, 3) / actualHeight * targetHeight;
            rootPose(1, 3) = rootPose(1, 3) / actualHeight * targetHeight;
            rootPose(2, 3) = rootPose(2, 3) / actualHeight * targetHeight;
            r->setGlobalPose(rootPose);
            robotNodeSet->setJointValues(minStartMotionFrame->joint);
            // ...
            VirtualRobot::RobotNodePtr segmentNode = r->getRobotNode(mainSegment);

            Eigen::Vector3f startSegmentPosition = segmentNode->getGlobalPosition();

            MMM::MotionFramePtr minEndMotionFrame = motion->getMotionFrame(slice.minEndIndex);

            rootPose = minEndMotionFrame->getRootPose();
            rootPose(0, 3) = rootPose(0, 3) / actualHeight * targetHeight;
            rootPose(1, 3) = rootPose(1, 3) / actualHeight * targetHeight;
            rootPose(2, 3) = rootPose(2, 3) / actualHeight * targetHeight;
            r->setGlobalPose(rootPose);
            robotNodeSet->setJointValues(minEndMotionFrame->joint);
            // ...
            segmentNode = r->getRobotNode(mainSegment);

            Eigen::Vector3f endSegmentPosition = segmentNode->getGlobalPosition();

            Eigen::Vector3f actualGlobalStartToEndDirection = getDirection(startSegmentPosition, endSegmentPosition);

            actualGlobalStartToEndDirection[2] = 0.0f;
            actualGlobalStartToEndDirection.normalize();

            Eigen::Vector3f v = idealGlobalStartToEndDirection.cross(actualGlobalStartToEndDirection);

            float dot = actualGlobalStartToEndDirection[0] * idealGlobalStartToEndDirection[0] + actualGlobalStartToEndDirection[1] * idealGlobalStartToEndDirection[1];
            float det = actualGlobalStartToEndDirection[0] * idealGlobalStartToEndDirection[1] - idealGlobalStartToEndDirection[0] * actualGlobalStartToEndDirection[1];

            float angle = std::atan2(det, dot);

            if (v == Eigen::Vector3f::Zero() || !previousGlobalCoordinatesInput)
            {
                rotation = Eigen::MatrixXf::Identity(3, 3);
            }
            else
            {
                rotation << std::cos(angle), -std::sin(angle), 0,
                         std::sin(angle),  std::cos(angle), 0,
                         0,                0, 1;
            }

            Eigen::Vector3f rotatedStartSegmentPosition = rotation * startSegmentPosition;


            if (previousGlobalCoordinatesInput)
            {
                translation[0] = startPoints[0][0] - rotatedStartSegmentPosition[0];
                translation[1] = startPoints[0][1] - rotatedStartSegmentPosition[1];
            }

        }

        for (int j = std::max((slice.beginIndex - bufferOfFrames), 0);
             j <= slice.endIndex + bufferOfFrames && (unsigned int)j < motion->getMotionFrames().size(); j++)
        {
            testLayer.clear();

            viz::Robot robot = getRobotFromMotionFrame(slice, targetHeight, j, rotation, translation);

            testLayer.add(robot);


            Eigen::Matrix3f textRotation;

            textRotation << 1.0f, 0, 0,
                         0, std::cos(M_PI / 2), -std::sin(M_PI / 2),
                         0, std::sin(M_PI / 2), std::cos(M_PI / 2);

            Eigen::Quaternionf testRotationQuat(textRotation);

            viz::Box referenceBox = viz::Box("referenceBox")
                                    .position(Eigen::Vector3f(0, 4000, 500))
                                    .color(viz::Color::fromRGBA(132, 140, 114))
                                    .size(Eigen::Vector3f(1000.0f, 1000.0f, 1000.0f));

            viz::Robot floor = viz::Robot("floor").file("HRPGroup3", "MMM/floor.xml").position(Eigen::Vector3f::Zero());

            viz::Robot banana = viz::Robot("banana").file("HRPGroup3", "MMM/banana1.xml").position(Eigen::Vector3f(0.0f, 0.0f, 3000.0f)).scale(100.0f);
            banana.useFullModel();

            floor.useFullModel();

            if (previousGlobalCoordinatesInput)
            {
                std::map<std::string, std::pair<Eigen::Vector3f, Eigen::Vector3f>>::iterator it = startEndPairs.begin();

                for (unsigned int i = 0; i < startPoints.size(); ++i)
                {
                    Eigen::Vector3f startTextPosition = startPoints[i];
                    startTextPosition[2] += 100.0f;

                    viz::Text startText = viz::Text("startText" + i).text((*it).first + " start").position(startTextPosition).color(viz::Color::fromRGBA(0, 0, 0)).scale(2.0f).orientation(testRotationQuat);

                    viz::Sphere startSphere = viz::Sphere("startSphere" + i)
                                              .position(startPoints[i])
                                              .color(pairColors[i])
                                              .radius(50.0f);

                    Eigen::Vector3f endTextPosition = endPoints[i];
                    endTextPosition[2] += 100.0f;
                    viz::Text endText = viz::Text("endText" + i).text((*it).first + " end").position(endTextPosition).color(viz::Color::fromRGBA(0, 0, 0)).scale(2.0f).orientation(testRotationQuat);

                    viz::Box endBox = viz::Box("endBox" + i)
                                      .position(endPoints[i])
                                      .color(pairColors[i])
                                      .size(Eigen::Vector3f(100.0f, 100.0f, 100.0f));

                    testLayer.add(startSphere);
                    testLayer.add(startText);
                    testLayer.add(endBox);
                    testLayer.add(endText);

                    ++it;
                }
            }




            testLayer.add(floor);
            testLayer.add(banana);
            testLayer.add(referenceBox);


            arviz.commit({testLayer});

            if (j == slice.minIndex || j == slice.minEndIndex)
            {
                sleep(1);
            }

            c.waitForCycleDuration();
        }
    }

    float getDistance(const Eigen::Vector3f& nodePosition, const Eigen::Vector3f& targetPoint)
    {
        float distance = 0.0f;

        if (targetPoint[0] != HUGE_VALF)
        {
            distance += std::pow(nodePosition[0] - targetPoint[0], 2);
        }
        if (targetPoint[1] != HUGE_VALF)
        {
            distance += std::pow(nodePosition[1] - targetPoint[1], 2);
        }
        if (targetPoint[2] != HUGE_VALF)
        {
            distance += std::pow(nodePosition[2] - targetPoint[2], 2);
        }

        return std::sqrt(distance);
    }

    std::vector<float> getSegmentLocalDistanceToPoint(MMM::LegacyMotionPtr motion, Eigen::Vector3f& targetPoint, std::string segmentName, VirtualRobot::RobotPtr r, VirtualRobot::RobotNodeSetPtr robotNodeSet, float targetHeight)
    {
        std::vector<float> distances = std::vector<float> ();

        float actualHeight = motion->getModel()->getHeight();

        std::vector<MMM::MotionFramePtr> motionFrames = motion->getMotionFrames();

        distances.resize(motionFrames.size());

        for (unsigned int i = 0; i < motionFrames.size(); ++i)
        {
            Eigen::Matrix4f rootPose = motionFrames[i]->getRootPose();
            rootPose(0, 3) = rootPose(0, 3) / actualHeight * targetHeight;
            rootPose(1, 3) = rootPose(1, 3) / actualHeight * targetHeight;
            rootPose(2, 3) = rootPose(2, 3) / actualHeight * targetHeight;
            r->setGlobalPose(rootPose);
            robotNodeSet->setJointValues(motionFrames[i]->joint);
            // ...
            VirtualRobot::RobotNodePtr segmentNode = r->getRobotNode(segmentName);

            Eigen::Vector3f segmentNodePosition = segmentNode->getPositionInRootFrame();

            distances[i] = getDistance(segmentNodePosition, targetPoint);
        }
        return distances;
    }

    std::vector<float> HumanPoseMatching::getSegmentGlobalDistanceToPoint(MMM::LegacyMotionPtr motion, Eigen::Vector3f& targetPoint, std::string segmentName, VirtualRobot::RobotPtr r, VirtualRobot::RobotNodeSetPtr robotNodeSet, float targetHeight)
    {
        std::vector<float> distances = std::vector<float> ();

        float actualHeight = motion->getModel()->getHeight();

        std::vector<MMM::MotionFramePtr> motionFrames = motion->getMotionFrames();

        distances.resize(motionFrames.size());

        bool angleThresholdExceeded = false;
        Eigen::Vector3f globalZUnitVector = Eigen::Vector3f(0.0f, 0.0f, 1.0f);

        for (unsigned int i = 0; i < motionFrames.size(); ++i)
        {
            Eigen::Matrix4f rootPose = motionFrames[i]->getRootPose();
            rootPose(0, 3) = rootPose(0, 3) / actualHeight * targetHeight;
            rootPose(1, 3) = rootPose(1, 3) / actualHeight * targetHeight;
            rootPose(2, 3) = rootPose(2, 3) / actualHeight * targetHeight + previousOffsetHeight * 1000;
            r->setGlobalPose(rootPose);
            robotNodeSet->setJointValues(motionFrames[i]->joint);
            // ...
            VirtualRobot::RobotNodePtr segmentNode = r->getRobotNode(segmentName);

            Eigen::Vector3f localZUnitVector = motionFrames[i]->getRootPose().block<3, 3>(0, 0) * globalZUnitVector;

            float angle = std::acos(localZUnitVector.dot(globalZUnitVector));

            Eigen::Vector3f segmentNodePosition = segmentNode->getGlobalPosition();

            distances[i] = getDistance(segmentNodePosition, Eigen::Vector3f(HUGE_VALF, HUGE_VALF, targetPoint[2]));

            if (angle > M_PI / 4)
            {
                angleThresholdExceeded = true;
            }
        }

        if (angleThresholdExceeded)
        {
            std::transform(distances.begin(), distances.end(), distances.begin(),
                           std::bind(std::plus<float>(), std::placeholders::_1, 9999999.0f));
        }

        return distances;
    }

    std::vector<float> getJointDifferenceToAngle(MMM::LegacyMotionPtr motion, float targetAngle, int jointIndex)
    {
        std::vector<float> jointDifferences = std::vector<float> ();

        for (MMM::MotionFramePtr motionFrame : motion->getMotionFrames())
        {
            float actualJointAngle = motionFrame->joint(jointIndex);
            float angleDiff = std::abs(actualJointAngle - targetAngle);
            jointDifferences.push_back(angleDiff);
        }

        return jointDifferences;
    }

    void HumanPoseMatching::checkCancelButton()
    {
        RemoteGui::TabProxy tab = RemoteGui::TabProxy(getRemoteGui(), getName());
        tab.receiveUpdates();
        if (tab.getButton("cancelButton").clicked())
        {
            ARMARX_IMPORTANT << "task should be stopped now";
            task->stop();
        }
    }

    void HumanPoseMatching::visualizeNFirstMotionSlices(std::vector<MotionSlice> motionSlicesByDistance, int n, std::map<std::string, Eigen::Vector3f>& startSegmentValues, std::map<std::string, Eigen::Vector3f>& endSegmentValues)
    {
        RemoteGui::TabProxy tab = RemoteGui::TabProxy(getRemoteGui(), getName());
        Eigen::Vector3f startPoint = Eigen::Vector3f(0, 0, 0);

        mainSegment = "";

        startPoints = std::vector<Eigen::Vector3f>();
        endPoints = std::vector<Eigen::Vector3f>();

        for (std::map<std::string, Eigen::Vector3f>::iterator it = startSegmentValues.begin(); it != startSegmentValues.end(); ++it)
        {
            if (endSegmentValues.find((*it).first) != endSegmentValues.end())
            {
                if (startPoint[0] == 0.0f && startPoint[1] == 0.0f && startPoint[2] == 0.0f)
                {
                    mainSegment = (*it).first;
                    startPoint = (*it).second;
                }
                startPoints.push_back((*it).second);
                endPoints.push_back(endSegmentValues[(*it).first]);
            }
        }

        int i = 0;

        for (std::vector<MotionSlice>::iterator it = motionSlicesByDistance.begin(); i < std::min(n, (int)motionSlicesByDistance.size()); ++it)
        {
            tab.getValue<std::string>("info").set("file: " + (*it).motion->getMotionFileName() + " - rank " + std::to_string(i + 1));
            tab.sendUpdates();
            visualizeMotionSlice(*it, previousHeight, i + 1);
            ++i;
        }
    }

    std::vector<float> HumanPoseMatching::getJointDifferences(MMM::LegacyMotionPtr motion, std::map<std::string, float>& startJointValues, std::vector<std::string> jointNames)
    {
        std::vector<float> jointDifferences = std::vector<float>();

        for (std::map<std::string, float>::iterator it = startJointValues.begin(); it != startJointValues.end(); it++)
        {
            std::vector<std::string>::iterator jointNameIt = std::find(jointNames.begin(), jointNames.end(), (*it).first);

            if (jointDifferences.size() == 0)
            {
                jointDifferences = getJointDifferenceToAngle(motion, (*it).second, std::distance(jointNames.begin(), jointNameIt));
            }
            else
            {
                std::vector<float> jointDifferencesTmp = getJointDifferenceToAngle(motion, (*it).second, std::distance(jointNames.begin(), jointNameIt));

                std::transform(jointDifferences.begin(), jointDifferences.end(), jointDifferencesTmp.begin(), jointDifferences.begin(), std::plus<float>());
            }
        }

        std::transform(jointDifferences.begin(), jointDifferences.end(), jointDifferences.begin(),
                       std::bind(std::multiplies<float>(), std::placeholders::_1, 1.0f / (2 * M_PI) * 3000 * previousHeight));

        //normalize differences by dividing by number of joints specified
        std::transform(jointDifferences.begin(), jointDifferences.end(), jointDifferences.begin(),
                       std::bind(std::multiplies<float>(), std::placeholders::_1, 1.0f / startJointValues.size()));

        return jointDifferences;
    }

    std::vector<float> HumanPoseMatching::getSegmentDifferences(MMM::LegacyMotionPtr motion, std::map<std::string, Eigen::Vector3f>& startSegmentValues, VirtualRobot::RobotPtr r, VirtualRobot::RobotNodeSetPtr robotNodeSet, float height)
    {
        std::vector<float> segmentDistances = std::vector<float>();

        std::vector<float> segmentDistancesTmp = std::vector<float>();

        for (std::map<std::string, Eigen::Vector3f>::iterator it = startSegmentValues.begin(); it != startSegmentValues.end(); it++)
        {
            if (segmentDistances.size() == 0)
            {
                if (previousGlobalCoordinatesInput)
                {
                    segmentDistances = getSegmentGlobalDistanceToPoint(motion, (*it).second, (*it).first, r, robotNodeSet, height);
                }
                else
                {
                    segmentDistances = getSegmentLocalDistanceToPoint(motion, (*it).second, (*it).first, r, robotNodeSet, height);
                }
            }
            else
            {
                if (previousGlobalCoordinatesInput)
                {
                    segmentDistancesTmp = getSegmentGlobalDistanceToPoint(motion, (*it).second, (*it).first, r, robotNodeSet, height);
                }
                else
                {
                    segmentDistancesTmp = getSegmentLocalDistanceToPoint(motion, (*it).second, (*it).first, r, robotNodeSet, height);
                }


                std::transform(segmentDistances.begin(), segmentDistances.end(), segmentDistancesTmp.begin(), segmentDistances.begin(), std::plus<float>());
            }
        }

        // normalize segment distances by dividing each distance by number of segments
        std::transform(segmentDistances.begin(), segmentDistances.end(), segmentDistances.begin(),
                       std::bind(std::multiplies<float>(), std::placeholders::_1, 1.0f / startSegmentValues.size()));
        return segmentDistances;
    }

    void getMotionSlicesForMotion(std::vector<MotionSlice>& motionSlicesByDistance, MMM::LegacyMotionPtr motion, std::vector<float> differences, float threshold)
    {
        float minDifference = HUGE_VALF;

        MotionSlice currentSlice{motion, -1, -1, -1, -1, HUGE_VALF, HUGE_VALF, HUGE_VALF, HUGE_VALF, HUGE_VALF};

        for (unsigned int i = 0; i < motion->getNumFrames(); ++i)
        {

            float difference = differences[i];

            if (difference < threshold && i < motion->getNumFrames() - 1)
            {
                // entweder dieses motion frame ist das erste unter dem threshold
                if (currentSlice.beginIndex == -1)
                {
                    currentSlice.beginIndex = i;
                }

                if (difference < minDifference)
                {
                    minDifference = difference;
                    currentSlice.minIndex = i;
                    currentSlice.minStartDifference = difference;
                }
            }
            else
            {
                if (currentSlice.beginIndex != -1)
                {
                    currentSlice.endIndex = i - 1;

                    motionSlicesByDistance.push_back(currentSlice);

                    currentSlice.beginIndex = -1;
                    currentSlice.endIndex = -1;
                    currentSlice.minIndex = -1;
                    currentSlice.minStartDifference = HUGE_VALF;
                    minDifference = HUGE_VALF;
                }
            }
        }
    }

    std::vector<MotionSlice> getMotionSlicesForMotion(MMM::LegacyMotionPtr motion, std::vector<float>& differences, float threshold)
    {
        float minDifference = HUGE_VALF;
        std::vector<MotionSlice> motionSlices = std::vector<MotionSlice> {};
        MotionSlice currentSlice{motion, -1, -1, -1, -1, HUGE_VALF, HUGE_VALF, HUGE_VALF, HUGE_VALF, HUGE_VALF};

        for (unsigned int i = 0; i < motion->getNumFrames(); ++i)
        {

            float difference = differences[i];

            if (difference < threshold && i < motion->getNumFrames() - 1)
            {
                // entweder dieses motion frame ist das erste unter dem threshold
                if (currentSlice.beginIndex == -1)
                {
                    currentSlice.beginIndex = i;
                }

                if (difference < minDifference)
                {
                    minDifference = difference;
                    currentSlice.minIndex = i;
                    currentSlice.minStartDifference = difference;
                }
            }
            else
            {
                if (currentSlice.beginIndex != -1)
                {

                    currentSlice.endIndex = i - 1;

                    motionSlices.push_back(currentSlice);

                    currentSlice.beginIndex = -1;
                    currentSlice.endIndex = -1;
                    currentSlice.minIndex = -1;
                    minDifference = HUGE_VALF;
                    currentSlice.minStartDifference = HUGE_VALF;
                }
            }
        }
        return motionSlices;
    }

    bool compareTwoMotionSlicesByStartIndex(MotionSlice a, MotionSlice b)
    {
        return a.beginIndex < b.beginIndex;
    }

    bool compareTwoMotionSlicesByStartDifference(MotionSlice a, MotionSlice b)
    {
        return a.minStartDifference < b.minStartDifference;
    }

    bool compareTwoMotionSlicesByEndDifference(MotionSlice a, MotionSlice b)
    {
        return a.minEndDifference < b.minEndDifference;
    }

    bool compareTwoMotionSlicesByBothDifferences(MotionSlice a, MotionSlice b)
    {
        return (a.minStartDifference + a.minEndDifference + (a.minEndIndex - a.minIndex) / 5.0f) < (b.minStartDifference + b.minEndDifference + (b.minEndIndex - b.minIndex) / 5.0f);
    }

    bool compareTwoMotionSlicesByDeviationFromDistance(MotionSlice a, MotionSlice b)
    {
        return a.deviationFromDistance < b.deviationFromDistance;
    }

    bool compareTwoMotionSlicesByDurationAndTrajectory(MotionSlice a, MotionSlice b)
    {
        float durationWeight = 1.5f;
        float trajectoryWeight = 1.0f;

        return a.duration * durationWeight  + a.trajectory * trajectoryWeight + a.deviationFromDistance / 50 < b.duration * durationWeight + b.trajectory * trajectoryWeight + b.deviationFromDistance / 50;
    }

    std::vector<MotionSlice> getMotionSlicesFromStartAndEnd(MMM::LegacyMotionPtr motion, std::vector<float>& startDifferences, std::vector<float>& endDifferences, float threshold)
    {
        std::vector<MotionSlice> motionSlices = std::vector<MotionSlice>();

        MotionSlice currentSlice{motion, -1, -1, -1, -1, HUGE_VALF, HUGE_VALF, HUGE_VALF, HUGE_VALF, HUGE_VALF};

        std::vector<MotionSlice> startSlices = getMotionSlicesForMotion(motion, startDifferences, threshold);
        std::vector<MotionSlice> endSlices = getMotionSlicesForMotion(motion, endDifferences, threshold);

        for (unsigned int i = 0; i < startSlices.size(); i++)
        {
            for (unsigned j = 0; j < endSlices.size(); j++)
            {
                if (startSlices[i].minIndex < endSlices[j].minIndex)
                {
                    currentSlice.beginIndex = startSlices[i].beginIndex;
                    currentSlice.endIndex = endSlices[j].endIndex;
                    currentSlice.minIndex = startSlices[i].minIndex;
                    currentSlice.minStartDifference = startSlices[i].minStartDifference;
                    currentSlice.minEndDifference = endSlices[j].minStartDifference;
                    currentSlice.minEndIndex = endSlices[j].minIndex;

                    motionSlices.push_back(currentSlice);

                    currentSlice.beginIndex = -1;
                    currentSlice.endIndex = -1;
                    currentSlice.minIndex = -1;
                    currentSlice.minEndIndex = -1;
                    currentSlice.minStartDifference = HUGE_VALF;
                    currentSlice.minEndDifference = HUGE_VALF;
                }
            }
        }

        return motionSlices;
    }

    std::vector<float> HumanPoseMatching::getDifferences(MMM::LegacyMotionPtr motion, std::map<std::string, Eigen::Vector3f>& segmentValues, std::map<std::string, float>& jointValues, float alpha, VirtualRobot::RobotPtr r, VirtualRobot::RobotNodeSetPtr robotNodeSet, float height)
    {
        std::vector<float> differences = std::vector<float>();
        std::vector<float> jointDifferences = std::vector<float>();
        std::vector<float> segmentDistances = std::vector<float>();

        differences.resize(motion->getMotionFrames().size());

        if (segmentValues.size() == 0 && jointValues.size() == 0)
        {
            ARMARX_IMPORTANT << "no start values specified";
            return std::vector<float>();
        }

        if (jointValues.size() > 0)
        {
            jointDifferences = getJointDifferences(motion, jointValues, motion->getJointNames());

            if (segmentValues.size() == 0)
            {
                differences = jointDifferences;
            }
        }

        if (segmentValues.size() > 0)
        {
            segmentDistances = getSegmentDifferences(motion, segmentValues, r, robotNodeSet, height);

            if (jointValues.size() == 0)
            {
                differences = segmentDistances;
            }
        }

        if (jointValues.size() > 0 && segmentValues.size() > 0)
        {
            ARMARX_IMPORTANT << "both start joints and segments specified, using weighted sum";

            std::transform(segmentDistances.begin(), segmentDistances.end(), segmentDistances.begin(),
                           std::bind(std::multiplies<float>(), std::placeholders::_1, (1 - alpha)));

            std::transform(jointDifferences.begin(), jointDifferences.end(), jointDifferences.begin(),
                           std::bind(std::multiplies<float>(), std::placeholders::_1, alpha));

            std::transform(jointDifferences.begin(), jointDifferences.end(), segmentDistances.begin(), differences.begin(), std::plus<float>());
        }

        return differences;
    }

    std::vector<MotionSlice> flatten(std::vector<std::vector<MotionSlice>>& vectorOfVectors)
    {
        std::vector<MotionSlice> motionSlices = std::vector<MotionSlice>();

        for (unsigned int i = 0; i < vectorOfVectors.size(); ++i)
        {
            for (unsigned int j = 0; j < vectorOfVectors[i].size(); ++j)
            {
                motionSlices.push_back(vectorOfVectors[i][j]);
            }
        }

        return motionSlices;
    }

    std::vector<MotionSlice> HumanPoseMatching::getBestMotionSlicesStartValuesOnly(std::vector<MMM::LegacyMotionPtr> motions, std::map<std::string, float> startJointValues, std::map<std::string, Eigen::Vector3f> startSegmentValues, float alpha, float inputThreshold, float height)
    {
        std::vector<std::vector<MotionSlice>> motionSlicesByDistance = std::vector<std::vector<MotionSlice>>();

        unsigned int numberOfMotions = motions.size();

        motionSlicesByDistance.resize(numberOfMotions);

        std::atomic<int> numberOfMotionsProcessed = 0;

        #pragma omp parallel for
        for (unsigned int i = 0; i < numberOfMotions; ++i)
        {
            ++numberOfMotionsProcessed;
            ARMARX_IMPORTANT << "Processing motion " << numberOfMotionsProcessed << " of " << numberOfMotions;

            MMM::ModelPtr model = motions[i]->getModel(false);

            MMM::ModelProcessorWinter processor = MMM::ModelProcessorWinter();
            processor.setup(height, height * model->getMass());
            model = processor.convertModel(model);

            VirtualRobot::RobotPtr r = MMM::SimoxTools::buildModel(model, false);
            VirtualRobot::RobotNodeSetPtr robotNodeSet = VirtualRobot::RobotNodeSet::createRobotNodeSet(r, "ExampleRobotNodeSet", motions[i]->getJointNames(), "", "", true);

            std::vector<float> differences = getDifferences(motions[i], startSegmentValues, startJointValues, alpha, r, robotNodeSet, height);

            motionSlicesByDistance[i] = getMotionSlicesForMotion(motions[i], differences, inputThreshold);
        }

        std::vector<MotionSlice> flattenedMotionSlices = flatten(motionSlicesByDistance);

        std::sort(flattenedMotionSlices.begin(), flattenedMotionSlices.end(), compareTwoMotionSlicesByStartDifference);

        return flattenedMotionSlices;
    }

    std::vector<MotionSlice> HumanPoseMatching::getBestMotionSlicesStartAndEndValues(std::vector<MMM::LegacyMotionPtr> motions, std::map<std::string, float> startJointValues, std::map<std::string, Eigen::Vector3f> startSegmentValues, std::map<std::string, float> endJointValues, std::map<std::string, Eigen::Vector3f> endSegmentValues, float alpha, float inputThreshold, float height)
    {
        std::vector<std::vector<MotionSlice>> motionSlicesByDistance = std::vector<std::vector<MotionSlice>>();

        unsigned int numberOfMotions = motions.size();

        motionSlicesByDistance.resize(numberOfMotions);

        std::atomic<int> numberOfMotionsProcessed = 0;

        #pragma omp parallel for
        for (unsigned int i = 0; i < numberOfMotions; ++i)
        {
            ++numberOfMotionsProcessed;
            ARMARX_IMPORTANT << "Processing motion " << numberOfMotionsProcessed << " of " << numberOfMotions;

            MMM::ModelPtr model = motions[i]->getModel(false);
            MMM::ModelProcessorWinter processor = MMM::ModelProcessorWinter();
            processor.setup(height, height * model->getMass());
            model = processor.convertModel(model);

            VirtualRobot::RobotPtr r = MMM::SimoxTools::buildModel(model, false);
            VirtualRobot::RobotNodeSetPtr robotNodeSet = VirtualRobot::RobotNodeSet::createRobotNodeSet(r, "ExampleRobotNodeSet", motions[i]->getJointNames(), "", "", true);

            std::vector<float> startDifferences = getDifferences(motions[i], startSegmentValues, startJointValues, alpha, r, robotNodeSet, height);
            std::vector<float> endDifferences = getDifferences(motions[i], endSegmentValues, endJointValues, alpha, r, robotNodeSet, height);

            motionSlicesByDistance[i] = getMotionSlicesFromStartAndEnd(motions[i], startDifferences, endDifferences, inputThreshold);
        }

        std::vector<MotionSlice> flattenedMotionSlices = flatten(motionSlicesByDistance);

        std::sort(flattenedMotionSlices.begin(), flattenedMotionSlices.end(), compareTwoMotionSlicesByBothDifferences);

        return flattenedMotionSlices;
    }

    std::map<std::string, float> getSegmentDistanceMap(std::map<std::string, Eigen::Vector3f>& startSegmentValues, std::map<std::string, Eigen::Vector3f>& endSegmentValues)
    {
        std::map<std::string, float> segmentDistanceMap = std::map<std::string, float>();

        for (std::map<std::string, Eigen::Vector3f>::iterator startIt = startSegmentValues.begin(); startIt != startSegmentValues.end(); startIt++)
        {
            std::string startSegmentName = (*startIt).first;

            if (endSegmentValues.find(startSegmentName) != endSegmentValues.end())
            {
                float distance = getDistance(startSegmentValues[startSegmentName], endSegmentValues[startSegmentName]);

                segmentDistanceMap[startSegmentName] = distance;
            }
        }

        return segmentDistanceMap;
    }

    Eigen::Vector3f HumanPoseMatching::getDirection(Eigen::Vector3f& startPoint, Eigen::Vector3f& endPoint)
    {
        Eigen::Vector3f direction = endPoint - startPoint;
        return direction.normalized();
    }

    std::map<std::string, Eigen::Vector3f> HumanPoseMatching::getSegmentDirectionMap(std::map<std::string, Eigen::Vector3f>& startSegmentValues, std::map<std::string, Eigen::Vector3f>& endSegmentValues)
    {
        std::map<std::string, Eigen::Vector3f> segmentDirectionMap = std::map<std::string, Eigen::Vector3f>();

        for (std::map<std::string, Eigen::Vector3f>::iterator startIt = startSegmentValues.begin(); startIt != startSegmentValues.end(); startIt++)
        {
            std::string startSegmentName = (*startIt).first;

            if (endSegmentValues.find(startSegmentName) != endSegmentValues.end())
            {
                Eigen::Vector3f direction = getDirection(startSegmentValues[startSegmentName], endSegmentValues[startSegmentName]);

                segmentDirectionMap[startSegmentName] = direction;
            }
        }

        return segmentDirectionMap;
    }

    std::vector<MotionSlice> HumanPoseMatching::getBestMotionSlicesGlobalStartAndEndValues(std::vector<MMM::LegacyMotionPtr>& motions, std::map<std::string, float>& startJointValues, std::map<std::string, Eigen::Vector3f>& startSegmentValues, std::map<std::string, float>& endJointValues, std::map<std::string, Eigen::Vector3f>& endSegmentValues, float alpha, float inputThreshold, float height)
    {
        std::vector<std::vector<MotionSlice>> motionSlicesByDistance = std::vector<std::vector<MotionSlice>>();

        std::map<std::string, float> segmentDistanceMap = getSegmentDistanceMap(startSegmentValues, endSegmentValues);

        unsigned int numberOfMotions = motions.size();

        motionSlicesByDistance.resize(numberOfMotions);

        std::atomic<int> numberOfLoadedMotions = 0;

        #pragma omp parallel for
        for (unsigned int i = 0; i < numberOfMotions; ++i)
        {
            ++numberOfLoadedMotions;
            ARMARX_IMPORTANT << "Processing motion " << numberOfLoadedMotions << " of " << numberOfMotions;

            MMM::ModelPtr model = motions[i]->getModel(false);
            MMM::ModelProcessorWinter processor = MMM::ModelProcessorWinter();
            processor.setup(height, height * model->getMass());
            model = processor.convertModel(model);

            VirtualRobot::RobotPtr r = MMM::SimoxTools::buildModel(model, false);
            VirtualRobot::RobotNodeSetPtr robotNodeSet = VirtualRobot::RobotNodeSet::createRobotNodeSet(r, "ExampleRobotNodeSet", motions[i]->getJointNames(), "", "", true);

            std::vector<float> startDifferences = getDifferences(motions[i], startSegmentValues, startJointValues, alpha, r, robotNodeSet, height);
            std::vector<float> endDifferences = getDifferences(motions[i], endSegmentValues, endJointValues, alpha, r, robotNodeSet, height);

            std::vector<MotionSlice> motionSlicesToCheck = getMotionSlicesFromStartAndEnd(motions[i], startDifferences, endDifferences, inputThreshold);


            for (unsigned int j = 0; j < motionSlicesToCheck.size(); j++)
            {
                motionSlicesToCheck[j].deviationFromDistance = getAggregatedDistanceDeviation(motionSlicesToCheck[j], segmentDistanceMap, r, robotNodeSet);
            }

            motionSlicesByDistance[i] = motionSlicesToCheck;
        }

        std::vector<MotionSlice> flattenedMotionSlices = flatten(motionSlicesByDistance);

        std::sort(flattenedMotionSlices.begin(), flattenedMotionSlices.end(), compareTwoMotionSlicesByDeviationFromDistance);

        // get first fifty motion slices
        flattenedMotionSlices.resize(std::min(100, (int)flattenedMotionSlices.size()));

        std::vector<MotionSlice> normalizedMotionSlices = normalizeMotionSlices(flattenedMotionSlices, startSegmentValues, endSegmentValues);
        // sort them by duration and trajectory

        std::sort(normalizedMotionSlices.begin(), normalizedMotionSlices.end(), compareTwoMotionSlicesByDurationAndTrajectory);

        // return

        return normalizedMotionSlices;
    }

    void HumanPoseMatching::fillDurationForMotionSlice(MotionSlice& motionSlice)
    {
        motionSlice.duration = motionSlice.minEndIndex - motionSlice.minIndex;
    }

    void HumanPoseMatching::fillTrajectoryForMotionSlice(MotionSlice& motionSlice, std::map<std::string, Eigen::Vector3f>& idealDirectionsMap)
    {
        MMM::ModelPtr model = motionSlice.motion->getModel(false);

        MMM::ModelProcessorWinter processor = MMM::ModelProcessorWinter();
        processor.setup(previousHeight, previousHeight * model->getMass());
        model = processor.convertModel(model);

        VirtualRobot::RobotPtr r = MMM::SimoxTools::buildModel(model, false);
        VirtualRobot::RobotNodeSetPtr robotNodeSet = VirtualRobot::RobotNodeSet::createRobotNodeSet(r, "ExampleRobotNodeSet", motionSlice.motion->getJointNames(), "", "", true);


        unsigned int sizeOfMotionSlice = motionSlice.duration;

        float totalDifference = 0.0f;

        int numberOfSamples = sizeOfMotionSlice;

        float actualHeight = motionSlice.motion->getModel()->getHeight();
        std::vector<Eigen::Vector3f> previousSegmentPositions = std::vector<Eigen::Vector3f>();
        previousSegmentPositions.resize(idealDirectionsMap.size());


        for (int i = motionSlice.minIndex; i < motionSlice.minEndIndex; ++i)
        {
            MMM::MotionFramePtr motionFrame = motionSlice.motion->getMotionFrame(i);

            Eigen::Matrix4f rootPose = motionFrame->getRootPose();
            rootPose(0, 3) = rootPose(0, 3) / actualHeight * previousHeight;
            rootPose(1, 3) = rootPose(1, 3) / actualHeight * previousHeight;
            rootPose(2, 3) = rootPose(2, 3) / actualHeight * previousHeight;
            r->setGlobalPose(rootPose);
            robotNodeSet->setJointValues(motionFrame->joint);
            // ...

            int j = 0;

            if (i % (sizeOfMotionSlice / numberOfSamples) == 0)
            {
                j = 0;
                if (i > motionSlice.minIndex)
                {
                    for (std::map<std::string, Eigen::Vector3f>::iterator it = idealDirectionsMap.begin(); it != idealDirectionsMap.end(); ++it)
                    {
                        VirtualRobot::RobotNodePtr segmentNode = r->getRobotNode((*it).first);
                        Eigen::Vector3f actualDirection = segmentNode->getGlobalDirection(-segmentNode->getDirectionInRootFrame(previousSegmentPositions[j])).normalized();

                        float angle = std::acos(actualDirection.dot((*it).second));

                        totalDifference += angle / numberOfSamples;

                        ++j;
                    }
                }
            }

            if (i % (sizeOfMotionSlice / numberOfSamples) == (sizeOfMotionSlice / numberOfSamples) - 1)
            {
                j = 0;
                for (std::map<std::string, Eigen::Vector3f>::iterator it = idealDirectionsMap.begin(); it != idealDirectionsMap.end(); ++it)
                {
                    VirtualRobot::RobotNodePtr segmentNode = r->getRobotNode((*it).first);
                    previousSegmentPositions[j] = segmentNode->getPositionInRootFrame();

                    ++j;
                }
            }
        }

        motionSlice.trajectory = totalDifference;
    }

    std::vector<MotionSlice> HumanPoseMatching::normalizeMotionSlices(std::vector<MotionSlice>& motionSlices, std::map<std::string, Eigen::Vector3f>& startSegmentValues, std::map<std::string, Eigen::Vector3f>& endSegmentValues)
    {
        std::map<std::string, Eigen::Vector3f> idealdirections = getSegmentDirectionMap(startSegmentValues, endSegmentValues);

        float minTrajectory = HUGE_VALF;
        float maxTrajectory = -HUGE_VALF;

        float minDuration = HUGE_VALF;
        float maxDuration = -HUGE_VALF;

        // find max and min duration and trajectory
        for (unsigned int i = 0; i < motionSlices.size(); i++)
        {
            // fill duration and trajectory for each motion slice
            fillDurationForMotionSlice(motionSlices[i]);
            if (motionSlices[i].duration < minDuration)
            {
                minDuration = motionSlices[i].duration;
            }

            if (motionSlices[i].duration > maxDuration)
            {
                maxDuration = motionSlices[i].duration;
            }

            fillTrajectoryForMotionSlice(motionSlices[i], idealdirections);
            if (motionSlices[i].trajectory < minTrajectory)
            {
                minTrajectory = motionSlices[i].trajectory;
            }

            if (motionSlices[i].trajectory > maxTrajectory)
            {
                maxTrajectory = motionSlices[i].trajectory;
            }
        }

        maxDuration = std::min(10 * minDuration, maxDuration);
        maxTrajectory = std::min(10 * minTrajectory, maxTrajectory);

        float durationSpan = maxDuration - minDuration;
        float trajectorySpan = maxTrajectory - minTrajectory;

        // normalize all durations and trajectories with respect to max and min
        for (unsigned int i = 0; i < motionSlices.size(); ++i)
        {
            motionSlices[i].duration = (motionSlices[i].duration - minDuration) / durationSpan;
            motionSlices[i].trajectory = (motionSlices[i].trajectory - minTrajectory) / trajectorySpan;
        }

        return motionSlices;
    }

    float HumanPoseMatching::getAggregatedDistanceDeviation(MotionSlice& slice, std::map<std::string, float>& segmentDistanceMap, VirtualRobot::RobotPtr r, VirtualRobot::RobotNodeSetPtr robotNodeSet)
    {
        float aggregatedDistance = 0.0f;

        std::vector<Eigen::Vector3f> startPositions = std::vector<Eigen::Vector3f>();
        std::vector<Eigen::Vector3f> endPositions = std::vector<Eigen::Vector3f>();

        float actualHeight = slice.motion->getModel()->getHeight();

        MMM::MotionFramePtr startMotionFrame = slice.motion->getMotionFrame(slice.minIndex);

        Eigen::Matrix4f startRootPose = startMotionFrame->getRootPose();
        startRootPose(0, 3) = startRootPose(0, 3) / actualHeight * previousHeight;
        startRootPose(1, 3) = startRootPose(1, 3) / actualHeight * previousHeight;
        startRootPose(2, 3) = startRootPose(2, 3) / actualHeight * previousHeight;
        r->setGlobalPose(startRootPose);
        robotNodeSet->setJointValues(startMotionFrame->joint);
        // ...

        for (std::map<std::string, float>::iterator it = segmentDistanceMap.begin(); it != segmentDistanceMap.end(); ++it)
        {
            VirtualRobot::RobotNodePtr segmentNode = r->getRobotNode((*it).first);

            startPositions.push_back(segmentNode->getGlobalPosition());
            ARMARX_IMPORTANT << segmentNode->getGlobalPosition();
        }

        MMM::MotionFramePtr endMotionFrame = slice.motion->getMotionFrame(slice.minEndIndex);

        Eigen::Matrix4f endRootPose = endMotionFrame->getRootPose();
        endRootPose(0, 3) = endRootPose(0, 3) / actualHeight * previousHeight;
        endRootPose(1, 3) = endRootPose(1, 3) / actualHeight * previousHeight;
        endRootPose(2, 3) = endRootPose(2, 3) / actualHeight * previousHeight;
        r->setGlobalPose(endRootPose);
        robotNodeSet->setJointValues(endMotionFrame->joint);

        ARMARX_IMPORTANT << "actual height: " << actualHeight << ", previous height: " << previousHeight;
        // ...

        for (std::map<std::string, float>::iterator it = segmentDistanceMap.begin(); it != segmentDistanceMap.end(); ++it)
        {
            VirtualRobot::RobotNodePtr segmentNode = r->getRobotNode((*it).first);

            endPositions.push_back(segmentNode->getGlobalPosition());
            ARMARX_IMPORTANT << segmentNode->getGlobalPosition();
        }

        int index = 0;

        for (std::map<std::string, float>::iterator it = segmentDistanceMap.begin(); it != segmentDistanceMap.end(); ++it)
        {
            float idealDistance = (*it).second;
            float actualDistance = getDistance(endPositions[index], startPositions[index]);

            aggregatedDistance += std::abs(idealDistance - actualDistance);

            ++index;
        }

        return aggregatedDistance / (float)index;
    }

    void HumanPoseMatching::drawInputVisualization()
    {

        viz::Layer testLayer = arviz.layer("Test");
        testLayer.clear();

        viz::Box referenceBox = viz::Box("referenceBox")
                                .position(Eigen::Vector3f(0, 4000, 500))
                                .color(viz::Color::fromRGBA(132, 140, 114))
                                .size(Eigen::Vector3f(1000.0f, 1000.0f, 1000.0f));

        viz::Robot floor = viz::Robot("floor").file("HRPGroup3", "MMM/floor.xml").position(Eigen::Vector3f::Zero());
        floor.useFullModel();

        //            viz::Sphere startSphere = viz::Sphere("startSphere")
        //                                         .position(startPoint)
        //                                         .color(viz::Color::green())
        //                                         .radius(50.0f);

        //            viz::Box targetBox = viz::Box("targetBox")
        //                                         .position(endPoint)
        //                                         .color(viz::Color::green())
        //                                         .size(Eigen::Vector3f(100.0f, 100.0f, 100.0f));

        if (previousGlobalCoordinatesInput)
        {
            int i = 0;
            for (std::map<std::string, Eigen::Vector3f>::iterator it = startPointsMap.begin(); it != startPointsMap.end(); ++it)
            {
                viz::Sphere startSphere = viz::Sphere("startSphere" + (*it).first)
                                          .position((*it).second)
                                          .color(startColors[i])
                                          .radius(50.0f);

                testLayer.add(startSphere);
                ++i;
            }

            i = 0;
            for (std::map<std::string, Eigen::Vector3f>::iterator it = endPointsMap.begin(); it != endPointsMap.end(); ++it)
            {
                viz::Box endBox = viz::Box("endBox" + (*it).first)
                                  .position((*it).second)
                                  .color(endColors[i])
                                  .size(Eigen::Vector3f(100.0f, 100.0f, 100.0f));

                testLayer.add(endBox);
                ++i;
            }



            i = 0;
            for (std::map<std::string, std::pair<Eigen::Vector3f, Eigen::Vector3f>>::iterator it = startEndPairs.begin(); it != startEndPairs.end(); ++it)
            {
                viz::Sphere startSphere = viz::Sphere("startSphere" + (*it).first)
                                          .position((*it).second.first)
                                          .color(pairColors[i])
                                          .radius(50.0f);

                testLayer.add(startSphere);

                viz::Box endBox = viz::Box("endBox" + (*it).first)
                                  .position((*it).second.second)
                                  .color(pairColors[i])
                                  .size(Eigen::Vector3f(100.0f, 100.0f, 100.0f));

                testLayer.add(endBox);
                ++i;
            }
        }




        testLayer.add(floor);
        testLayer.add(referenceBox);


        arviz.commit({testLayer});
    }

    void HumanPoseMatching::waitForInput()
    {
        RemoteGui::TabProxy tab = RemoteGui::TabProxy(getRemoteGui(), getName());

        RemoteGui::ValueProxy<bool> showEndInput = tab.getValue<bool>("End Point");
        RemoteGui::ValueProxy<bool> globalCoordinates = tab.getValue<bool>("Global Coordinates");
        RemoteGui::ValueProxy<float> height = tab.getValue<float>("heightBox");

        // new gui value
        RemoteGui::ValueProxy<std::string> startRobotNode = tab.getValue<std::string>("startRobotNodeCombobox");
        RemoteGui::ValueProxy<std::string> endRobotNode = tab.getValue<std::string>("endRobotNodeCombobox");
        RemoteGui::ValueProxy<float> offsetHeight = tab.getValue<float>("offsetHeightBox");

        while (!tab.getButton("submitButton").clicked())
        {

            drawInputVisualization();


            tab.receiveUpdates();
            if (tab.getButton("startaddButton").clicked())
            {
                using namespace RemoteGui;

                if (std::find(startRowNames.begin(), startRowNames.end(), startRobotNode.get()) == startRowNames.end())
                {
                    startRowNames.push_back(startRobotNode.get());
                    if (isSegment(startRobotNode.get()))
                    {
                        if (std::find(endRowNames.begin(), endRowNames.end(), startRobotNode.get()) == endRowNames.end())
                        {
                            startPointsMap[startRobotNode.get()] = Eigen::Vector3f::Zero();
                        }
                        else
                        {
                            startEndPairs[startRobotNode.get()] = std::pair<Eigen::Vector3f, Eigen::Vector3f>(Eigen::Vector3f::Zero(), endPointsMap[startRobotNode.get()]);
                            endPointsMap.erase(startRobotNode.get());
                        }
                    }
                }

                createStartEndTab_Widgets();
            }
            else if (previousShowEndInput != showEndInput.get())
            {
                previousShowEndInput = showEndInput.get();
                createStartEndTab_Widgets();
            }
            else if (tab.getButton("endaddButton").clicked())
            {
                using namespace RemoteGui;

                if (std::find(endRowNames.begin(), endRowNames.end(), endRobotNode.get()) == endRowNames.end())
                {
                    endRowNames.push_back(endRobotNode.get());

                    if (isSegment(endRobotNode.get()))
                    {
                        if (std::find(startRowNames.begin(), startRowNames.end(), endRobotNode.get()) == startRowNames.end())
                        {
                            endPointsMap[endRobotNode.get()] = Eigen::Vector3f::Zero();
                        }
                        else
                        {
                            startEndPairs[endRobotNode.get()] = std::pair<Eigen::Vector3f, Eigen::Vector3f>(startPointsMap[endRobotNode.get()], Eigen::Vector3f::Zero());
                            startPointsMap.erase(endRobotNode.get());
                        }
                    }
                }

                createStartEndTab_Widgets();
            }
            else if (previousGlobalCoordinatesInput != globalCoordinates.get())
            {
                previousGlobalCoordinatesInput = globalCoordinates.get();
            }
            else if (height.get() != previousHeight)
            {
                previousHeight = height.get();
            }
            else if (offsetHeight.get() != previousOffsetHeight)
            {
                previousOffsetHeight = offsetHeight.get();
            }

            for (int i = startRowNames.size() - 1; i >= 0; i--)
            {
                std::string startRowName = startRowNames[i];
                if (tab.getButton(startRowName + "startDeleteButton").clicked())
                {
                    startRowNames.erase(startRowNames.begin() + i);

                    if (std::find(endRowNames.begin(), endRowNames.end(), startRowName) == endRowNames.end())
                    {
                        startPointsMap.erase(startRowName);
                    }
                    else
                    {
                        endPointsMap[startRowName] = startEndPairs[startRowName].second;
                        startEndPairs.erase(startRowName);
                    }
                    createStartEndTab_Widgets();
                }
            }

            for (int i = endRowNames.size() - 1; i >= 0; i--)
            {
                std::string endRowName = endRowNames[i];
                if (tab.getButton(endRowName + "endDeleteButton").clicked())
                {
                    endRowNames.erase(endRowNames.begin() + i);

                    if (std::find(startRowNames.begin(), startRowNames.end(), endRowName) == startRowNames.end())
                    {
                        endPointsMap.erase(endRowName);
                    }
                    else
                    {
                        startPointsMap[endRowName] = startEndPairs[endRowName].first;
                        startEndPairs.erase(endRowName);
                    }
                    createStartEndTab_Widgets();
                }
            }

            for (std::map<std::string, Eigen::Vector3f>::iterator it = startPointsMap.begin(); it != startPointsMap.end(); ++it)
            {
                float x = tab.getValue<float>((*it).first + "startPositionXBox").get();
                float y = tab.getValue<float>((*it).first + "startPositionYBox").get();
                float z = tab.getValue<float>((*it).first + "startPositionZBox").get();

                if ((*it).second[0] != x || (*it).second[1] != y || (*it).second[2] != z)
                {
                    (*it).second = Eigen::Vector3f(x, y, z);
                }
            }

            for (std::map<std::string, Eigen::Vector3f>::iterator it = endPointsMap.begin(); it != endPointsMap.end(); ++it)
            {
                float x = tab.getValue<float>((*it).first + "endPositionXBox").get();
                float y = tab.getValue<float>((*it).first + "endPositionYBox").get();
                float z = tab.getValue<float>((*it).first + "endPositionZBox").get();

                if ((*it).second[0] != x || (*it).second[1] != y || (*it).second[2] != z)
                {
                    (*it).second = Eigen::Vector3f(x, y, z);
                }
            }

            for (std::map<std::string, std::pair<Eigen::Vector3f, Eigen::Vector3f>>::iterator it = startEndPairs.begin(); it != startEndPairs.end(); ++it)
            {
                float startX = tab.getValue<float>((*it).first + "startPositionXBox").get();
                float startY = tab.getValue<float>((*it).first + "startPositionYBox").get();
                float startZ = tab.getValue<float>((*it).first + "startPositionZBox").get();

                float endX = tab.getValue<float>((*it).first + "endPositionXBox").get();
                float endY = tab.getValue<float>((*it).first + "endPositionYBox").get();
                float endZ = tab.getValue<float>((*it).first + "endPositionZBox").get();

                if ((*it).second.first[0] != startX || (*it).second.first[1] != startY || (*it).second.first[2] != startZ)
                {
                    (*it).second.first = Eigen::Vector3f(startX, startY, startZ);
                }

                if ((*it).second.second[0] != endX || (*it).second.second[1] != endY || (*it).second.second[2] != endZ)
                {
                    (*it).second.second = Eigen::Vector3f(endX, endY, endZ);
                }
            }
        }
    }


    void HumanPoseMatching::taskRun()
    {

        RemoteGui::TabProxy tab = RemoteGui::TabProxy(getRemoteGui(), getName());

        RemoteGui::ValueProxy<int> numberOfFiles = tab.getValue<int>("fileNumberBox");

        RemoteGui::ValueProxy<int> numberOfBestMatches = tab.getValue<int>("intSpinBox");

        ARMARX_IMPORTANT << "waiting for submit button" ;

        waitForInput();

        tab.receiveUpdates();

        std::map<std::string, float> startJointValues = getStartAngleValues(tab);
        std::map<std::string, float> endJointValues = getEndAngleValues(tab);
        std::map<std::string, Eigen::Vector3f> startSegmentValues = getStartSegmentValues(tab);
        std::map<std::string, Eigen::Vector3f> endSegmentValues = getEndSegmentValues(tab);

        RemoteGui::ValueProxy<float> alpha = tab.getValue<float>("alphaBox");
        RemoteGui::ValueProxy<float> threshold = tab.getValue<float>("thresholdBox");
        RemoteGui::ValueProxy<float> height = tab.getValue<float>("heightBox");



        ARMARX_IMPORTANT << "submit button received." ;

        ARMARX_IMPORTANT << "number of files: " << numberOfFiles.get() << ", motionFiles size: " << motionFiles.size();


        ARMARX_IMPORTANT << "number of matches: " << numberOfBestMatches.get();

        if (numberOfFiles.get() > (int) motionFiles.size())
        {
            MMM::LegacyMotionReaderXMLPtr motionReader(new MMM::LegacyMotionReaderXML());

            std::string path = "/common/homes/students/hrp3/Downloads/2017-06-22";

            std::vector<std::string> paths = std::vector<std::string>();

            int loadedMotionsNum = 0;

            int i = 0;
            for (auto& entry : std::filesystem::directory_iterator(path))
            {
                if (i >= (int)motionFiles.size())
                {
                    if (hasEnding(entry.path(), "_mmm.xml") && loadedMotionsNum < numberOfFiles.get())
                    {
                        paths.push_back(entry.path());
                    }
                }
                i++;
            }

            int numberOfFilesValue = numberOfFiles.get();

            int endIndex = std::min(numberOfFilesValue, (int)paths.size());


            int lastIndex = motionFiles.size();
            motionFiles.resize(endIndex);

            std::atomic<int> numberOfLoadedFiles = 0;

            #pragma omp parallel for
            for (int i = lastIndex; i < endIndex; ++i)
            {
                std::vector<std::string> motions = motionReader->getMotionNames(paths[i]);
                motionFiles[i] = motionReader->loadMotion(paths[i], motions[0]);
                ++numberOfLoadedFiles;
                ARMARX_IMPORTANT << "loaded file " << numberOfLoadedFiles << " of " << endIndex;
            }

        }

        std::vector<MotionSlice> motionSlicesByDistance = std::vector<MotionSlice>();

        if (endSegmentValues.size() == 0 && endJointValues.size() == 0 && !previousGlobalCoordinatesInput)
        {
            motionSlicesByDistance = getBestMotionSlicesStartValuesOnly(motionFiles, startJointValues, startSegmentValues, alpha.get(), threshold.get() * height.get(), height.get());
        }
        else if (!previousGlobalCoordinatesInput)
        {
            motionSlicesByDistance = getBestMotionSlicesStartAndEndValues(motionFiles, startJointValues, startSegmentValues, endJointValues, endSegmentValues, alpha.get(), threshold.get() * height.get(), height.get());
        }
        else if (endSegmentValues.size() != 0)
        {
            motionSlicesByDistance = getBestMotionSlicesGlobalStartAndEndValues(motionFiles, startJointValues, startSegmentValues, endJointValues, endSegmentValues, alpha.get(), threshold.get() * height.get(), height.get());
        }
        else
        {
            ARMARX_IMPORTANT << "if global coordinates are used, start and end position need to be specified for at least one segment." ;
        }

        ARMARX_IMPORTANT << "motion map size: " << motionSlicesByDistance.size();
        while (!task->isStopped())
        {
            visualizeNFirstMotionSlices(motionSlicesByDistance, numberOfBestMatches.get(), startSegmentValues, endSegmentValues);
            tab.receiveUpdates();
            if (tab.getButton("cancelButton").clicked())
            {
                ARMARX_IMPORTANT << "task should be stopped now";
                task->stop();
            }
        }
    }
}

