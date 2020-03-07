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
 * @package    HRPExamples::application::HumanPoseMatching
 * @author     Simon Ottenhaus ( simon dot ottenhaus at kit dot edu )
 * @date       2020
 * @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
 *             GNU General Public License
 */

#include <HRPGroup3/components/HumanPoseMatching/HumanPoseMatching.h>

#include <ArmarXCore/core/application/Application.h>
#include <ArmarXCore/core/Component.h>
#include <ArmarXCore/core/logging/Logging.h>

int main(int argc, char* argv[])
{
    return armarx::runSimpleComponentApp < armarx::HumanPoseMatching > (argc, argv, "HumanPoseMatching");
}
