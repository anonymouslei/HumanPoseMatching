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

#define BOOST_TEST_MODULE HRPExamples::ArmarXObjects::HumanPoseMatching

#define ARMARX_BOOST_TEST

#include <HRPExamples/Test.h>
#include <HRPExamples/components/HumanPoseMatching/HumanPoseMatching.h>

#include <iostream>

BOOST_AUTO_TEST_CASE(testExample)
{
    armarx::HumanPoseMatching instance;

    BOOST_CHECK_EQUAL(true, true);
}
