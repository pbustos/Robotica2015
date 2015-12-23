/*
    <one line to give the library's name and an idea of what it does.>
    Copyright (C) 2013  pbustos <email>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/


#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <QtCore>
//#include "pointstoroad.h"
#include <OmniRobot.h>
#include "waypoints.h"
#include <innermodel/innermodel.h>
#include <Laser.h>

class LineFollower
{
	public:
		//LineFollower(InnerModel *innerModel, const RoboCompLaser::TLaserData &laserData, int delay); //in secs
		LineFollower(){ delay = 1;};
		~LineFollower();
		void initialize(InnerModel *innerModel, const RoboCompLaser::TLaserData &laserData, int delay);
		bool update(InnerModel *innerModel, RoboCompLaser::TLaserData &laserData, RoboCompOmniRobot::OmniRobotPrx omnirobot_proxy, WayPoints &road);
		void stopTheRobot(RoboCompOmniRobot::OmniRobotPrx differentialrobot_proxy);

	private:
		QTime time;
		int delay;
		bool avoidanceControl(InnerModel* innerModel, const RoboCompLaser::TLaserData& laserData, float& vadvance, float& vrot);
		std::vector<float> computeRobotOffsets(InnerModel *innerModel, const RoboCompLaser::TLaserData &laserData);
		std::vector<float> baseOffsets;
		
		float exponentialFunction(float value, float xValue, float yValue, float min = 0.f);

};

#endif // CONTROLLER_H
