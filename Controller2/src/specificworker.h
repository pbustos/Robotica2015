/*
 *    Copyright (C) 2015 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
       \brief
       @author authorname
*/


#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <osgviewer/osgview.h>
#include <innermodel/innermodelviewer.h>
#include <innermodel/innermodeldraw.h>
#include "currenttarget.h"
#include "waypoints.h"

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
		
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	NavState getState();
	float goBackwards(const TargetPose &target);
	void stop();
	float goReferenced(const TargetPose &target, const float xRef, const float zRef, const float threshold);
	float changeTarget(const TargetPose &target);
	float go(const TargetPose &target);
	void mapBasedTarget(const NavigationParameterMap &parameters);
	

public slots:
	void compute(); 	

private:

	RoboCompTrajectoryRobot2D::NavState nState;
	InnerModel* inner;
	TLaserData ldata, ldataR;
	TBaseState bState;
	CurrentTarget cTarget;
	QGraphicsScene scene;
	
	void createSubTarget();
	void goToSubTarget();
	bool freeWay();
	void goToTarget();
	bool atTarget();
	void histogram();
	void stopRobot();
	void drawTarget(const QString &name, const QVec& target, const QString &color="#009900");
	void undrawTarget(const QString &name);
	void turn();
	RoboCompTrajectoryRobot2D::NavState toMiddleware();
	QTime elapsedTime;
	
	enum class State  {INIT, IDLE, WORKING, FINISH, TURN};
	State state = State::INIT;
	
	OsgView *osgView;
	InnerModelViewer *innerViewer;
	
	WayPoints road;
};

#endif
