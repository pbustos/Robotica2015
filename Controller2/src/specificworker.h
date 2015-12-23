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
#include "linefollower.h"

#define ROBOT_SIZE 400.f
#define ROBOT_RADIUS 200.f
#define MAX_ROBOT_ROTATION_SPEED 0.8
#define MAX_ADVANCE_SPEED 700
#define MAX_ANGLE_ERROR 0.5


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
		
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	//ICE
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
	RoboCompOmniRobot::TBaseState bState;
	CurrentTarget cTarget;
	QGraphicsScene scene;
	
	enum class State  {INIT, IDLE, SET_NEW_TARGET,  FOLLOW_PATH, FINISH, FINAL_TURN, TURN};
	State state = State::INIT;
	
	State setNewTarget();
	State followPath();
	bool atTarget();
	State turn();
	State finalTurn();
	bool plan();
	bool controller();

	void freeWay(const QVec& tg);
	void histogram();
	void stopRobot();
	void stopRobotAndFinish();
	void drawTarget(const QString &name, const QVec& target, const QString &color="#009900");
	void undrawTarget(const QString &name);
	RoboCompTrajectoryRobot2D::NavState toMiddleware();
	QTime elapsedTime;
	
	OsgView *osgView;
	InnerModelViewer *innerViewer;
	
	WayPoints path;
	
	LineFollower linefollower;
};

#endif

