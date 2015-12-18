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
#include <math.h>
#include <tagslist.h>
//LEMON
#include <lemon/list_graph.h>
#include <lemon/dijkstra.h>

#define FLOOR 2500
#define ROBOT_SIZE 400.f
#define ROBOT_RADIUS 200.f
#define MAX_ROBOT_ROTATION_SPEED 0.8
#define MAX_ADVANCE_SPEED 500

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void newAprilTag(const tagsList &tags);
	enum class State  { INIT, FINISH, SEARCH, PICK_NEW_POINT, GOTO_POINTS, VERIFY_POINT, IDLE, TRAVELLING};

public slots:
	void compute();

private:
	
	ListaMarcas* listaMarcas;
	State estado = State::INIT;
	InnerModel* inner;
	RoboCompLaser::TLaserData ldata;
	RoboCompDifferentialRobot::TBaseState bState;
	State pickNewPoint();
	State verifyPoint();
	State gotoPoints();
	State search();
	State travelling();

	//Lemon
	lemon::ListGraph graph;
	lemon::ListGraph::NodeMap<QVec> *map;
	lemon::ListGraph::EdgeMap <float> *edgeMap;
	lemon::ListGraph::NodeIt closestNode;
	lemon::Path<lemon::ListGraph> path;
	QVec qpos; //New random point
	
	QQueue<QVec> colaPuntos;
	lemon::ListGraph::NodeIt robotNode;
	
	OsgView *osgView;
	InnerModelViewer *innerViewer;
	
};

#endif

