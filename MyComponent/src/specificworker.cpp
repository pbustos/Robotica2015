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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
 inner = new InnerModel("/home/robocomp/Software/robotica/Robotica2015/apartament.xml");
 //inner = new InnerModel("/home/robocomp/Escritorio/Robotica2015/apartament.xml");
 
 listaMarcas= new ListaMarcas(inner);
 
 lemon::ListGraph::Node robot = graph.addNode();
 map = new lemon::ListGraph::NodeMap<QVec>(graph);

  try
	{
		differentialrobot_proxy->getBaseState(bState);
		inner->updateTransformValues("robot", bState.x, 0, bState.z, 0, bState.alpha, 0);	//actualiza los valores del robot en el arbol de memoria
		map->set(robot, inner->transform("world","robot"));
		robotNode = lemon::ListGraph::NodeIt(graph,robot);
		
		qDebug() << __FUNCTION__<< "Robot inserted in the graph at " << inner->transform("world","robot");
	}
	catch(const Ice::Exception &ex){ std::cout << ex.what() << std::endl;}; 
	
  edgeMap = new lemon::ListGraph::EdgeMap<float>(graph);
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(500);
	return true;
}

void SpecificWorker::compute()
{   
    try
    {
		differentialrobot_proxy->getBaseState(bState);
		
		inner->updateTransformValues("robot", bState.x, 0, bState.z, 0, bState.alpha, 0);	//actualiza los valores del robot en el arbol de memoria
		ldata = laser_proxy->getLaserData();  //read laser data 

		switch( estado )
		{
			case State::INIT:
				estado = State::PICK_NEW_POINT;
				break;
			case State::SEARCH:
				break;
			case State::PICK_NEW_POINT:
				estado = pickNewPoint();
				break;
			case State::GOTO_POINTS:
				estado = gotoPoints();
				break;
			case State::VERIFY_POINT:
				estado = verifyPoint();
				break;	
			case State::TRAVELLING:
				estado = travelling();
				break;
			case State::FINISH:
				std::cout << "FINISH" << std::endl;
				sleep(1);
				estado = State::IDLE;
				break;
			case State::IDLE:
				break;
			} 
		}
    catch(const Ice::Exception e)
    {
      std::cout << e << std::endl;
    }
}

SpecificWorker::State SpecificWorker::pickNewPoint()
{
  try
  {
		//get a random state
		this->qpos = QVec::uniformVector(3, -FLOOR, FLOOR);
		qpos[1] = 0;  //to avoid y
		qDebug() << __FUNCTION__ << qpos;
		
		//obtain the closest point to the graph
		float dist = std::numeric_limits< float >::max(), d;		
		for (lemon::ListGraph::NodeIt n(graph); n != lemon::INVALID; ++n)
		{
			d = (qpos - map->operator[](n)).norm2();
	
			if( d < dist ) 
			{ 
				dist = d;
				closestNode = n;
			}	
		}
		qDebug() << __FUNCTION__ << "dist" << d << "node" << map->operator[](closestNode);
		
		//Search shortest path along graph from closest node to robot
		if( closestNode != robotNode)
		{
			float dd;
			
			bool reached = dijkstra(graph,*edgeMap).path(path).dist(dd).run(robotNode,closestNode);
		
			qDebug() << reached << d;
		
			for( lemon::PathNodeIt<lemon::Path<lemon::ListGraph> > pIt(graph, path); pIt != lemon::INVALID; ++pIt)
				qDebug() << map->operator[](pIt);
			
			qDebug() << "---------------";
		}	
	}
	catch(const Ice::Exception &ex)
  {
        std::cout << ex << std::endl;
  }
  return State::GOTO_POINTS;
}



SpecificWorker::State SpecificWorker::gotoPoints()
{
	//NavState state=trajectoryrobot2d_proxy->getState();
	qDebug() << __FUNCTION__ << "Path length" << path.length();
	
	if( path.length() == 0 ) 
	{
		qDebug() << __FUNCTION__ << "Empty path, we are finished here!";
		return State::VERIFY_POINT;
	}
	
	lemon::PathNodeIt<lemon::Path<lemon::ListGraph> > pIt(graph, path);
	
	try	
	{ 
		RoboCompTrajectoryRobot2D::NavState state =  trajectoryrobot2d_proxy->getState();	
		if( state.state == "WORKING" )
			return State::GOTO_POINTS;
		if( state.state == "FINISH" )
			return State::GOTO_POINTS;
		if( state.state == "IDLE" )
		{
			qDebug() << __FUNCTION__ << "Now travelling to ---" << map->operator[](++pIt);
			RoboCompTrajectoryRobot2D::TargetPose target; target.y=0;
			target.x = map->operator[](pIt).x(); target.z = map->operator[](pIt).z();
			trajectoryrobot2d_proxy->go(target);
			return State::TRAVELLING;
		}
	}
	catch(Ice::Exception &ex) {std::cout<<ex.what()<<std::endl;};
	
	qDebug() << "---------------";
	return State::GOTO_POINTS;
}

SpecificWorker::State SpecificWorker::travelling()
{
	try	
	{ 
		RoboCompTrajectoryRobot2D::NavState state =  trajectoryrobot2d_proxy->getState();	
		if( state.state == "WORKING" )
			return State::TRAVELLING;
		if( state.state == "FINISH" )
		{
			lemon::PathNodeIt<lemon::Path<lemon::ListGraph> > pIt(graph, path);
			robotNode = lemon::ListGraph::NodeIt(graph, lemon::ListGraph::Node(++pIt));
			path.eraseFront();
			return State::GOTO_POINTS;
		}
	}
	catch(Ice::Exception &ex) {std::cout<<ex.what()<<std::endl;};
	return State::TRAVELLING;
}


SpecificWorker::State SpecificWorker::verifyPoint()
{
	qDebug() << __FUNCTION__ ;
	
	float alfa = atan2(qpos.x(), qpos.z());
	
	if( alfa > ldata.front().angle )
	{
		try	{	differentialrobot_proxy->setSpeedBase(0, 0.4*fabs(alfa));	}
		catch(Ice::Exception &ex) {std::cout<<ex.what()<<std::endl;};	
		return State::VERIFY_POINT;
	}
	else if( alfa < ldata.back().angle ) 
	{
		try	{	differentialrobot_proxy->setSpeedBase(0, -0.4*fabs(alfa));	}
		catch(Ice::Exception &ex) {std::cout<<ex.what()<<std::endl;};	
		return State::VERIFY_POINT;
	}
	else 
	{
		try	{	differentialrobot_proxy->setSpeedBase(0,0);	}
		catch(Ice::Exception &ex) {std::cout<<ex.what()<<std::endl;};	
	}
	
	//Check if target is inside laser field. If not make qpos the furthest possible point
	QVec qposR = inner->transform("laser",qpos,"world").norm2();
	float dist = qposR.norm2();
	for(auto l: ldata)
		if(l.angle < alfa)	
		{
			if( l.dist < dist + ROBOT_RADIUS)
				qpos = qposR.normalize() * (l.dist-ROBOT_RADIUS);
			break;
		}

	//add new node to the graph
	lemon::ListGraph::Node newP = graph.addNode();
	map->set(newP, qpos);
	lemon::ListGraph::Edge newEdge = graph.addEdge(closestNode,newP);
	edgeMap->set( newEdge, (map->operator[](newP) - map->operator[](closestNode)).norm2());
	
	qDebug() << "Listing the graph";
	for (lemon::ListGraph::NodeIt n(graph); n != lemon::INVALID; ++n)
		qDebug() << map->operator[](n);
	qDebug() << "---------------";
	return State::PICK_NEW_POINT;	
}

////////////////////////////
/// ICEPeriod
////////////////////////////

void SpecificWorker::newAprilTag(const tagsList& tags)
{
}
 