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
 listaMarcas= new ListaMarcas(inner);
 
 lemon::ListDigraph::Node robot = graph.addNode();

  map = new lemon::ListDigraph::NodeMap<QVec>(graph);
	try
	{
		differentialrobot_proxy->getBaseState(bState);
		inner->updateTransformValues("robot", bState.x, 0, bState.z, 0, bState.alpha, 0);	//actualiza los valores del robot en el arbol de memoria
		map->set(robot, inner->transform("world","robot"));
		qDebug() << __FUNCTION__<< "Robot inserted in the graph at " << inner->transform("world","robot");
	}
	catch(const Ice::Exception &ex){ std::cout << ex.what() << std::endl;}; 
	
	arcMap = new lemon::ListDigraph::ArcMap<float>(graph);
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
						break;
					case State::SEARCH:
						break;
					case State::CONTROLLER:
						std::cout << "CONTROLLER" << std::endl;
						controller();
						break;
					case State::WAIT:
						break;
					case State::FINISH:
						std::cout << "FINISH" << std::endl;
					break;
					} 
			}
    catch(const Ice::Exception e)
    {
      std::cout << e << std::endl;
    }
}


void SpecificWorker::controller()
{
  try
  {
		NavState state=trajectoryrobot2d_proxy->getState();
    
    if(state.state == "IDLE")
    {
			if( colaPuntos.isEmpty() == false )
			{
				QVec head = colaPuntos.dequeue();
				RoboCompTrajectoryRobot2D::TargetPose t;
				t.x = head.x(); t.z = head.z();
      //trajectoryrobot2d_proxy->go(t);
			}
			
			//get a random state
			QVec qpos = QVec::uniformVector(2, -FLOOR, FLOOR);
			
			//obtain the closest point to the graph
			float dist = std::numeric_limits< float >::max(), d;
			lemon::ListDigraph::NodeIt closestNode;
			for (lemon::ListDigraph::NodeIt n(graph); n != lemon::INVALID; ++n)
			{
				d = (qpos-(*map)[n]).norm2();
				if( dist < d ) 
				{ 
					dist = d;
					closestNode = n;
				}	
			}
			//Check if closest point is where the robot is now
			if( closestNode == robotNode)
			{}
			else	//Search shortest path along graph from closest node to robot
			{
				//Dijkstra robotNode, closesNode
				lemon::Path<lemon::ListDigraph> p;
				//lemon::ListDigraph::DistMap d;
				float d;
				bool reached = dijkstra(graph,*arcMap).path(p).dist(d).run(robotNode,closestNode);
			}
			
    }
    else if(state.state == "FINISH")
    {
      qDebug() << __FUNCTION__ << "Controller has finished";
      return;
    }
  }
  catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
}


////////////////////////////
/// ICEPeriod
////////////////////////////

void SpecificWorker::newAprilTag(const tagsList& tags)
{
}
 