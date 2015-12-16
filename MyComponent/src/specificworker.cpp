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
						break;
					case State::SEARCH:
						break;
					case State::PICKNEWPOINT:
						std::cout << "CONTROLLER" << std::endl;
						controller();
						break;
					case State::GOTOPOINTS:
						break;
					case State::VERIFYPOINT:
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
			QVec qpos = QVec::uniformVector(3, -FLOOR, FLOOR);
			qpos[1] = 0;
			
			//obtain the closest point to the graph
			float dist = std::numeric_limits< float >::max(), d;
			lemon::ListGraph::NodeIt closestNode;
			
			for (lemon::ListGraph::NodeIt n(graph); n != lemon::INVALID; ++n)
			{
				d = (qpos-(*map)[n]).norm2();
				if( dist < d ) 
				{ 
					dist = d;
					closestNode = n;
				}	
			}
		
			//Search shortest path along graph from closest node to robot
			if( closestNode != robotNode)
			{
				lemon::Path<lemon::ListGraph> p;
				float dd;
				
				bool reached = dijkstra(graph,*edgeMap).path(p).dist(dd).run(robotNode,closestNode);
				qDebug() << reached << d;
				
	// 			while (!dijkstra.emptyQueue()) 
	// 			{
	// 				ListGraph::Node n = dijkstra.processNextNode();
	// 				cout << g.id(n) << ' ' << dijkstra.dist(g) << endl;
	// 			}

				for( lemon::PathNodeIt<lemon::Path<lemon::ListGraph> > pIt(graph, p); pIt != lemon::INVALID; ++pIt)
				{
					qDebug() << map->operator[](pIt);
					colaPuntos.enqueue(map->operator[](pIt));
					state = State::GOTOPOINTS;
					return;
				}			
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
 