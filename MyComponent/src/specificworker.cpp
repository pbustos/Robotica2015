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
 
 
 lemon::ListGraph::Node u = graph.addNode();
 lemon::ListGraph::Node v = graph.addNode();
 lemon::ListGraph::Edge  a = graph.addEdge(u, v);
 
 map = new lemon::ListGraph::NodeMap<QVec>(graph);
 
 //(*map)[u]=2;
 map->set(u,QVec::vec3(3,4,5));
 
 cout << "Hello World! This is LEMON library here." << endl;
 cout << "We have a directed graph with " << countNodes(graph) << " nodes "  << "and " << countArcs(graph) << " arc." << endl;
 qDebug() << "Map " << map->operator[](u);
 
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
    //qDe
    if(state.state == "IDLE")
    {
      ListaMarcas::Marca m=listaMarcas->get(listaMarcas->getInitMark());
      QVec w = inner -> transform("world",QVec::vec3(m.tx,m.ty,m.tz),"rgbd");
      TargetPose t={w.x(), w.y(), w.z()};
      trajectoryrobot2d_proxy->go(t);
    }
    else if(state.state == "FINISH")
    {
      
      estado = State::WAIT;
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
   for( auto t: tags)
   {
     listaMarcas->add(t);
    
   }
}
 