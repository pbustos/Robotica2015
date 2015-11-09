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
 inner = new InnerModel("/home/salabeta/robocomp/files/innermodel/simpleworld.xml");
 listaMarcas= new ListaMarcas(inner);
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
    TBaseState bState;

    try
    {
      differentialrobot_proxy->getBaseState(bState);
      inner->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);	//actualiza los valores del robot en el arbol de memoria
    }
    catch(const Ice::Exception e)
    {
      std::cout << e << std::endl;
    }
  
    ldata = laser_proxy->getLaserData();  //read laser data 

    switch( estado )
    {
      case State::INIT:
	std::cout << "INIT" << std::endl;
	estado = State::SEARCH;
	break;
      case State::SEARCH:
	std::cout << "SEARCH" << std::endl;
	searchMark(listaMarcas->getInitMark());
	break;
      case State::CONTROLLER:
	std::cout << "CONTROLLER" << std::endl;
	controller();
	break;
      /*case State::NAVEGATE:
	std::cout << "NAVEGATE" << std::endl;
	  navegate();
	break;
      case State::WALL:
	std::cout << "WALL" << std::endl;
	  wall();
	break;*/
      case State::WAIT:
	std::cout << "WAIT" << std::endl;
	  wait();
	break;
      case State::FINISH:
	std::cout << "FINISH" << std::endl;
	
	break;
    } 
}

void SpecificWorker::searchMark(int initMark)
{
    static bool firstTime=true;
    
    if(listaMarcas->exists(initMark))
    {
      try
      {
	differentialrobot_proxy->setSpeedBase(0,0);
      }
      catch(const Ice::Exception e)
      {
	std::cout << e << std::endl;
      }
      estado = State::CONTROLLER;
      firstTime=true;
      return;
    }
    
    if(firstTime)
    {
      try
      {
	differentialrobot_proxy->setSpeedBase(0,0.5);
      }
      catch(const Ice::Exception e)
      {
	std::cout << e << std::endl;
      }
      firstTime=false;
    }

}

void SpecificWorker::wait()
{
    static bool primeraVez=true;
    static QTime reloj;
    int initMark=listaMarcas->getInitMark();
    
    if(primeraVez)
    {
      reloj = QTime::currentTime();
      primeraVez=false;
      int newState = (initMark + 1) % 4;
      listaMarcas->setInitMark(newState);
      listaMarcas->setInMemory(false);
    }
    
    if(reloj.elapsed() > 3000)
    {
      estado = State::SEARCH;
      primeraVez=true;
      return;
    } 
}

void SpecificWorker::navegate()
{
    const int offset = 20;
    int initMark=listaMarcas->getInitMark();
    float distance= listaMarcas->distance(initMark);
    
    if(listaMarcas->exists(initMark))
    {
      std::cout << "Existe la marca::" << initMark << std::endl;
      if(distance<400)
      {	
	differentialrobot_proxy->setSpeedBase(0,0);

	estado = State::WAIT;
	return;
      }
    }
    else
    {
      estado = State::SEARCH;
      return;
    }
    

    try
    {		
        std::sort( ldata.begin()+offset, ldata.end()-offset, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.
	
	float dist=(ldata.data()+offset)->dist;
	
	//Si encuentra un obstaculo
	if(dist <= 450)
	{
	    estado = State::WALL;
	    return;
	}  
	else
	{
	  float tx= listaMarcas->get(initMark).tx;
	  float tz= listaMarcas->get(initMark).tz;
	  float r= atan2(tx, tz);
	  differentialrobot_proxy->setSpeedBase(150, 0.4*r);
	}
	
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
}


void SpecificWorker::wall()
{
    RoboCompLaser::TLaserData ldataCopy = ldata;
    const int offset = 30;
    float rot=-0.3;
    
    std::sort( ldataCopy.begin()+offset, ldataCopy.end()-offset, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.
    
    //Si no hay obstaculo
    if((ldataCopy.data() + offset )->dist > 450)
    {
      estado = State::NAVEGATE;
      return;
    }
    
    //gira a izq o der para no traspasar el obstaculo o la pared
    if((ldataCopy.data() + offset )->angle < 0)
      rot=0.3;
    else
    {
      if((ldataCopy.data() + offset )->angle > 0)
	rot=-0.3;
    }
    
     differentialrobot_proxy->setSpeedBase(40, rot);
     usleep(1000000);
}

void SpecificWorker::controller()
{
  try
  {
    NavState state=controller_proxy->getState();
    //qDe
    if(state.state == "IDLE")
    {
      ListaMarcas::Marca m=listaMarcas->get(listaMarcas->getInitMark());
      QVec w = inner -> transform("world",QVec::vec3(m.tx,m.ty,m.tz),"rgbd");
      TargetPose t={w.x(), w.y(), w.z()};
      controller_proxy->go(t);
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
 