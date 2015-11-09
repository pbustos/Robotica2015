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
  inner = new InnerModel("/home/robocomp/robocomp/files/innermodel/simpleworld.xml");
  state.state="IDLE";

	graphicsView->setScene(&scene);
	graphicsView->show();
	graphicsView->scale(3,3);
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{	
	timer.start(Period);

	return true;
}

void SpecificWorker::compute()
{
    
  try
  {
     differentialrobot_proxy->getBaseState(bState);
     ldata = laser_proxy->getLaserData();
     inner->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
     
     if( state.state == "WORKING")
     {
	if( heLlegado() )
	{ 
	  qDebug()<<"he llegado";
	  differentialrobot_proxy->setSpeedBase(0,0);
	  state.state = "FINISH";
	  sleep(2);
	   state.state = "IDLE";
	  return;
	}
      
       else if(hayCamino())
       {
	   goToTarget(); 
       }
       else if(cTarget.activeSub == true)
       {
	  goToSubTarget(); 
       }
       else
       {
	 createSubTarget();
       }
    }
  }
  catch(const Ice::Exception &e)
  {    std::cout << "Error reading from Camera" << e << std::endl;  }
	
	histogram();
}

void SpecificWorker::histogram()
{
	static QGraphicsPolygonItem *p;
	static QGraphicsLineItem *l, *sr, *sl, *safety;
	const float R = 400; //Robot radius
	const float SAFETY = 600;

	scene.removeItem(p);
	scene.removeItem(l);
	scene.removeItem(sr);
	scene.removeItem(sl);
	scene.removeItem(safety);
	
	//Search the first increasing step from the center to the right
	uint i,j;
	for(i=ldata.size()/2; i>0; i--)
	{
		if( (ldata[i].dist - ldata[i-1].dist) < -R )
		{
			uint k=i-2;
			while( (k >= 0) and (fabs( ldata[k].dist*sin(ldata[k].angle - ldata[i-1].angle)) < R ))
			{ k--; }
			i=k;
			break;
		}
	}
	for(j=ldata.size()/2; j<ldata.size()-1; j++)
	{
		if( (ldata[j].dist - ldata[j+1].dist) < -R )
		{
			uint k=j+2;
			while( (k < ldata.size()) and (fabs( ldata[k].dist*sin(ldata[k].angle - ldata[j+1].angle)) < R ))
			{ k++; }
			j=k;
			break;
		}
	}
	
	safety = scene.addLine(QLine(QPoint(0,-SAFETY/100),QPoint(ldata.size(),-SAFETY/100)), QPen(QColor(Qt::yellow)));
	sr = scene.addLine(QLine(QPoint(i,0),QPoint(i,-40)), QPen(QColor(Qt::blue)));
	sl = scene.addLine(QLine(QPoint(j,0),QPoint(j,-40)), QPen(QColor(Qt::magenta)));
	
	//DRAW		
	QPolygonF poly;
	int x=0;
	poly << QPointF(0, 0);
	
	for(auto d : ldata)
		poly << QPointF(++x, -d.dist/100); // << QPointF(x+5, d.dist) << QPointF(x+5, 0);
	poly << QPointF(x, 0);

	l = scene.addLine(QLine(QPoint(ldata.size()/2,0),QPoint(ldata.size()/2,-20)), QPen(QColor(Qt::red)));
  p = scene.addPolygon(poly, QPen(QColor(Qt::green)));
	
	scene.update();
	
	//select the best subtarget and return coordinates
}


bool SpecificWorker::heLlegado()
{
  QVec t = inner->transform("rgbd", cTarget.target, "world");
  //qDebug()<< cTarget.target;
  float d = t.norm2();
  //qDebug()<< "distancia: "<<d;
  if ( d < 400 ) 
    return true;
  else return false;
}

bool SpecificWorker::hayCamino()
{
  
  QVec t = inner->transform("rgbd", cTarget.target, "world");
  float d = t.norm2();
  float alpha =atan2(t.x(), t.z() );
  
  //qDebug() << d << alpha;
  for(uint i = 0; i<ldata.size(); i++)
  {
      if(ldata[i].angle < alpha)
      {
	if( ldata[i].dist < d)
	{
	  return false;
	}
	else
	{
	  cTarget.activeSub=false;
	  qDebug()<<"hay camino";
	  return true;
	}
      }
  }
  return false;
}

void SpecificWorker::goToTarget()
{
   qDebug()<<"andar";

    QVec t = inner->transform("rgbd", cTarget.target, "world");
    float alpha =atan2(t.x(), t.z());
    float r= 0.3*alpha;
    float d = 0.3*t.norm2();
    if( fabs(r) > 0.2) d = 0;
    if(d>300)d=300;
    differentialrobot_proxy->setSpeedBase(d,r);
}

void SpecificWorker::goToSubTarget()
{
    qDebug()<<  __FUNCTION__<<"ir subTarget";  
    QVec t = inner->transform("laser", cTarget.subTarget, "world");
    float alpha =atan2(t.x(), t.z());
    float r= 0.4*alpha;
    float d = t.norm2();
    qDebug()<<  __FUNCTION__<< "subtarget" << cTarget.subTarget;
   qDebug()<<  __FUNCTION__<< "subtarget"<< d << alpha << d << r;
   
    if(d<100)
    {
        cTarget.activeSub=false;

    //  qFatal("fary");
        differentialrobot_proxy->setSpeedBase(0,0);
      sleep(1);
      
    }else
    {
      if( fabs(r) > 0.2) d = 0;
      if(d>300)d=300;
      differentialrobot_proxy->setSpeedBase(d,r);
    }
    qDebug() <<  __FUNCTION__<< "subtarget"<< d << r;
   
}

void SpecificWorker::createSubTarget()
{
  
  qDebug() <<  __FUNCTION__ << "creando subTarget";
  uint i, j;
  QVec t;
  float dt;
  
//   if(cTarget.activeSub == false)
//     t = inner->transform("rgbd", cTarget.target, "world");
//   else
//   {
     t = inner->transform("rgbd", cTarget.target, "world");
 // }
  
  float d = t.norm2();
  float alpha =atan2(t.x(), t.z() );

  //Extremos
  /*if( alpha > ldata[0].angle ) 
  {
    
    cTarget.subTarget = QVec::vec3(ldata[0].dist *sin(ldata[0].angle),0, ldata[0].dist *cos(ldata[0].angle)-400);
    cTarget.activeSub=true;
    return;
  }
 
  if( alpha < ldata[ldata.size()-1].angle ) 
  {
    int r = ldata.size()-1;
    cTarget.subTarget = QVec::vec3(ldata[r].dist *sin(ldata[r].angle),0, ldata[r].dist *cos(ldata[r].angle)-400);  
    cTarget.activeSub=true;
    return;
  }
  */  
    
    
  for(i = 5; i<ldata.size()-5; i++)
  {
      if(ldata[i].angle < alpha)
      {
	if(d>ldata[i].dist)
	{
	  dt=ldata[i].dist;
	 break;
	}
      } 
  }
  qDebug()<<  __FUNCTION__<<i;
  qDebug()<<  __FUNCTION__<<ldata[i].dist<<ldata[i].angle;
  
  for(j = i;j<ldata.size()-5;j++)
  {
      qDebug()<<  __FUNCTION__<<dt<< dt+(dt*0.2) <<ldata[j].dist << ldata[j].angle;
    
      if(ldata[j].dist> (dt+(dt*0.2)) and ldata[j].angle < 0)
      {
	cTarget.subTarget=inner->transform("world", QVec::vec3(ldata[j].dist *sin(ldata[j].angle)-2000,0, ldata[j].dist *cos(ldata[j].angle)), "laser");
	cTarget.activeSub=true;
	break;
      }
  }
  qDebug()<<  __FUNCTION__<< "Subtargeet" << QVec::vec3(ldata[j].dist *sin(ldata[j].angle),0, ldata[j].dist *cos(ldata[j].angle));
  
}



float SpecificWorker::go(const TargetPose &target)
{
 qDebug()<<"GO";
 //primeraVez=true;
 cTarget.target = QVec::vec3(target.x, target.y, target.z);
 cTarget.activeT = true;
 state.state = "WORKING";
}

NavState SpecificWorker::getState()
{
  return state;
}



void SpecificWorker::stop()
{

}