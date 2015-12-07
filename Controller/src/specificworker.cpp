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
  //inner = new InnerModel("/home/robocomp/robocomp/files/innermodel/simpleworld.xml");
	QString innerFile = "/home/robocomp/Software/robotica/Robotica2015/apartment.xml";
	if( QFile::exists(innerFile) )
		inner = new InnerModel(innerFile.toStdString());
	else	
		qFatal("InnerModel file not found");
			
	graphicsView->setScene(&scene);
	graphicsView->show();
	graphicsView->scale(3,3);
	
	//Innermodelviewer
	osgView = new OsgView(this);
	osgGA::TrackballManipulator *tb = new osgGA::TrackballManipulator;
	osg::Vec3d eye(osg::Vec3(4000.,4000.,-1000.));
	osg::Vec3d center(osg::Vec3(0.,0.,-0.));
	osg::Vec3d up(osg::Vec3(0.,1.,0.));
	tb->setHomePosition(eye, center, up, true);
	tb->setByMatrix(osg::Matrixf::lookAt(eye,center,up));
 	osgView->setCameraManipulator(tb);
	innerViewer = new InnerModelViewer(inner, "root", osgView->getRootGroup(), true);
	show();
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{	
	//timer.start(Period);
	timer.start(100);	
	return true;
}

void SpecificWorker::compute()
{
  try
  {
     differentialrobot_proxy->getBaseState(bState);
     ldata = laser_proxy->getLaserData();
     inner->updateTransformValues("robot", bState.x, 0, bState.z, 0, bState.alpha, 0);
	
		 switch( state )
		 {
			 case State::INIT:
				 state = State::IDLE;
				 break;
				 
			 case State::IDLE:
				 break;
				 
			 case State::WORKING:
				 if( atTarget() )
				 { 
						qDebug()<< __FUNCTION__<< "Arrived to target" << cTarget.target;
						stopRobot();
						state = State::FINISH;
					}
					else if(freeWay())
					{
						goToTarget(); 
					}
 					else if(cTarget.isActiveSubtarget == true)
 					{
 						goToSubTarget(); 
 					}
 					else
 					{
 						createSubTarget();
 					}
 					break;
					
			 case State::TURN:
						turn();
				 break;
				 
			 case State::FINISH:
						sleep(2);
						undrawTarget("target");
						state = State::IDLE;
				 break;
	  	}
	}
  catch(const Ice::Exception &e)
  {    std::cout << "Error reading from Camera" << e << std::endl;  }
	
	//histogram();
	innerViewer->update();
	osgView->autoResize();
	osgView->frame();
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
	int i,j;
	for(i=(int)ldata.size()/2; i>1; i--)
	{
		if( (ldata[i].dist - ldata[i-1].dist) < -R )
		{
			int k=i-2;
			while( (k > 0) and (fabs( ldata[k].dist*sin(ldata[k].angle - ldata[i-1].angle)) < R ))
			{ k--; }
			i=k;
			break;
		}
	}
	for(j=(int)ldata.size()/2; j<(int)ldata.size()-2; j++)
	{
		if( (ldata[j].dist - ldata[j+1].dist) < -R )
		{
			int k=j+2;
			while( (k < (int)ldata.size()-1) and (fabs( ldata[k].dist*sin(ldata[k].angle - ldata[j+1].angle)) < R ))
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


bool SpecificWorker::atTarget()
{
  QVec t = inner->transform("robot", cTarget.target, "world");
  float d = t.norm2();
	static float dAnt = d;
	
	qDebug()<< "---------------------------";
  qDebug()<< __FUNCTION__<< "target " << t << "distancia: " << d;
  
	if ( d < 100 or (d<500 and (d-dAnt)>0)
		
	)
	{
		qDebug() << __FUNCTION__<< " = true";
		dAnt = std::numeric_limits<float>::max();
    return true;
	}
  else 
	{
		qDebug() << __FUNCTION__<< " = false";
		dAnt=d;
		return false;
	}
}

bool SpecificWorker::freeWay()
{
	QVec t = inner->transform("robot", cTarget.target, "world");
  float d = t.norm2();
  float alpha =atan2(t.x(), t.z() );
	
	qDebug()<< "---------------------------";
  qDebug()<< __FUNCTION__<< "target " << t << "distancia: " << d;
  
	for(uint i = 0; i<ldata.size(); i++)
  {
		if(ldata[i].angle <= alpha)
    {
			if( ldata[i].dist < 4000 and ldata[i].dist < d)
			{	
				qDebug() << __FUNCTION__<< " = false at d=" << d << "alpha = " << alpha << "laser=" << ldata[i].angle << ldata[i].dist;
				return false;
			}
			else
			{
				cTarget.isActiveSubtarget = false;
				qDebug()<<__FUNCTION__<< "Hay camino libre al target" << ldata[i].dist << d;
				return true;
			}
     }
  }
  qDebug() << "The target is behind. I should turn around";
	state = State::TURN;
  return false;
}

void SpecificWorker::goToTarget()
{
	QVec t = inner->transform("robot", cTarget.target, "world");
 
	float distToTarget = t.norm2();
	float alpha =atan2(t.x(), t.z());
	if( distToTarget < 250 ) 
 		alpha = 0;
	
  float r = alpha;
  
// 	
  float d = 0.3*distToTarget;
	/*if( fabs(r) > 0.2 and distToTarget> 400) d = 0;
 */ 
	if(d>300) d=300;
	
	qDebug()<< "---------------------------";
	qDebug() << __FUNCTION__ << t << "rot" << r << "adv" << d;
	
	try
	{		
		differentialrobot_proxy->setSpeedBase(d,r); 
	}
	catch(Ice::Exception &ex) {std::cout<<ex.what()<<std::endl;};
 }

 /**
	* @brief Turns the robot until the target is in front of it
	* 
	* @return void
	*/
 void SpecificWorker::turn()
{
	float alpha;
	QVec t = inner->transform("robot", cTarget.target, "world");
	alpha =atan2(t.x(), t.z() );
	
	if( alpha <= ldata.front().angle and alpha >= ldata. back().angle)
	{
		stopRobot();
		state = State::WORKING;
	}
	else
	{
		if( alpha > ldata.front().angle )  // turn right
			try{ differentialrobot_proxy->setSpeedBase(0, fabs(alpha));}
			catch(Ice::Exception &ex) {std::cout<<ex.what()<<std::endl;}
		else															// turn left
			try{ differentialrobot_proxy->setSpeedBase(0, -fabs(alpha));}
			catch(Ice::Exception &ex) {std::cout<<ex.what()<<std::endl;};
	}
}


void SpecificWorker::goToSubTarget()
{
  qDebug()<<  __FUNCTION__<<"ir subTarget";  
  QVec t = inner->transform("robot", cTarget.subTarget, "world");
  float alpha =atan2(t.x(), t.z());
	
  float r = 0.4*alpha;
  float d = 0.4*t.norm2();
	
  qDebug()<<  __FUNCTION__<< "subtarget" << cTarget.subTarget;
  qDebug()<<  __FUNCTION__<< "subtarget"<< d << alpha << d << r;
   
    if(d<100)  //Subtarget achieved
    {
        cTarget.isActiveSubtarget = false;
				differentialrobot_proxy->setSpeedBase(0,0);
				qDebug()<<  __FUNCTION__<<"subTarget alcanzado";  	
				sleep(1);
				undrawTarget("subTarget");
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
  qDebug() <<  __FUNCTION__;
  
  QVec t = inner->transform("laser", cTarget.target, "world");
  float d = t.norm2();
  float alpha =atan2(t.x(), t.z() );
	const float R = 400.f;
    
	//Search the first increasing step from the center to the right
	int i,j;
	for(i=(int)ldata.size()/2; i>1; i--)
	{
		if( (ldata[i].dist - ldata[i-1].dist) < -R )
		{
			int k=i-1;
			while( (k > 0) and (fabs( ldata[i].dist*sin(ldata[k].angle - ldata[i].angle)) < R*1.2 ))
			{ 
				qDebug() << "arco" << fabs( ldata[k].dist*sin(ldata[k].angle - ldata[i-1].angle));
				k--; }
			i=k;
			break;
		}
	}
	//search now the left side
	for(j=(int)ldata.size()/2; j<(int)ldata.size()-2; j++)
	{
		if( (ldata[j].dist - ldata[j+1].dist) < -R )
		{
			int k=j+1;
			while( (k < (int)ldata.size()-1) and (fabs( ldata[j].dist*sin(ldata[k].angle - ldata[j].angle)) < R*1.2 ))
			{ k++; }
			j=k;
			break;
		}
	}
 
	//Select i or j, the closest one
	if( fabs(ldata[i].angle - alpha) < fabs(ldata[j].angle - alpha) )
		cTarget.subTarget=inner->laserTo ("world", "laser", ldata[i].dist-2000,ldata[i].angle);
	else
		cTarget.subTarget=inner->laserTo ("world", "laser", ldata[j].dist-2000,ldata[j].angle);
		
	drawTarget("subTarget",cTarget.subTarget,"#000099");
	cTarget.isActiveSubtarget = true;
  qDebug()<<  __FUNCTION__<< "Subtarget" << cTarget.subTarget;
  
}

////////////////////////
/// Utilities
///////////////////////


void SpecificWorker::stopRobot()
{
	try
	{
			differentialrobot_proxy->setSpeedBase(0,0);
	}
	catch(Ice::Exception &ex) {std::cout<<ex.what()<<std::endl;};
}

void SpecificWorker::drawTarget(const QString &name, const QVec &target, const QString &color)
{
  InnerModelDraw::addPlane_ignoreExisting(innerViewer, name, "world", QVec::vec3(target(0), 100, target(2)), QVec::vec3(1,0,0), "#009900", QVec::vec3(100,100,100));
}
void SpecificWorker::undrawTarget(const QString& name)
{
	InnerModelDraw::removeNode(innerViewer, name);
}

RoboCompTrajectoryRobot2D::NavState SpecificWorker::toMiddleware()
		{
			QMutexLocker l(mutex);
			RoboCompTrajectoryRobot2D::NavState n;
			switch( state )
			{
				case State::INIT:
					n.state = "INIT";
					break;
				case State::WORKING:
					n.state = "WORKING";
					break;
				case State::IDLE:
					n.state = "IDLE";
					break;
				case State::FINISH:
					n.state = "FINISH";
					break;
				case State::TURN:
					n.state = "TURN";
					break;
			}
			n.elapsedTime = elapsedTime.elapsed();
			n.estimatedTime = inner->transform("robot", cTarget.subTarget, "world").norm2() / 250; //mean speed
			n.x = bState.x;
			n.z = bState.z;
			n.ang = bState.alpha;
			n.advV = bState.advV;
			n.rotV = bState.rotV;
			n.distanceToTarget = inner->transform("robot", cTarget.subTarget, "world").norm2();
			return n;
		};
/////////////////////////////////////////7
///////  Interfaz implementation
/////////////////////////////////////////777


NavState SpecificWorker::getState()
{
	
  return toMiddleware();
}


float SpecificWorker::goBackwards(const TargetPose &target)
{
	return 0;
}


float SpecificWorker::goReferenced(const TargetPose &target, const float xRef, const float zRef, const float threshold)
{
	return 0;
}

float SpecificWorker::changeTarget(const TargetPose &target)
{
	return 0;
}

float SpecificWorker::go(const TargetPose &target)
{
	cTarget.target = QVec::vec3(target.x, target.y, target.z);
	cTarget.isActiveTarget = true;
	elapsedTime = QTime::currentTime();
	qDebug()<< __FUNCTION__ <<"ICE::GO" << cTarget.target;
	
	state = State::WORKING;
	drawTarget("target",cTarget.target);
	return (inner->transform("world","robot") - cTarget.target).norm2();	//Distance to target in mm
}

void SpecificWorker::mapBasedTarget(const NavigationParameterMap &parameters)
{

}


void SpecificWorker::stop()
{
	cTarget.isActiveTarget = true;
	stopRobot();
}