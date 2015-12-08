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
	QString innerFile = "/home/robocomp/Software/robotica/Robotica2015/apartament.xml";
	if( QFile::exists(innerFile) )
		inner = new InnerModel(innerFile.toStdString());
	else	
		qFatal("InnerModel file not found");
	
	InnerModelLaser *lm;
	if ((lm = dynamic_cast<InnerModelLaser *>(inner->getNode("laser"))) != NULL)
		LASER_MAX = lm->max;
	else
		qFatal("Fatal errro. Laser element not found in XML file");
	
	graphicsView->setScene(&scene);
	graphicsView->show();
	graphicsView->scale(3,3);
	
// 	//fake robot
// 	inner->newTransform("vbox", "static", inner->getNode("robot"), 0, 100, 0); 
// 	inner->newPlane("vboxPlane", inner->getNode("vbox"), "#ff0000", 400,50,400, 0, 0,0,1, 0, 0, -100 );
 			
	//Innermodelviewer
	osgView = new OsgView( frame );
	osgGA::TrackballManipulator *tb = new osgGA::TrackballManipulator;
	osg::Vec3d eye(osg::Vec3(4000.,4000.,-1000.));
	osg::Vec3d center(osg::Vec3(0.,0.,-0.));
	osg::Vec3d up(osg::Vec3(0.,-1.,0.));
	tb->setHomePosition(eye, center, up, true);
	osg::Matrixf m;
	//m.set( 0.998955, 0.043652, -0.0135747, 0, -0.0158273, 0.0516788, -0.998538, 0, -0.0428867, 0.997709, 0.0523157, 0, 4185.27, 7313.77, 4883.92, 1); 
	tb->setByMatrix(osg::Matrixf::lookAt(eye,center,up));
	tb->setByMatrix(m);
 	osgView->setCameraManipulator(tb);
	innerViewer = new InnerModelViewer(inner, "root", osgView->getRootGroup(), true);
	show();
	
	connect( pushButton, SIGNAL(clicked()), this, SLOT(stopSlot()));
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
	static std::string stateAnt = "";
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
						cTarget.isActiveSubtarget=false;
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
	
	histogram();
	innerViewer->update();
	osgView->autoResize();
	osgView->frame();
	RoboCompTrajectoryRobot2D::NavState ns = toMiddleware();
	if( stateAnt != ns.state )
		plainTextEdit->appendPlainText(QString::fromStdString(ns.state));
	stateAnt = ns.state;
}

void SpecificWorker::histogram()
{
	static QGraphicsPolygonItem *p;
	static QGraphicsLineItem *l, *sr, *sl, *safety, *st=nullptr;
	const float R = 400; //Robot radius
	const float SAFETY = 600;

	scene.removeItem(p);
	scene.removeItem(l);
	scene.removeItem(sr);
	scene.removeItem(sl);
	scene.removeItem(safety);
	//if(st != nullptr) scene.removeItem(st);
	
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
// 	if(cTarget.isActiveTarget)
// 	{
// 		QVec t = inner->transform("robot",cTarget.target,"world");
// 		for(uint i=0; i<ldata.size(); i++)
// 			if( ldata[i].angle <= atan2(t.x(),t.z()))
// 			{
// 				st = scene.addLine(QLine(QPoint(i,0),QPoint(i,-50)), QPen(Qt::cyan));
// 				break;
// 			}
// 	}
	
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
	
}


bool SpecificWorker::atTarget()
{
  QVec t = inner->transform("robot", cTarget.target, "world");
  float d = t.norm2();
	static float dAnt = d;
	
	qDebug()<< "---------------------------";
  qDebug()<< __FUNCTION__<< "target " << t << "distancia: " << d;
  
	if ( d < 100 or (d<500 and (d-dAnt)>0))
	{
		qDebug() << __FUNCTION__<< " = true";
		dAnt = std::numeric_limits<float>::max();
		undrawTarget("subTarget");
    return true;
	}
  else 
	{
		qDebug() << __FUNCTION__<< " = false";
		dAnt=d;
		return false;
	}
}

/**
 * @brief Checks if there is a free way from robot position to target
 * 
 * @return bool
 */
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
			if( thereIsATubeToTarget(i, t, alpha) == true )	
			{	
				cTarget.isActiveSubtarget = false;
				qDebug()<<__FUNCTION__<< "Hay camino libre al target" << ldata[i].dist << d;
				undrawTarget("subTarget");
				return true;
			}
			else
			{
				qDebug() << __FUNCTION__<< " = false at d=" << d << "alpha = " << alpha << "laser=" << ldata[i].angle << ldata[i].dist;
				return false;
			}
     }
  }
  if(cTarget.isActiveSubtarget == false)
	{
		qDebug() << "The target is behind. I should turn around";
		state = State::TURN;
	}
  return false;
}

/**
 * @brief Check is a robot-box fits in the laser field along the current laser line
 * 
 * @return void
 */
bool SpecificWorker::thereIsATubeToTarget(int i, const QVec &targetInRobot, float alpha )
{
	const int ROBOT_RADIUS = 200;  //READ FROM XML
	QList<QVec> lPoints;
	
	//points on the corners of thw square
	lPoints.append( QVec::vec3(ROBOT_RADIUS,0,ROBOT_RADIUS)); 
	lPoints.append( QVec::vec3(ROBOT_RADIUS,0,-ROBOT_RADIUS)); 
	lPoints.append( QVec::vec3(-ROBOT_RADIUS,0,-ROBOT_RADIUS));
	lPoints.append( QVec::vec3(-ROBOT_RADIUS,0,ROBOT_RADIUS));
	
	//points on the sides
	lPoints.append( QVec::vec3(ROBOT_RADIUS,0,0));
	lPoints.append( QVec::vec3(0,0,-ROBOT_RADIUS));
	lPoints.append( QVec::vec3(-ROBOT_RADIUS,0,0));
	lPoints.append( QVec::vec3(0,0, ROBOT_RADIUS));
	
	
	float dist = targetInRobot.norm2();
	float step = ceil(dist/ (ROBOT_RADIUS/3));
	QVec tNorm = targetInRobot.normalize();
	
	QVec r;
	inner->updateTransformValues ("vbox",0, 0, 0, 0, 0, 0, "robot");	
	for(float landa=400; landa<=dist; landa+=step)
	{
		r = tNorm * landa;
		inner->updateTransformValues ("vbox",r.x(), r.y(), r.z(), 0, alpha, 0, "robot");	
		innerViewer->update();
		
		foreach(QVec p, lPoints)
			if( inLaserField( inner->transform("robot", p, "vbox")) == false )
			{
				return false;
			}
	}
	return true;
}

/**
 * @brief Checks is pointInRobot is within the laser field
 * 
 * @param pointInRobot point to check in robots coordinate frame
 * @param laserIndex closes index of laser array to the target direction
 * @return bool
 */
bool SpecificWorker::inLaserField(const QVec& pointInRobot)
{
	float d = pointInRobot.norm2();
	float alpha = atan2(pointInRobot.x(), pointInRobot.z());
	
	for(auto ld : ldata)
		if(ld.angle <= alpha)
		{
			if( ld.dist < LASER_MAX and ld.dist >= d)
				return true;
			else
				return false;
		}
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
	
	if( alpha < ldata[20].angle and alpha > ldata[ldata.size()-20].angle)
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
	
//   qDebug()<<  __FUNCTION__<< "subtarget" << cTarget.subTarget;
//   qDebug()<<  __FUNCTION__<< "subtarget"<< d << alpha << d << r;
   
    if(d<100)  //Subtarget achieved
    {
        cTarget.isActiveSubtarget = false;
				differentialrobot_proxy->setSpeedBase(0,0);
				qDebug()<<  __FUNCTION__<<"subTarget alcanzado";  	
				undrawTarget("subTarget");
    }else
    {
      if( fabs(r) > 0.2) d = 0;
      if(d>300)d=300;
      differentialrobot_proxy->setSpeedBase(d,r);
    }
    qDebug() <<  __FUNCTION__<< "subtarget"<< d << r;
}

/// ATENCIÓN, este método tiene que ser mejorado. Tal y como está puede seleccionar subtargets al otro lado de una pared.
/// habría que analizar mejor el histograma laser para buscar "huecos" cercanos al target, no solo escalones.

void SpecificWorker::createSubTarget()
{
  qDebug() <<  __FUNCTION__;
  
  QVec t = inner->transform("laser", cTarget.target, "world");
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
		
	drawTarget("subTarget",cTarget.subTarget,"#ff0000");
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
		cTarget.isActiveTarget = false;
		cTarget.isActiveSubtarget = false;
		state = State::IDLE;
		
// 		osgGA::CameraManipulator *cm = osgView->getCameraManipulator();
// 		std::cout << cm->getMatrix() << std::endl;
	}
	catch(Ice::Exception &ex) {std::cout<<ex.what()<<std::endl;};
}

void SpecificWorker::drawTarget(const QString &name, const QVec &target, const QString &color)
{
  InnerModelDraw::addPlane_ignoreExisting(innerViewer, name, "world", QVec::vec3(target(0), 100, target(2)), QVec::vec3(1,0,0), color, QVec::vec3(100,100,100));
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
			if(cTarget.isActiveTarget)
				n.estimatedTime = inner->transform("robot", cTarget.target, "world").norm2() / 250; //mean speed
			else 
				n.estimatedTime = 0;
			n.x = bState.x;
			n.z = bState.z;
			n.ang = bState.alpha;
			n.advV = bState.advV;
			n.rotV = bState.rotV;
			if(cTarget.isActiveTarget)
				n.distanceToTarget = inner->transform("robot", cTarget.target, "world").norm2();
			else 
				n.distanceToTarget = 0;
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
	stopRobot();
}