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
			
	path.setInnerModel(inner);
	path.setRobotRadius(ROBOT_RADIUS);
	
	try 
	{	
		ldata = laser_proxy->getLaserData(); 
		linefollower.initialize(inner, ldata, 1);
	}
	catch(Ice::Exception &ex) {std::cout<<ex.what()<<std::endl;};
	
	graphicsView->setScene(&scene);
	graphicsView->show();
	graphicsView->scale(3,3);
	
	//Innermodelviewer
	osgView = new OsgView(this->frame);
	osgGA::TrackballManipulator *tb = new osgGA::TrackballManipulator;
	osg::Vec3d eye(osg::Vec3(4000.,4000.,-1000.));
	osg::Vec3d center(osg::Vec3(0.,0.,-0.));
	osg::Vec3d up(osg::Vec3(0.,1.,0.));
	tb->setHomePosition(eye, center, up, true);
	tb->setByMatrix(osg::Matrixf::lookAt(eye,center,up));
 	osgView->setCameraManipulator(tb);
	innerViewer = new InnerModelViewer(inner, "root", osgView->getRootGroup(), true);

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
     omnirobot_proxy->getBaseState(bState);
     ldata = laser_proxy->getLaserData();
     inner->updateTransformValues("robot", bState.x, 0, bState.z, 0, bState.alpha, 0);
	
		 switch( state )
		 {
			 case State::INIT:
				 state = State::IDLE;
				 break;
				 
			 case State::IDLE:
				 break;
				 
			 case State::SET_NEW_TARGET:
				 state  = setNewTarget();
				 break;
			
			 case State::FOLLOW_PATH:
				 state  = followPath();
				 break;
			 
		   case State::FINAL_TURN:
						finalTurn();
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


/**
 * @brief Start a new target killing the current one. We create the elastic band here
 * 
 * @return SpecificWorker::State
 */
SpecificWorker::State SpecificWorker::setNewTarget()
{
	qDebug() << __FUNCTION__ << "robot at:" << inner->transform("world","robot") << ". Target at: " << cTarget.getTarget();
	try
	{	omnirobot_proxy->setSpeedBase(0, 0,0); }
	catch(Ice::Exception &ex) {std::cout<<ex.what()<<std::endl;};
	
	//Build the path. Initially a 2 points path
	QList<QVec> points;
	points  << inner->transform("world","robot");
	points  << cTarget.getTranslation();
	path.readRoadFromList( points );
	qDebug() << __FUNCTION__ << "Path built:" << points;
	qDebug() << __FUNCTION__ << "---------------------------";
	return State::FOLLOW_PATH;
}

SpecificWorker::State SpecificWorker::followPath()
{
	qDebug() << __FUNCTION__;
	if( atTarget())
	{
		qDebug() << __FUNCTION__ << "At target! Moving to finalTurn";
		return State::FINAL_TURN;
	}
	
	path.project( ldata );
	controller();
	path.draw(innerViewer, inner, cTarget);

	qDebug() << __FUNCTION__ << "---------------------------";
	return State::FOLLOW_PATH;
}

/**
 * @brief Turn the robot to a final absolute orientation in the world coordinate frame.
 * 
 * @return SpecificWorker::State
 */
SpecificWorker::State SpecificWorker::finalTurn()
{
	qDebug()<<  __FUNCTION__;
	
	if( cTarget.doRotation == false)
	{
		qDebug()<<  __FUNCTION__ << "No target angle to go. doRotation = false";
		stopRobotAndFinish();
	}
		
  float alpha = inner->transform6D("world","robot").ry();  //Current robot angle wrt to global Z axis
	float beta = cTarget.getRotation();  //target angle with to Z axis
	qDebug()<<  __FUNCTION__<< "error" << (alpha-beta);
	
	//float error = atan2(sin(beta-alpha), cos(beta-alpha));
	float errA = fmod(alpha -beta, M_PI);
	float errB = fmod(beta -alpha, M_PI);
	float error;
	if( errA < errB ) error = -errA;
	if( errB >= errA ) error = errB;
	
	if( fabs(error) < MAX_ANGLE_ERROR )
	{
		qDebug()<<  __FUNCTION__ << "Finish TurnFinal with erro:"  << error;
		cTarget.doRotation = false;
		stopRobotAndFinish();
	}
	else
	{	
		try	
		{	
			omnirobot_proxy->setSpeedBase(0, 0, -0.4 * error);
			//yDQ.enqueue(-0.4*error); if(yDQ.size() >= 100) yDQ.dequeue();	
		}
		catch(Ice::Exception &ex) {std::cout<<ex.what()<<std::endl;};	
	}
	return State::FINAL_TURN;
}


SpecificWorker::State SpecificWorker::turn()
{
	qDebug()<<  __FUNCTION__;
	float alpha;
	QVec t = inner->transform("robot", cTarget.getTranslation(), "world");
	alpha =atan2(t.x(), t.z() );
	
	if( alpha < ldata[20].angle and alpha > ldata[ldata.size()-20].angle)
	{
		stopRobot();
		state = State::FOLLOW_PATH;
	}
	else
	{	
		float rot = alpha;
		if( fabs(rot) > MAX_ROBOT_ROTATION_SPEED) 
			rot = std::copysignf(MAX_ROBOT_ROTATION_SPEED,alpha);
		
		if( alpha > ldata.front().angle ) 	// turn right
				rot = fabs(rot);
		if( alpha < ldata.back().angle ) 		// turn left
				rot = -fabs(rot);
		
		try
		{ 
			omnirobot_proxy->setSpeedBase(0, 0, rot);
			//yDQ.enqueue(rot); if(yDQ.size() >= 100) yDQ.dequeue();	
			//yDAQ.enqueue(0); if(yDAQ.size() >= 100) yDAQ.dequeue();	
		}
		catch(Ice::Exception &ex) {std::cout<<ex.what()<<std::endl;};
	}
	return State::TURN;
}

/**
 * @brief Computes if there is freeway betwwen the robot and a given target
 * 
 * @return bool
 */
void SpecificWorker::freeWay(const QVec &tg)
{
// 	Q_ASSERT( tg.size()>=3 );
// 	QVec t = inner->transform("robot", tg.subVector(0,2) , "world");
//   float d = t.norm2();
//   float alpha =atan2(t.x(), t.z() );
// 	
// 	qDebug()<< "---------------------------";
//   qDebug()<< __FUNCTION__<< "target " << t << "distancia: " << d;
//   
// 	for(uint i = 0; i<ldata.size(); i++)
//   {
// 		if(ldata[i].angle <= alpha)
//     {
// 			if( ldata[i].dist < 4000 and ldata[i].dist < d)
// 			{	
// 				qDebug() << __FUNCTION__<< " = false at d=" << d << "alpha = " << alpha << "laser=" << ldata[i].angle << ldata[i].dist;
// 				return State::;
// 			}
// 			else
// 			{
// 				qDebug()<<__FUNCTION__<< "Hay camino libre al target" << ldata[i].dist << d;
// 				return true;
// 			}
//      }
//   }
//   qDebug() << "The target is behind. I should turn around";
// 	state = State::TURN;
//   return false;
}

////////////////////////
/// Utilities
///////////////////////
bool SpecificWorker::controller()
{
	
	return true;
}

bool SpecificWorker::plan()
{
	return true;
}

bool SpecificWorker::atTarget()
{
  QVec t = inner->transform("robot", cTarget.getTarget(), "world");
  float d = t.norm2();
	static float dAnt = d;
	
	qDebug()<< "---------------------------";
  qDebug()<< __FUNCTION__<< "target(R) " << t << "distancia: " << d << "mm";
  
	if ( d < 100 or (d<500 and (d-dAnt)>0) )
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

void SpecificWorker::stopRobot()
{
	try
	{	
		qDebug() << __FUNCTION__ ;
		omnirobot_proxy->setSpeedBase(0, 0,0);
	}
	catch(Ice::Exception &ex) {std::cout<<ex.what()<<std::endl;};
}

void SpecificWorker::stopRobotAndFinish()
{
	try
	{	
		qDebug() << __FUNCTION__ ;
		omnirobot_proxy->setSpeedBase(0, 0,0);
		cTarget.isActiveTarget = false;
		state = State::FINISH;
		undrawTarget("target");		
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
				case State::FOLLOW_PATH:
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
				default:
					break;
			}
			n.elapsedTime = elapsedTime.elapsed();
			n.estimatedTime = inner->transform("robot", cTarget.getTarget(), "world").norm2() / 250; //mean speed
			n.x = bState.x;
			n.z = bState.z;
			n.ang = bState.alpha;
	/*		n.advV = bState.advV;
			n.rotV = bState.rotV;
	*/		n.distanceToTarget = inner->transform("robot", cTarget.getTarget(), "world").norm2();
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
	cTarget.setTarget(QVec::vec6(target.x, target.y, target.z, target.rx, target.ry, target.rz));
	cTarget.doRotation =  target.doRotation;
	cTarget.isActiveTarget = true;
	elapsedTime = QTime::currentTime();
	qDebug()<< __FUNCTION__ <<"ICE::GO" << cTarget.getTarget();
	if( target.doRotation)
			qDebug() << "With rotation: " << target.ry;
	state = State::SET_NEW_TARGET;
	drawTarget("target",cTarget.getTarget(), "#ff0000");
	qDebug() << __FUNCTION__ << "---------------------------";
	return (inner->transform("world","robot") - cTarget.getTarget()).norm2();	//Distance to target in mm
}


void SpecificWorker::mapBasedTarget(const NavigationParameterMap &parameters)
{

}


void SpecificWorker::stop()
{
	cTarget.isActiveTarget = true;
	stopRobot();
}