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
 
	InnerModelLaser *lm;
	if ((lm = dynamic_cast<InnerModelLaser *>(inner->getNode("laser"))) != NULL)
		LASER_MAX = lm->max;
	else
		qFatal("Fatal error. Laser element not found in XML file");
 
	//Inir random seed
 qsrand(QTime::currentTime().msec());
 
 //add robot node
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
	
	//Innermodelviewer
	osgView = new OsgView( frame );
	osgGA::TrackballManipulator *tb = new osgGA::TrackballManipulator;
	osg::Vec3d eye(osg::Vec3(4000.,4000.,-1000.));
	osg::Vec3d center(osg::Vec3(0.,0.,-0.));
	osg::Vec3d up(osg::Vec3(0.,-1.,0.));
	tb->setHomePosition(eye, center, up, true);
	osg::Matrixf m;
	m.set( 0.998955, -0.0011, -0.09798, 0, -0.0798273, 0.0116788, -0.998538, 0, 0.000867, 0.997709, 0.0153157, 0, 4585.27, 11113.77, 4683.92, 1); 
	//tb->setByMatrix(osg::Matrixf::lookAt(eye,center,up));
	tb->setByMatrix(m);
 	osgView->setCameraManipulator(tb);
	innerViewer = new InnerModelViewer(inner, "root", osgView->getRootGroup(), true);
	
	//draw the initial position of the roobt
	InnerModelDraw::addPlane_ignoreExisting(innerViewer, "vertex_0", "world", inner->transform("world","robot"), 
																					QVec::vec3(1,0,0), "#00ff00", QVec::vec3(100,100,100));
	//Init closestNode
	closestNode = robotNode;

	//UI
	connect(startButton, SIGNAL(clicked()), this, SLOT(startAction()));
	connect(stopButton, SIGNAL(clicked()), this, SLOT(stopAction()));
	
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(100);
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
			case State::ERROR:
				qDebug() << "Error";
				break;
			} 
		}
    catch(const Ice::Exception e)
    {
      std::cout << e << std::endl;
    }
    
  innerViewer->update();
 	osgView->autoResize();
 	osgView->frame();
}

SpecificWorker::State SpecificWorker::pickNewPoint()
{
  try
  {
		//get a random state
		qpos = QVec::zeros(3);
		this->qpos[0] = QVec::uniformVector(1, FLOORX_MIN, FLOORX_MAX)[0];
		this->qpos[2] = QVec::uniformVector(1, FLOORZ_MIN, FLOORZ_MAX)[0];
		InnerModelDraw::addPlane_ignoreExisting(innerViewer, "target", "world", qpos, QVec::vec3(1,0,0), "#0000ff", QVec::vec3(100,100,100));
	
		qDebug() << __FUNCTION__ << "Robot at:" << inner->transform("world","robot")<< "New point:" << qpos;
		map->set(robotNode, inner->transform("world","robot"));
		
		//Check if the point is accesible from robot position
// 		QVec qposR = inner->transform("laser",qpos,"world");
// 		if ( checkFreeWay( qposR ))
// 		{
// 			qDebug() << __FUNCTION__ << "Free way found. Leaving for VERIFY_POINT";
// 			return State::VERIFY_POINT;
// 		}
// 		
		//If not free way to target, obtain the closest point to the graph
	
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
		qDebug() << __FUNCTION__ << "dist to new point" << d << "at node with position:" << map->operator[](closestNode);
		
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

bool SpecificWorker::checkFreeWay( const QVec &targetInRobot)
{
	qDebug() << __FUNCTION__ ;
	//First turn the robot to point towards the new target
	if (alignToTarget( targetInRobot ) != true )
	{
		stopAction();
		qFatal("Could not align the robot");
	}
	
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
	float alpha =atan2(targetInRobot.x(), targetInRobot.z() );
	float step = ceil(dist/ (ROBOT_SIZE/3));
	QVec tNorm = targetInRobot.normalize();
	QVec r;
	inner->updateTransformValues ("vbox",0, 0, 0, 0, 0, 0, "robot");	
	
	for(size_t  i = 0; i<ldata.size(); i++)
  {
		if(ldata[i].angle <= alpha)
    {
			for(float landa=400; landa<=dist; landa+=step)
			{	
				r = tNorm * landa;
				inner->updateTransformValues ("vbox",r.x(), r.y(), r.z(), 0, alpha, 0, "robot");	
				foreach(QVec p, lPoints)
				{
						QVec pointInRobot = inner->transform("robot", p, "vbox");
						float dPR = pointInRobot.norm2();
						float alphaPR = atan2(pointInRobot.x(), pointInRobot.z());
						for(auto ld : ldata)
							if(ld.angle <= alphaPR)
							{
								if( ld.dist>0 and ld.dist < LASER_MAX and ld.dist >= dPR)
								{
									//qDebug()<<__FUNCTION__<< "Hay camino libre al target" << ldata[i].dist << d;
									break;
								}
								else
								{
									qDebug() << "collision of point(R) " << inner->transform("robot", p, "vbox");
									return false;
								}
							}		
				}
			}
			qDebug()<<__FUNCTION__<< "Hay camino libre al target. Laser distance:" << ldata[i].dist << ". Target distance:" << dist;
			break;  //We found the laser ray aligned with the target.
     }
  }
	return true;  
}

bool SpecificWorker::alignToTarget( const QVec &targetInRobot)
{
	float alfa = atan2(targetInRobot.x(), targetInRobot.z());
	qDebug() << __FUNCTION__ << "Robot at:" << inner->transform("world","robot") << "target(R) at:" << targetInRobot << "alfa:" << alfa;
	QTime reloj; reloj.start();
	
	if( fabs(alfa > 0.5) )	//If target qposR not in front of the robot, turn around
	{
		try	
		{	
			RoboCompTrajectoryRobot2D::TargetPose tp;
			QVec r = inner->transform6D("world","robot");
			tp.x = r.x(); tp.y = r.y();	tp.z = r.z();	tp.ry = alfa + r.ry();
			tp.doRotation = true;
			qDebug() << __FUNCTION__<< "Calling Traj";
			trajectoryrobot2d_proxy->go(tp);	
		}
		catch(Ice::Exception &ex) {std::cout<<ex.what()<<std::endl;};	
	
		RoboCompTrajectoryRobot2D::NavState nState;
		do
		{
			try	
			{	
				nState = trajectoryrobot2d_proxy->getState();	
				usleep(100000);
			}
			catch(Ice::Exception &ex) {std::cout<<ex.what()<<std::endl;}
			qDebug() << __FUNCTION__ << reloj.elapsed();
		}
		while( nState.state != "FINISH" and (reloj.elapsed() < 9000));
		
		if( reloj.elapsed() >= 9000)
			return false;
	}
	return true;
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
			//// CHECK IF THERE IS DIRECT WAY TO TARGET
			
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
	qDebug() << __FUNCTION__ ;
	try	
	{ 
		RoboCompTrajectoryRobot2D::NavState state =  trajectoryrobot2d_proxy->getState();	
		if( state.state == "WORKING" )
			return State::TRAVELLING;
		if( state.state == "FINISH" or state.state == "IDLE")
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
	QVec qposR = inner->transform("laser",qpos,"world");
	float alfa = atan2(qposR.x(), qposR.z());
	qDebug() << __FUNCTION__ << "Robot at:" << inner->transform("world","robot") << "alfa:" << alfa;
	
	//CHANGE THIS FOR A CALL TO THE CONTROLLER
	if( fabs(alfa) > 0.5)	//If target qposR not in front of the robot, turn around
	{
		try	{	differentialrobot_proxy->setSpeedBase(0, 0.4*alfa);	}
		catch(Ice::Exception &ex) {std::cout<<ex.what()<<std::endl;};	
		return State::VERIFY_POINT;
	}
	else 
	{
		try	{	differentialrobot_proxy->setSpeedBase(0,0);	}
		catch(Ice::Exception &ex) {std::cout<<ex.what()<<std::endl;};	
	}
	
	//Check if target is inside laser field. If not make qpos the furthest possible point
	
	///USE FREEWAY HERE -------------------------
	
	QVec newPosR = qposR;
	QVec qposRN = qposR.normalize();
	float dist = qposR.norm2();
	
	for(auto l: ldata)
		if(l.angle < alfa)	//can be achieved because we just oriented the robot towards the target.
		{
			qDebug() << __FUNCTION__ << "Corrected point(R)" << newPosR << "laser dist" << l.dist << "dist" << dist << "laser angle" << l.angle << "dist ++ RR" << dist + ROBOT_RADIUS;
			if( l.dist < (dist + ROBOT_RADIUS*1.9))
			{
				newPosR = qposRN * (float)(l.dist - ROBOT_RADIUS*1.9);
			}
			break;
		}
	
	qDebug() << __FUNCTION__ << "Final point to add to graph:" << inner->transform("world",newPosR,"laser");
		
	//add new node to the graph
	lemon::ListGraph::Node newP = graph.addNode();
	map->set(newP, inner->transform("world",newPosR,"laser"));
	lemon::ListGraph::Edge newEdge = graph.addEdge(this->closestNode, newP);
	edgeMap->set( newEdge, (map->operator[](newP) - map->operator[](this->closestNode)).norm2());
	
	qDebug() << "Listing the graph";
	for (lemon::ListGraph::NodeIt n(graph); n != lemon::INVALID; ++n)
		qDebug() << map->operator[](n);
	
	//Drawing
	static int cont=1;
	InnerModelDraw::addPlane_ignoreExisting(innerViewer, "vertex_" + QString::number(cont), "world", inner->transform("world",newPosR,"laser"), 
																					QVec::vec3(1,0,0), "#00ff00", QVec::vec3(100,100,100));
	
	QLine2D line(inner->transform("world",newPosR,"laser") , map->operator[](this->closestNode) );	
	float dl = (inner->transform("world",newPosR,"laser") - map->operator[](this->closestNode)).norm2();
	QVec center = map->operator[](this->closestNode) + ((inner->transform("world",newPosR,"laser") - map->operator[](this->closestNode))*(float)0.5);
	
	InnerModelDraw::drawLine(innerViewer, "line_" + QString::number(cont++), "world", line.getNormalForOSGLineDraw(), center, dl, 50, "#0000ff");
	
	InnerModelDraw::removeNode(innerViewer, "target");
	qDebug() << "---------------";
	
	//return State::PICK_NEW_POINT;	
	return State::FINISH;	

}

////////////////////////////
/// UI
////////////////////////////

void SpecificWorker::startAction()
{
	estado = State::PICK_NEW_POINT;
}

void SpecificWorker::stopAction()
{
	trajectoryrobot2d_proxy->stop();
	estado = State::FINISH;
	osgGA::CameraManipulator *cm = osgView->getCameraManipulator();
	std::cout << cm->getMatrix() << std::endl;
}


////////////////////////////
/// ICEPeriod
////////////////////////////

void SpecificWorker::newAprilTag(const tagsList& tags)
{
}
 