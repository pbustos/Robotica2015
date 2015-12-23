/*
 * Copyright 2013 <copyright holder> <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "waypoints.h"

WayPoints::WayPoints()
{
	indexOfClosestPointToRobot = 0;
	robotDistanceToClosestPoint = 0;
	robotPerpendicularDistanceToRoad = 0;
	indexOfClosestPointToRobot = 0;
	currentPointIndex = 0;
	angleWithTangentAtClosestPoint = 0;
	roadCurvatureAtClosestPoint = 0;
	robotDistanceToTarget = 0;
	robotDistanceToLastVisible = 0;
	currentPointIndex = 0;
	nextPointIndex = 1;
	finish = false;
	blockedRoad = false;
	isLost = false;
	currentCollisionIndex = 0;
	currentDistanceToFrontier = 0;
	requiresReplanning = false;
	meanSpeed = 300.f;  //Initial speed. Should be read from disk
	antDist = std::numeric_limits< float >::max();
	robotRadius = 200.f;
}

WayPoints::~WayPoints()
{
}

void WayPoints::reset()
{
	clear();
	indexOfClosestPointToRobot = 0;
	robotDistanceToClosestPoint = 0;
	robotPerpendicularDistanceToRoad = 0;
	indexOfClosestPointToRobot = 0;
	currentPointIndex = 0;
	angleWithTangentAtClosestPoint = 0;
	roadCurvatureAtClosestPoint = 0;
	robotDistanceToTarget = 0;
	robotDistanceToLastVisible = 0;
	currentPointIndex = 0;
	nextPointIndex = 1;
	finish = false;
	blockedRoad = false;
	isLost = false;
	currentCollisionIndex = 0;
	currentDistanceToFrontier = 0;
	requiresReplanning = false;
	backList.clear();
	antDist = std::numeric_limits< float >::max();
	
}

void WayPoints::startRoad()
{
	reloj.restart();
	setETA();
	initialDurationEstimation = getETA();
}

void WayPoints::endRoad()
{
	elapsedTime = reloj.restart();
	meanSpeed = 0.5 * meanSpeed + 0.5 * (getRobotDistanceToTarget() / elapsedTime);
	//We should save it now to disk
}

void WayPoints::readRoadFromFile(InnerModel *innerModel, std::string name)
{
	clear();
	std::ifstream file(name.c_str(), std::ios_base::in);
	if( file.is_open() )
	{
		QVec rPos = innerModel->transform("world","robot");
		append(WayPoint(rPos));

		while(file.eof() == false)
		{
			WayPoint w(QVec::zeros(3));
			file >> w.pos[0] >> w.pos[2];
			append(w);
		}
// 		qDebug() << "SpecificWorker::ReadFromFile:: " << QString::fromStdString(name) << "read with" << size() << "points";
	}
	else
		qDebug() << "Could not open File " << QString::fromStdString( name );
}

void WayPoints::readRoadFromList( QList< QVec > list)
{
	Q_ASSERT_X(list.size()>0, "readRoadFromList", "Empty list");
	clear();
	foreach(QVec point, list)
	{
		append(WayPoint(point));
	}
}

void WayPoints::computeDistancesToNext()
{
	for(int i=0; i+1<this->size(); i++) // exlude 0 because it is underneath the robot
	{
		this->operator[](i).initialDistanceToNext = (this->operator[](i).pos - this->operator[](i+1).pos).norm2();
	}
}

void WayPoints::removeFirst(InnerModelViewer *innerViewer)
{
	InnerModelDraw::removeObject(innerViewer, first().centerTransformName);
	InnerModelDraw::removeObject(innerViewer, first().centerMeshName);
	InnerModelDraw::removeObject(innerViewer, first().ballTransformName);
	InnerModelDraw::removeObject(innerViewer, first().ballMeshName);

	QList< WayPoint >::removeFirst();
}

/**
 * @brief Compute QLine2D corresponding to the robot Z axis in World reference frame
 *
 * @return QLine2D
 */
QLine2D WayPoints::getRobotZAxis(InnerModel* innerModel)
{
	Q_ASSERT(currentPoint+1<road.size() and road.size()>0);

	QVec robotPos = innerModel->transform("world", QVec::zeros(3),"base");
	//QVec robot2DPos = QVec::vec2( innerModel->getBaseX(), innerModel->getBaseZ());
	QVec nose = innerModel->transform("world", QVec::vec3(0,0,1000), "base");
	//QVec noseR = QVec::vec2(nose.x(),nose.z());
	return QLine2D(robotPos, nose);
	
}


float WayPoints::robotDistanceToCurrentPoint(InnerModel* innerModel)
{
	return (innerModel->transform("world", QVec::zeros(3), "base") - (*this)[currentPointIndex].pos).norm2();
}

float WayPoints::robotDistanceToNextPoint(InnerModel* innerModel)
{
	return (innerModel->transform("world", QVec::zeros(3), "base") - (*this)[nextPointIndex].pos).norm2();
}

QLine2D WayPoints::getTangentToCurrentPoint()
{
	Q_ASSERT (currentPoint+1 < size() and size()>0);

	QVec p1 = QVec::vec2( (*this)[currentPointIndex].pos.x(), (*this)[currentPointIndex].pos.z());
	QVec p2 = QVec::vec2( (*this)[currentPointIndex+1].pos.x(), (*this)[currentPointIndex+1].pos.z());
	QLine2D line( p1, p2);
	return line;
}

void WayPoints::printRobotState(InnerModel* innerModel, const CurrentTarget &currentTarget)
{
		QVec robot3DPos = innerModel->transform("world", "robot");
		qDebug() << "-------Road status report  ---------------------";
		qDebug() << "	Robot position:" << robot3DPos;
		qDebug() << "	Target:" << currentTarget.getTranslation();
		qDebug() << "	Target2:" << last().pos;
		qDebug() << "	Num points:" << this->size();
		qDebug() << "	Robot dist to closest point in road:" << getRobotDistanceToClosestPoint();
		qDebug() << "	Robot perp. dist to road tangent at closest point:" << getRobotPerpendicularDistanceToRoad();
		qDebug() << "	Angle with road:" << getAngleWithTangentAtClosestPoint();
		qDebug() << "	Dist to target:" << getRobotDistanceToTarget();
		qDebug() << "	Distance variation to target:" << getRobotDistanceVariationToTarget();
		qDebug() << "	Road curvature:" << getRoadCurvatureAtClosestPoint();
		//qDebug() << "	Index of closest point:" << getIndexOfClosestPointToRobot();
		qDebug() << "	Closest point:" << getIndexOfClosestPointToRobot()->pos;
		qDebug() << "	Tangent at closest point:" << getTangentAtClosestPoint();
		qDebug() << "	Current point index: " << currentPointIndex;
		qDebug() << "	Is Blocked:" << isBlocked();
		qDebug() << "	Is Lost:" << isLost;
		qDebug() << "	Is Finished:" << isFinished();
		qDebug() << "	Requires replanning:" << requiresReplanning;
		qDebug() << "	ETA:" << estimatedTimeOfArrival << " sg";
		qDebug() << "	ElapsedTime:" << elapsedTime/1000 << " sg";
		qDebug() << "	Duration estimation:" << initialDurationEstimation << " sg";
		qDebug() << "	Estimation error:" << initialDurationEstimation - elapsedTime/1000 << " sg";
		qDebug() << "	Road BackList size:" << backList.size();
		

		qDebug() << "----------------------------------------------------";
}

void WayPoints::print() const
{
	qDebug() << "Printing Road";
	for(int i=0; i<this->size(); i++)
	{
		const WayPoint &w = (*this)[i];
		qDebug() << "		" << w.pos << i << "Visible" << w.isVisible << "MinDist" << w.minDist << "MinDistVector" << w.minDistPoint
							<< "visibilityAngle" << w.visibleLaserAngle << "visibilityDistance" << w.visibleLaserDist << "in robot frame" << w.posInRobotFrame;
	}
}

bool WayPoints::draw(InnerModelViewer *innerViewer, InnerModel *innerModel, const CurrentTarget &currentTarget)
{
	if (size() == 0) return false;
	InnerModelDraw::addTransform_ignoreExisting(innerViewer, "road", "floor");
	
	//Draw all points now
	for(int i=1; i<size(); i++)
	{
		WayPoint &w = (*this)[i];
		WayPoint &wAnt = (*this)[i-1];
		QLine2D l(wAnt.pos, w.pos);
		QLine2D lp = l.getPerpendicularLineThroughPoint( QVec::vec2(w.pos.x(), w.pos.z()));
		QVec normal = lp.getNormalForOSGLineDraw();  //3D vector
		QVec tangent = roadTangentAtClosestPoint.getNormalForOSGLineDraw();		//OJO, PETA SI NO ESTA LA TG CALCULADA ANTES
		QString item = "p_" + QString::number(i);
		InnerModelDraw::addTransform_ignoreExisting(innerViewer, item, "road");
		innerViewer->innerModel->updateTransformValues(item, w.pos.x(), 10, w.pos.z(), 0,0,0);
		
		if ( (int)i == (int)currentPointIndex+1 ) //CHANGE TO getIndexOfClosestPointToRobot()
		{
			InnerModelDraw::drawLine(innerViewer, item+"_line", item, tangent, QVec::zeros(3), 600, 30, "#000055" );
		}
		if (w.isVisible)
			InnerModelDraw::drawLine(innerViewer, item + "_point", item, normal, QVec::zeros(3), 250, 50, "#005500" );
		else
			InnerModelDraw::drawLine(innerViewer, item + "_point", item, normal, QVec::zeros(3), 250, 50, "#550099" );  //Morado
	}
	if( currentTarget.doRotation == true) 	//Draw an arrow indicating final desired orientation
	{
		float rot = currentTarget.getRotation();
		WayPoint &w = this->last();
		QLine2D l(w.pos, w.pos + QVec::vec3((T)(500*sin(rot)),0,(T)(500*cos(rot))));
		QVec ln = l.getNormalForOSGLineDraw();
		QString item = "p_" + QString::number(this->size()-1);
		InnerModelDraw::drawLine(innerViewer, item + "_line", item, ln, QVec::zeros(3), 600, 30, "#400055" );
	}
	
	return true;
}


void WayPoints::clearDraw(InnerModelViewer *innerViewer)
{
	if (innerViewer->innerModel->getNode("road"))
		InnerModelDraw::removeNode(innerViewer, "road");
}


//////////////////////////////////////////////////////////////////////////////////
///////COMPUTATION OF SCALAR MAGNITUDES OF FORCEFIELD
//////////////////////////////////////////////////////////////////////////////////


/**
 * @brief Computes closest point in trajectory to robot. Updates robotDistanceToClosestPoint and currentPointIndex
 *
 * @param robot 3d vector coding robot's position (x,y,z)
 * @return WayPoint closest to Robot
 */
WayPoints::iterator WayPoints::computeClosestPointToRobot(const QVec& robot)
{
	float min = FLT_MAX;
	QList<WayPoint>::iterator it = this->begin();
	QList<WayPoint>::iterator res = this->end();
	uint count=0, index = 0;

	for(it = this->begin(); it != this->end(); ++it, ++count)
	{
		float d = (it->pos - robot).norm2();
		//qDebug () << "d" << d;
		if( d < min )
		{
			min = d;
			res = it;
			index = count;
		}
	}

	robotDistanceToClosestPoint = min;
	currentPointIndex = index;   //DEPRECATED
	orderOfClosestPointToRobot = index;
	indexOfClosestPointToRobot = res;  // INDICES SHOULD NOT BE USED WITH LISTS
	return res;
}



/**
 * @brief Computes the road tangent at point pointed by iterator w.
 *
 * @param w WayPoints::iterator pointing to the point of interest
 * @return QLine2D
 */
QLine2D WayPoints::computeTangentAt(WayPoints::iterator w) const
{
	static QLine2D antLine = QLine2D(QVec::zeros(3), QVec::vec3(0,0,1));  //Initial well formed tangent

	if( size() <2)
	{
		qDebug() << __FUNCTION__ << "Warning. A size 1 road got into here!";
		return antLine;
	}

	WayPoints::iterator ant,post;
	const float MIN_DISTANCE_ALLOWED = 10.f;

	if( w == this->begin())
		ant = w;
	else
		ant = w-1;

	if( w+1 == this->end())
		post = w;
	else
		post = w+1;

	//Now check if the points are too close
	if((post->pos - ant->pos).norm2() < MIN_DISTANCE_ALLOWED )
		return antLine;

		/*and ((post+1) != this->end()))  //Search by the end side
		while ( post != this->end() and ((post->pos - ant->pos).norm2() < MIN_DISTANCE_ALLOWED))
		{ ++post; };

	if (post == this->end()) --post;
	if( (post->pos - ant->pos).norm2() < MIN_DISTANCE_ALLOWED and (post != this->begin())) //Search by the begin side
		while ( post != this->begin() and ((post->pos - ant->pos).norm2() < MIN_DISTANCE_ALLOWED))
		{ --post; };

	if( (post->pos - ant->pos).norm2() < MIN_DISTANCE_ALLOWED )
	{
		qDebug() << __FUNCTION__ << "Warning. This should not happen. Looks like the all remaining points are the same";*/
	//	return antLine;
	//}

	QLine2D l( ant->pos , post->pos );
	if( isnan(l[0]) or isnan(l[1]) or isnan(l[2]))
	{
		ant->pos.print("ant");
 		post->pos.print("post");
		l.print("line");
		print();
		qFatal("Fary in tg");
	}
	antLine = l;
	return  l;
}

/**
 * @brief Computes the distance from the robot to the last visible point in the road
 *
 * @param road ...
 * @param closestPoint ...
 * @param robotPos ...
 * @return float
 */
float WayPoints::computeDistanceToLastVisible(WayPoints::iterator closestPoint, const QVec &robotPos)
{
	float dist = (robotPos - closestPoint->pos).norm2();
	WayPoints::iterator it;
	for(it = closestPoint; it != end()-1; ++it)
	{
		if(it->isVisible == true )
			dist += (it->pos - (it+1)->pos).norm2();
		else
			break;
	}
	indexOfLastVisiblePoint = it;
	return dist;
}

/**
 * @brief Computes the distance to the Target along the road
 *
 * @param road ...
 * @param robotPos ...
 * @return float Distance to target in the units of InnerModel
 */
float WayPoints::computeDistanceToTarget(WayPoints::iterator closestPoint, const QVec &robotPos)
{
	float dist = (robotPos - closestPoint->pos).norm2();
	WayPoints::iterator it;
	for(it = closestPoint; it != end()-1; ++it)
	{
		dist += (it->pos - (it+1)->pos).norm2();
	}
	float distE = (robotPos - it->pos).norm2();  //Euclidean distance to last point to detect the robot going away from the target
	robotDistanceVariationToTarget = distE - antDist;
	antDist = distE;
	return dist;
}

/**
 * @brief Compute angle between current and next segment using the cross product
 * @param robot2DPos (X,Y) positionof the robot
 * @return float value between 0 curvature for straight lines and PI for U turns.
 */

float WayPoints::computeRoadCurvature(WayPoints::iterator closestPoint, uint pointsAhead)
{
	WayPoints::iterator it, final;
	uint count = 0;
	int achieved = 0;
	float sumAng = 0.f;
	for(it = closestPoint; it != end()-1 and count < pointsAhead; ++it, ++count, ++achieved)
	{
		float ang = computeTangentAt( it ).signedAngleWithLine2D( computeTangentAt( it + 1));
		//qDebug() << achieved << road.size() << ang;
		sumAng += ang;
	}
	if( achieved > 0 and isnan(sumAng)==false)
		return sumAng/achieved;
	else
		return 0;
}

/**
 * @brief Estimated time of arrival en miliseconds
 * 
 * @return void
 */
void WayPoints::setETA()
{
	estimatedTimeOfArrival = getRobotDistanceToTarget() / meanSpeed;
	elapsedTime = reloj.elapsed();
}

/**
 * @brief Computes all scalar values used by the Controller to obtain the force field that acts on the robot
 *
 * @return void
 */
bool WayPoints::update()
{
	qDebug() << __FUNCTION__ ;
	
	//Get robot's position in world and create robot's nose
	QVec robot3DPos = innerModel->transform("world", "robot");
	QVec noseInRobot = innerModel->transform("world", QVec::vec3(0,0,1000), "robot");
	QLine2D nose =  QLine2D(  QVec::vec2(robot3DPos.x(),robot3DPos.z()), QVec::vec2(noseInRobot.x(), noseInRobot.z()));

	//Compute closest existing trajectory point to robot
// 	if(getRobotDistanceToTarget() < 1000)  //FOR GO_REFERENCED
// 	{
// 		robot3DPos = innerModel->transform("world", "virtualRobot");
// 	}
	
	WayPoints::iterator closestPoint = computeClosestPointToRobot(robot3DPos);

	//Compute roadTangent at closestPoint;
	//qDebug() << __FILE__  << __FUNCTION__ << "just here"  << getRobotDistanceToClosestPoint();
// 	if(closestPoint == this->end())
// 	{
// 		qDebug("fary en Compute Forces");
// 		return false;
// 	}

	QLine2D tangent = computeTangentAt( closestPoint );
	setTangentAtClosestPoint(tangent);

	//Compute signed perpenduicular distance from robot to tangent at closest point
	setRobotPerpendicularDistanceToRoad( tangent.perpendicularDistanceToPoint(robot3DPos) );
	float ang = nose.signedAngleWithLine2D( tangent );
	if (isnan(ang))
		ang = 0;
 	setAngleWithTangentAtClosestPoint( ang );

	//Compute distanceToTarget along trajectory
  	setRobotDistanceToTarget( computeDistanceToTarget(closestPoint, robot3DPos) );  //computes robotDistanceVariationToTarget
	setRobotDistanceVariationToTarget( robotDistanceVariationToTarget);

	//Update estimated time of arrival
	setETA();

	//Check for arrival to target  TOO SIMPLE
// 	if(	( ((int)getCurrentPointIndex()+1 == (int)size())  and  ( (int)getCurrentPointIndex()+1< 80) )
	if(((((int)getCurrentPointIndex()+1 == (int)size())  or  ( getRobotDistanceToTarget()< 100) ))
		or ( (getRobotDistanceToTarget() < 300) and ( getRobotDistanceVariationToTarget() > 0) ) )
	{
	
		setFinished(true);		
//		qDebug() << __FUNCTION__ << "Arrived:" << (int)getCurrentPointIndex()+1 << (int)getCurrentPointIndex()+1 << getRobotDistanceToTarget() << getRobotDistanceVariationToTarget();
// 		getIndexOfClosestPointToRobot()->pos.print("closest point");
	}

	//compute curvature of trajectory at closest point to robot
  	setRoadCurvatureAtClosestPoint( computeRoadCurvature(closestPoint, 3) );
	setRobotDistanceToLastVisible( computeDistanceToLastVisible(closestPoint, robot3DPos ) );
	return true;
}

/**
 * @brief This method "projects" the path into the laser field, putting it under several forces
 * 
 * @param ldata ...
 * @return bool
 */
bool WayPoints::project(const RoboCompLaser::TLaserData &ldata)
{
	//update road variables
	update();
	
	//Tags all points in the road ar visible or blocked, depending on laser visibility. Only visible points are processed in this iteration
	checkVisiblePoints(ldata);	
	 
 	//Add points to achieve an homogenoeus chain
 	addPoints();
 
 	//Remove point too close to each other
 	cleanPoints();
 	
 	//Compute the scalar magnitudes
 	computeForces(ldata); 	 
 		
// 	//Delete half the tail behind, if greater than 6, to release resources
// 	if( road.getOrderOfClosestPointToRobot() > 6)
// 	{
// 		for(auto it = road.begin(); it != road.begin() + (road.getOrderOfClosestPointToRobot() / 2); ++it)
// 			road.backList.append(it->pos);
// 		road.erase(road.begin(), road.begin() + (road.getOrderOfClosestPointToRobot() / 2));
// 	}
	return true;
}


bool WayPoints::checkVisiblePoints(const RoboCompLaser::TLaserData &laserData)
{	
	qDebug() << __FUNCTION__ ;
	WayPoints &road = *this;
	if( road.size() <=1 ) return false;
	
	float maxAngle = std::max(laserData.front().angle, laserData.back().angle);
	float minAngle = std::min(laserData.front().angle, laserData.back().angle);
	
	for(int i=1; i< road.size(); i++)//We leave out the first point wich is usually under the robot. SHOULD change for iterators
	{		
		WayPoint &w = road[i];	
		w.isVisible = true;
		QVec pr = innerModel->transform("laser", w.pos, "world");
		float angle = atan2(pr.x(),pr.z());
		//Check for outofbounds
		if(((angle < minAngle) or (angle > maxAngle)) and (pr.z()>10)) 
		{
			//qDebug() << __FILE__<< __FUNCTION__ << "ElasticBand::checkVisiblePoints - exiting due to angle" << angle << i << pr;
			w.isVisible = false;
			for(int k=i; k<road.size(); k++)  //should be i+1
				road[k].isVisible = false;
			return false;
		}
			
		//Find laser index corresponding to "angle"
		uint j;
		float init = laserData[0].angle;				
		//qDebug() << "angle" << angle << "length" << length << "init" << init << pr << w1.pos << w2.pos;
		for(j=1; j< laserData.size();j++)
		{
			if( laserData[j].angle > init ) //ascending order
			{	
				if ( laserData[j].angle >= angle ) //already there
					break;
			}
			else //descending
			{
				if( laserData[j].angle <= angle )
					break;
			}
		}
		//Check if the point is behind the laser beam end
		w.visibleLaserAngle = laserData[j].angle;
		pr[1]=innerModel->getNode("laser")->getTr()[1];
		w.posInRobotFrame = pr;
		w.visibleLaserDist = laserData[j].dist;
		
		if( pr.norm2() > laserData[j].dist and (pr.z()>10)) // laser beam is smaller than p point and p is beyond the laser. When p is being crossed visibility is compromised
		{
			w.isVisible = false;
			for(int k=i;k<road.size();k++)
				road[k].isVisible = false;

			return false;
		}
	}
	return true;
}

bool WayPoints::addPoints()
{
	WayPoints &road = *this;
	qDebug() << __FUNCTION__ << "road size:" << road.size();
	if( road.size() <= 1) return false;
	
	int offset = 1;
	for(int i=0; i< road.size()-offset; i++)	
	{
		if( i>0 and road[i].isVisible == false )  
			break;

		WayPoint &w = road[i];
		WayPoint &wNext = road[i+1];
		float dist = (w.pos-wNext.pos).norm2();
	
		if( dist > ROAD_STEP_SEPARATION)  //SHOULD GET FROM IM
		{
			float l = 0.9*ROAD_STEP_SEPARATION/dist;   //Crucial que el punto se ponga mas cerca que la condición de entrada
			WayPoint wNew( (w.pos * (1-l)) + (wNext.pos * l));
			road.insert(i+1,wNew);
			qDebug() << __FUNCTION__ << i << road.size() << "dist" << dist << "inserted point at:" << wNew.pos;
		}
	}
	qDebug() << __FUNCTION__ << "goodbye";
}

bool WayPoints::cleanPoints()
{
	WayPoints &road = *this;
	qDebug() << __FUNCTION__ << "road size:" << road.size();
	if( road.size() <= 2) return false;

	int i;
	int offset=2;
	//if( road.last().hasRotation ) offset = 3; else offset = 2;
	
	for(i=1; i< road.size()-offset; i++) // exlude 1 to avoid deleting the nextPoint and the last two to avoid deleting the target rotation
	{
		if( road[i].isVisible == false )  
			break;
		
		WayPoint &w = road[i];
		WayPoint &wNext = road[i+1];

		float dist = (w.pos-wNext.pos).norm2();
		if( dist < ROAD_STEP_SEPARATION/4. )
		{
			road.removeAt(i+1);
			qDebug() << __FUNCTION__ << i << road.size() << "dist" << dist << "removed point at:" << i+1;
		}
	}
	qDebug() << __FUNCTION__ << "goodbye";
	return true;
}

float WayPoints::computeForces(const RoboCompLaser::TLaserData& laserData)
{
	WayPoints &road = *this;
	qDebug() << __FUNCTION__ << "road size:" << road.size();
	
	if(road.size() < 3 )
		return 0;
	
	QVec atractionForce(3);
	QVec repulsionForce(3);
	QVec jacobian(3);
	float totalChange=0.f;
	int lastP;					// To avoid moving the rotation element attached to the last
	
	if( road.last().hasRotation )
		lastP = road.size()-2;
	else
		lastP = road.size()-1;
	for(int i=1; i< lastP; i++) 
	{
		if( road[i].isVisible == false )
			break;
		
		WayPoint &w0 = road[i-1];
		WayPoint &w1 = road[i];
		WayPoint &w2 = road[i+1];
			
		//LINEAR FORCE II
		float n = (w0.pos-w1.pos).norm2() / (( w0.pos-w1.pos).norm2() + w1.initialDistanceToNext );
		atractionForce = (w2.pos - w0.pos)*n - (w1.pos - w0.pos);	
			
		//REPULSION FORCE FROM OBSTACLEs Compute jacobian of free space wrt to x,y
		QVec repulsionForce = QVec::zeros(3);
		computeDistanceField(w1, laserData, FORCE_DISTANCE_LIMIT);
		float h = DELTA_H;  //CHECK THIS
		if ( ( w1.minDistHasChanged == true) /*and (w1.minDist < 250)*/ )
		{	
			jacobian = QVec::vec3( w1.bMinusX  - w1.bPlusX , 0 , w1.bMinusY  - w1.bPlusY ) * (T)(1.f/(2.f*h));														
			repulsionForce = jacobian * ( FORCE_DISTANCE_LIMIT - w1.minDist );
			//qDebug() << "ElasticBand::computeForces" << i << w1.minDist << w1.	pos << jacobian << repulsionForce << atractionForce;
		}
		
		//REPULSION FORCE II
		// 		QVec repulsionForce = QVec::zeros(3);
		// 		computeDistanceField(w1, laserData, FORCE_DISTANCE_LIMIT);
		// 		repulsionForce = w1.minDistPoint * (FORCE_DISTANCE_LIMIT - w1.minDist);
		
		//combine the forces
		float alpha = -0.4;//0.6
		float beta = 0.80; //0.09	
		QVec change = (atractionForce*alpha) + (repulsionForce*beta);		
		
		if(isnan(change.x()) or isnan(change.y()) or isnan(change.z()))
		{
			road.print();
			qDebug() << atractionForce << repulsionForce;
			qFatal("change");	
		}
		
		// Now we remove the tangencial component of the force to avoid recirculation of band points 
		// by projecting the resulting force on the perpendicular to the point.
		QVec pp = road.getTangentToCurrentPoint().getPerpendicularVector().normalize();
		Q_ASSERT(pp.size() == 3);
		QVec nChange = pp* (pp * change);
		
		w1.pos = w1.pos - change;		
		totalChange = totalChange + nChange.norm2();
	}
	return totalChange;
}

void WayPoints::computeDistanceField(WayPoint &ball, const RoboCompLaser::TLaserData &laserData, float forceDistanceLimit)
{
	
	ball.minDist = ball.bMinusX = ball.bPlusX = ball.bMinusY = ball.bPlusY = std::numeric_limits<float>::max();
	ball.minDistHasChanged = false;
	
	//QVec c = innermodel->transform("base", ball.pos, "world");
	QVec c = ball.pos;
	c[1] = innerModel->getNode("laser")->getTr()[1]; //Put the y coordinate to laser height so norm() works allright
	int index = -1;
	
	for(uint i=0; i<laserData.size(); i++)
	{
		QVec l = innerModel->laserTo("world", "laser" , laserData[i].dist, laserData[i].angle);
	
		float dist = (l-c).norm2();
		
		if( dist < ball.minDist )
		{
			ball.minDist = dist;
			index = i;
		}
		
 		float h = DELTA_H; //Delta 
 		dist = (l-(c - QVec::vec3(h,0,0))).norm2();
 		if( dist < ball.bMinusX ) ball.bMinusX = dist;
		
		dist = (l-(c + QVec::vec3(h,0,0))).norm2();
		if( dist < ball.bPlusX ) ball.bPlusX = dist;
		
		dist = (l-(c - QVec::vec3(0,0,h))).norm2();
		if( dist < ball.bMinusY ) ball.bMinusY = dist;		
		
		dist = (l-(c + QVec::vec3(0,0,h))).norm2();
		if( dist < ball.bPlusY ) ball.bPlusY = dist;				
	}

	QVec lw = innerModel->laserTo("world", "laser" , laserData[index].dist, laserData[index].angle);
	ball.minDistPoint = (lw - c).normalize();
	
	//qDebug() << "before if" << ball.minDist;
	
		if ( ball.minDist < forceDistanceLimit )
	{
		if ( fabs(ball.minDist - ball.minDistAnt) > 2 ) //mm
		{
			ball.minDistAnt = ball.minDist;
			ball.minDistHasChanged = true;
		}
	}
	else
		ball.minDist = forceDistanceLimit;
}