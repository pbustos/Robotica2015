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

#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include <QObject>
#include <qmat/QMatAll>
#include <innermodel/innermodel.h>
#include <iostream>
#include <fstream>
#include <innermodel/innermodeldraw.h>
#include <float.h>

#include <Laser.h>

#include <qline2d/qline2d.h>
#include "currenttarget.h"


#define ROAD_STEP_SEPARATION (robotRadius*2)
#define FORCE_DISTANCE_LIMIT (robotRadius*3)  //mm
#define DELTA_H (robotRadius/4)  //Increment of translation for J in rep forces

class WayPoints;

class WayPoint
{
	public:
		WayPoint()
			{ pos = QVec::zeros(3); minDist = 0; minDistAnt = 0.f; isVisible = false; minDistPoint = QVec::zeros(3); hasRotation = false;};
		WayPoint(QVec p) 	
			{ pos = p; minDist = 0; minDistAnt = 0.f; isVisible = false; minDistPoint = QVec::zeros(3); hasRotation = false;};
		~WayPoint(){};
	
		//For ElasticBand
		QVec pos;								// 3D point (x,y,z)
		QVec rot;								// Euler angles (x,y,z)
		float minDist, minDistAnt;
		QVec minDistPoint; //In world ref system
		float bMinusY, bPlusY, bMinusX, bPlusX;
		bool minDistHasChanged;
		QString centerTransformName, centerMeshName, centerLineName, centerPointName, ballTransformName, ballMeshName;
		bool isVisible;
		float initialDistanceToNext;
		float visibleLaserAngle;
		float visibleLaserDist;
		QVec posInRobotFrame;
		bool hasRotation;
		
}; 

class WayPoints : public QList< WayPoint >
{
	
	public:
		WayPoints();
		~WayPoints();
		void reset();
		void startRoad();
		void endRoad();
		void setThreshold(const float _threshold) 										{ threshold = _threshold;};
		void setInnerModel( InnerModel *inner) 												{ innerModel = inner;};
		void readRoadFromFile(InnerModel *innerModel, std::string name);
		void readRoadFromList(QList<QVec> list);
		void printRobotState(InnerModel* innerModel, const CurrentTarget& currentTarget);
		void print() const;
		bool draw(InnerModelViewer *innerViewer, InnerModel *innerModel, const CurrentTarget &currentTarget);  //Default in upTo means all list
		void clearDraw(InnerModelViewer *innerViewer);
		QList<QVec> backList;
		bool update();
		
		void setRobotRadius( float r) { robotRadius = r;};
		bool project( const RoboCompLaser::TLaserData &ldata);
		bool checkVisiblePoints(const RoboCompLaser::TLaserData &laserData);
		bool addPoints();
		bool cleanPoints();
		float computeForces(const RoboCompLaser::TLaserData& laserData);
		void computeDistanceField(WayPoint &ball, const RoboCompLaser::TLaserData &laserData, float forceDistanceLimit);
		
		//GOOD ONES
		float robotDistanceToCurrentPoint(InnerModel *innerModel);
		float robotDistanceToNextPoint(InnerModel *innerModel);
		WayPoint const getCurrentPoint() const 												{return (*this)[currentPointIndex];};
		WayPoint const getNextPoint() const 												{return (*this)[nextPointIndex];};
		QLine2D getRobotZAxis(InnerModel* innerModel);
		void computeDistancesToNext();
		QLine2D getTangentToCurrentPoint();
		WayPoints::iterator getIndexOfClosestPointToRobot() const 							{ return indexOfClosestPointToRobot;};
		QLine2D getTangentAtClosestPoint() const											{ return roadTangentAtClosestPoint;};
		float getRobotDistanceToClosestPoint() 	const										{ return robotDistanceToClosestPoint;};
		float getRobotPerpendicularDistanceToRoad()	const									{ return robotPerpendicularDistanceToRoad;};
		float getAngleWithTangentAtClosestPoint() const										{ return angleWithTangentAtClosestPoint;};
		uint getCurrentPointIndex() const													{ return currentPointIndex;};  //DEPRECATED
		float getRoadCurvatureAtClosestPoint() const										{ return roadCurvatureAtClosestPoint;};
		float getRobotDistanceToTarget() const												{ return robotDistanceToTarget;};
		float getRobotDistanceToLastVisible() const											{ return robotDistanceToLastVisible;};
		float getRobotDistanceVariationToTarget() const 									{ return robotDistanceVariationToTarget;};
		ulong getETA() const 																{ return estimatedTimeOfArrival;};
		WayPoints::iterator getIndexOfLastVisiblePoint() const								{ return indexOfLastVisiblePoint;};
		uint32_t getOrderOfClosestPointToRobot() const										{ return orderOfClosestPointToRobot;};
		bool isBlocked() const																{ return blockedRoad;};
		bool isFinished() const 															{ return finish;};
		
		void setIndexOfClosestPointToRobot(WayPoints::iterator it) 							{ indexOfClosestPointToRobot = it;};
		void setTangentAtClosestPoint(const QLine2D &tangent) 								{ roadTangentAtClosestPoint = tangent;};
		void setRobotDistanceToClosestPoint(float dist) 									{ robotDistanceToClosestPoint = dist;};
		void setRobotPerpendicularDistanceToRoad(float dist) 								{ robotPerpendicularDistanceToRoad = dist;};
		void setAngleWithTangentAtClosestPoint( float ang)									{ angleWithTangentAtClosestPoint = ang;};
		void setRoadCurvatureAtClosestPoint( float c)										{ roadCurvatureAtClosestPoint = c;};
		void setRobotDistanceToTarget( float dist)											{ robotDistanceToTarget = dist;};
		void setRobotDistanceToLastVisible( float dist)										{ robotDistanceToLastVisible = dist;};
		void setFinished( bool b)															{ QMutexLocker ml(&mutex); finish = b; }
		void setRobotDistanceVariationToTarget(float dist)									{ robotDistanceVariationToTarget = dist;};
		void setBlocked(bool b)																{ blockedRoad = b;};
		void changeTarget(const QVec &target)												{ QMutexLocker ml(&mutex); replace(length()-1, target); antDist = std::numeric_limits< float >::max();};
		void setETA();
		void removeFirst(InnerModelViewer *innerViewer);
		
		int nextPointIndex;
		bool blockedRoad;
		bool isLost;
		int currentCollisionIndex;
		float currentDistanceToFrontier;
		bool requiresReplanning;
	
	private:
		float robotDistanceToClosestPoint;
		float robotPerpendicularDistanceToRoad;
		QLine2D roadTangentAtClosestPoint;
		WayPoints::iterator indexOfClosestPointToRobot, indexOfLastVisiblePoint;
		uint32_t orderOfClosestPointToRobot;
		uint currentPointIndex;
		float angleWithTangentAtClosestPoint;
		float roadCurvatureAtClosestPoint;
		float robotDistanceToTarget;
		float robotDistanceVariationToTarget;
		float robotDistanceToLastVisible;
		float threshold;
		bool finish;
		ulong estimatedTimeOfArrival;
		InnerModel *innerModel;
		QTime reloj;
		float meanSpeed;  
		long elapsedTime;
		int initialDurationEstimation;
		float antDist; //To be used in robotDistanceVariationToTarget computation
		float robotRadius;
		QMutex mutex;
		
		float computeRoadCurvature(WayPoints::iterator closestPoint, uint pointsAhead);
		float computeDistanceToTarget(WayPoints::iterator closestPoint, const QVec &robotPos);
		float computeDistanceToLastVisible(WayPoints::iterator closestPoint, const QVec &robotPos);
		QLine2D computeTangentAt(WayPoints::iterator w) const;
		WayPoints::iterator computeClosestPointToRobot(const QVec& robot);
		
		
};

#endif // WAYPOINTS_H
