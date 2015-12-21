/*
 * Copyright 2015 <copyright holder> <email>
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

#ifndef CURRENTTARGET_H
#define CURRENTTARGET_H

#include <qmat/QMatAll>

class CurrentTarget
{
	public:
		CurrentTarget();
		CurrentTarget(const CurrentTarget& other);
		~CurrentTarget();
	
		QVec getTranslation() const { return target.subVector(0,2);};
		float getRotation() const { Q_ASSERT(target.size() == 6); return target.ry();};
		QVec getTarget(){ return target;};
		void setTarget(QVec t) {target = t;};
		bool isActiveTarget=false;	
		bool doRotation = false;
		void reset() { doRotation = false; isActiveTarget = false; };
		
private:
		float rot;
	 	QVec target;
	
};



#endif // CURRENTTARGET_H
