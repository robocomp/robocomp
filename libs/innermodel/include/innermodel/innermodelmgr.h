/*
 * Copyright 2017 pbustos <email>
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

#ifndef INNERMODELMGR_H
#define INNERMODELMGR_H

#include <mutex>
#include <innermodel/innermodel.h>
#include <thread>

class InnerModelMgr
{
	public:
		
		typedef std::lock_guard<std::recursive_mutex> guard;
		
		InnerModelMgr() = default;
		InnerModelMgr(std::shared_ptr<InnerModel> inner_) : innerptr(inner_){};
		InnerModelMgr(InnerModel *inner_) : innerptr(std::shared_ptr<InnerModel>(inner_)){};
		InnerModelMgr(const InnerModelMgr &other) : innerptr(other.innerptr) {	/*std::cout << "COPY CONSTRUCTOR";*/};
		InnerModelMgr(InnerModelMgr &other): innerptr((other.innerptr)){ /*std::cout << "MOVE CONSTRUCTOR" << std::endl;*/ };
		InnerModelMgr& operator=(const InnerModelMgr &other)
		{ 
			//std::cout << "InnerModelMgr: COPY ASSIGNMENT" << std::endl;
			if (this != &other)
				this->innerptr = other.innerptr;
			return *this;
		};
		InnerModelMgr& operator=(InnerModelMgr &other)
		{ 
			//std::cout << "InnerModelMgr: NON CONST COPY ASSIGNMENT" << std::endl;
			if (this != &other)
				this->innerptr = other.innerptr;
			return *this;
		};
// 		InnerModelMgr& operator=(const InnerModelMgr &&other)
// 		{ 
// 			//std::cout << "InnerModelMgr:MOVE ASSIGNMENT" << std::endl;
// 			if (this != &other)
// 				this->innerptr = std::move(other.innerptr);
// 			return *this;
// 		};
		
		void initialize(InnerModel *inner_)						{ innerptr = std::make_shared<InnerModel>(inner_);};
		void initialize(std::shared_ptr<InnerModel> innerptr_)	{ innerptr = innerptr_;};
		InnerModel* operator->()								{ return innerptr.get();};
		void lock()												{ innerptr->mutex.lock();};
		void unlock()											{ innerptr->mutex.unlock();};
		InnerModel* get()										{ return innerptr.get();};
		InnerModelMgr deepcopy()
		{ 
			std::cout << "DEEP COPY" << std::endl;
			lock();
				InnerModelMgr ret(std::make_shared<InnerModel>(innerptr->copy()));
			unlock();
			return ret;
		};
		std::recursive_mutex& mutex() 							{ return innerptr->mutex;};
		void reset(InnerModelMgr &innerModel_)					{ lock(); innerptr.reset(innerModel_.get()); unlock();};
		void reset(InnerModel *inner)							{ lock(); innerptr.reset(inner); unlock();};
		void print(){ std::cout << "Estoy OK" << std::endl;};
		
 	private:
		std::shared_ptr<InnerModel> innerptr;
		
};

#endif // INNERMODELMGR_H
