
#ifndef BOUNDBUFFER_H
#define BOUNDBUFFER_H

#include <tuple>


//
// Buffer for a non void return type call
//
template<class P, class R=void>
class BoundBuffer
{
	private:
		std::queue<P> buffer;
		QMutex mutex;
		uint current_id=0;
		std::map<uint, R> results;

	public:
		BoundBuffer(){};
		uint push(P params);
		std::tuple<uint, P> pop();
		R result(uint cid); // only use when the function has a non-void return type
		bool isEmpty();
		void setFinished(uint id);
		bool isFinished(uint id);
};

template<class P, class R>
uint BoundBuffer<P, R>::push(P params)
{
	cout<<"pushed "<<endl;
	mutex.lock();
	buffer.push(params);
	current_id++;
	mutex.unlock();
	return current_id;
}

template<class P, class R>
std::tuple<uint, P> BoundBuffer<P, R>::pop()
{
	cout<<"poped"<<endl;
	uint cid = current_id;
	P params;
	mutex.lock();
	params = buffer.front();
	buffer.pop();
	current_id--;
	mutex.unlock();
	return std::make_tuple(cid, params);
}

template<class P, class R>
R BoundBuffer<P, R>::result(uint cid)
{
	R result;
	if(isFinished(cid)){
	    results.erase(cid);
	}
	return result;
}

template<class P, class R>
bool BoundBuffer<P, R>::isEmpty()
{
	return buffer.empty();
}

template<class P, class R>
void BoundBuffer<P, R>::setFinished(uint cid)
{
	results[cid] = true;
}

template<class P, class R>
bool BoundBuffer<P, R>::isFinished(uint cid)
{
	bool is_finished = results.find(cid) != results.end();
	return is_finished;
}


//
// For non return types
//
template<class P>
class BoundBuffer<P, void>
{
	private:
		std::queue<P> buffer;
		QMutex mutex; 
		uint current_id=0;

	public:
		BoundBuffer(){};
		uint push(P params);
		std::tuple<uint, P> pop();
		bool isEmpty();
		void setFinished(uint id);
		bool isFinished(uint id);
};

template<class P>
uint BoundBuffer<P, void>::push(P params)
{
	cout<<"pushed "<<endl;
	mutex.lock();
	buffer.push(params);
	current_id++;
	mutex.unlock();
	return current_id;
}

template<class P>
std::tuple<uint, P> BoundBuffer<P, void>::pop()
{
	cout<<"poped"<<endl;
	uint cid = current_id;
	P params;
	mutex.lock();
	params = buffer.front();
	buffer.pop();
	current_id--;
	mutex.unlock();
	return std::make_tuple(cid, params);
}

template<class P>
bool BoundBuffer<P, void>::isEmpty()
{
	return buffer.empty();
}

#endif