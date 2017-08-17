
#ifndef BOUNDBUFFER_H
#define BOUNDBUFFER_H

#include <tuple>
#include <queue>


//
// Buffer for a non void return type call
//
template<class P, class R=void>
class BoundBuffer
{
	private:
		std::queue<std::tuple<uint, P> > buffer;
		QMutex mutex;
		uint current_id=0;
		std::map<uint, R> results;

	public:
		BoundBuffer(){};
		uint push(P params);
		std::tuple<uint, P> pop();
		R result(uint cid); // only use when the function has a non-void return type
		bool isEmpty();
		void setFinished(uint id, R result);
		bool isFinished(uint id);
};

template<class P, class R>
uint BoundBuffer<P, R>::push(P params)
{
	int cid=0;
	mutex.lock();
	cid = current_id;
	buffer.push(std::make_tuple(cid, params));
	current_id++;
	mutex.unlock();
	return cid;
}

template<class P, class R>
std::tuple<uint, P> BoundBuffer<P, R>::pop()
{
	std::tuple<uint, P> call_info;	
	mutex.lock();
	call_info = buffer.front();
	buffer.pop();
	mutex.unlock();
	return call_info;
}

template<class P, class R>
R BoundBuffer<P, R>::result(uint cid)
{
	R result ;
	if(isFinished(cid)){
		result = results[cid];
	    results.erase(cid);
	} else {
		throw "Aecssing call result before it's ready";
	}
	return result;
}

template<class P, class R>
bool BoundBuffer<P, R>::isEmpty()
{
	return buffer.empty();
}

template<class P, class R>
void BoundBuffer<P, R>::setFinished(uint cid, R result)
{
	results[cid] = result;
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
		std::queue<std::tuple<uint, P> > buffer;
		QMutex mutex; 
		uint current_id=0;

	public:
		BoundBuffer(){};
		uint push(P params);
		std::tuple<uint, P> pop();
		bool isEmpty();
};

template<class P>
uint BoundBuffer<P, void>::push(P params)
{
	int cid=0;
	mutex.lock();
	cid = current_id;
	buffer.push(std::make_tuple(cid, params));
	current_id++;
	mutex.unlock();
	return cid;
}

template<class P>
std::tuple<uint, P> BoundBuffer<P, void>::pop()
{
	std::tuple<uint, P> call_info;	
	mutex.lock();
	call_info = buffer.front();
	buffer.pop();
	mutex.unlock();
	return call_info;
}

template<class P>
bool BoundBuffer<P, void>::isEmpty()
{
	return buffer.empty();
}

#endif