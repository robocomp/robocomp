//
// Created by robolab on 24/07/18.
//

#ifndef PROJECT_DOUBLEBUFFER_H
#define PROJECT_DOUBLEBUFFER_H
#include <mutex>

template <class I, class O> class Converter
{
    public:
        virtual bool ItoO(const I & iTypeData, O &oTypeData)=0;
        virtual bool OtoI(const O & oTypeData, I &iTypeData)=0;
		virtual bool clear(O & oTypeData)=0;
};

template <class I, class O> class ConverterDefault : Converter<I,O>
{
    bool ItoO(const I & iTypeData, O &oTypeData) { std::copy(iTypeData.begin(), iTypeData.end(), oTypeData.begin()); return true;};
    bool OtoI(const O & oTypeData, I &iTypeData) { std::copy(oTypeData.begin(), oTypeData.end(), iTypeData.begin());  return true;};
	bool clear(O & oTypeData){ oTypeData.clear() ; return true;};
};

template <class I, class O, class C = ConverterDefault<I,O>> class DoubleBuffer
{
private:
    mutable std::mutex bufferMutex;
    O bufferA, bufferB;
    O &readBuffer = bufferA;
    O &writeBuffer = bufferB;
    C *converter;
    bool to_clear=false;
    
public:
    DoubleBuffer(){};
    void init(C& _converter)
    {
        converter = &_converter;
		to_clear=false;
    }
	void clear()
	{
    	this->to_clear = true;
		if( converter->clear(writeBuffer))
		{
			std::lock_guard<std::mutex> lock(bufferMutex);
			std::swap(writeBuffer,readBuffer);
		}
	}
    void get(O &oData) const
    {
        std::lock_guard<std::mutex> lock(bufferMutex);
        oData = readBuffer;
    }
    void put(const I &d)
    {
        if( converter->ItoO(d, writeBuffer))
        {
            std::lock_guard<std::mutex> lock(bufferMutex);
            std::swap(writeBuffer, readBuffer);
        }
    }
};

#endif //PROJECT_DOUBLEBUFFER_H


