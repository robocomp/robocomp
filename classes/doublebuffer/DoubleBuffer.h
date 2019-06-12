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

template <class I, class O, class C> class DoubleBuffer
{
private:
    mutable std::mutex bufferMutex;
    O bufferA, bufferB;
    O &readBuffer = bufferA;
    O &writeBuffer = bufferB;
    C *converter;
    bool to_clear=false;
    //, *readBuffer, bufferB;

//    void resize(std::size_t size_)
//    {
//        if (size_!= size)
//        {
//            bufferA.resize(size_);
//            bufferB.resize(size_);
//            size = size_;
//        }
//    }

public:
//    std::size_t size=0;
    DoubleBuffer()
    {
//        resize(640*480);
    };

//    void init(/*std::size_t v_size, */C &converter)
//    {
//        resize(v_size);
//        this->converter = &converter;
//        to_clear=false;
//    }

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

//    inline typename O::value_type& operator[](int i) { return (writeBuffer)[i]; };

    void get(O &oData) const
    {
        std::lock_guard<std::mutex> lock(bufferMutex);
        oData = readBuffer;
    }

//    O& getNextPtr()
//    {
//        return writeBuffer;
//    }

    void put(const I &d, std::size_t data_size)
    {
        if( converter->ItoO(d, writeBuffer))
        {
            std::lock_guard<std::mutex> lock(bufferMutex);
            std::swap(writeBuffer, readBuffer);
        }
    }

//    void put_stdcopy(const I &d, std::size_t data_size) {
//        if (d.is_valid())
//        {
//            this->resize(d.width() * d.height()*data_size);
//            std::copy(d.data(), &d.data()[0]+(500), begin(*writeBuffer));
//        }
//    }
};

#endif //PROJECT_DOUBLEBUFFER_H


