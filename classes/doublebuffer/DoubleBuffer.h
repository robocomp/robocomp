//
// Created by robolab on 24/07/18.
// This class is a generic container for a thread safe doublebuffer used to transfer data between threads.
// For example, between the main thread of a component and the (threaded) middleware stubs
// Example of DoubleBuffer creation with default converters between input and output types:
//      DoubleBuffer<RoboCompLaser::TLaserData, RoboCompLaser::TLaserData> laser_buffer;
// Example of DoubleBuffer creation with user-defined converter from input to output types
//

#include <shared_mutex>
#include <mutex>
#include <vector>
#include <iostream>
#include <string>
#include <functional>
#include <atomic>

//Función utilizada como argumento por defecto.
constexpr auto empty_fn = [](auto &I, auto &T) {};

template <typename T, typename = void>
struct is_iterable : std::false_type
{
};
template <typename T>
//especialización del template si tiene función begin y end.
struct is_iterable<T, std::void_t<decltype(std::declval<T>().begin()), decltype(std::declval<T>().end())>> : std::true_type
{
};

template <class I, class O>
class Converter
{
    public:
        bool ItoO(const I &iTypeData, O &oTypeData, std::function<void(const I &, O &)> t = empty_fn)
        {
            //Si es el mismo tipo o es convertible de I a O
            if constexpr (std::is_same<I, O>::value || std::is_convertible<I, O>::value)
            {
                oTypeData = iTypeData;
            }
            else if constexpr (is_iterable<I>::value && is_iterable<O>::value)
            { //Si es iterable, hay que ver si esto es suficiente

                //Conseguimos el tipo que contiene el iterador
                using I_T = typename std::decay<decltype(*iTypeData.begin())>::type;
                using O_T = typename std::decay<decltype(*oTypeData.begin())>::type;

                //Comprobamos si se puede convertir el I_T a O_T
                if constexpr (std::is_convertible<I_T, O_T>::value)
                {
                    // No funciona con copy;
                    oTypeData = std::vector<O_T>(iTypeData.begin(), iTypeData.end());
                }
                else
                {
                    //Si no es convertible necesitamos una función para convertir de I a O.
                    //Comprobamos que el tipo de la función t es distinto del tipo de empty_fn (Las lambdas tienen tipos únicos).
                    static_assert(!std::is_same<decltype(t), decltype(empty_fn)>::value, "A function needs to be implemented to transform OtoI and ItoO");
                    //Llamamos a la función
                    t(iTypeData, oTypeData);
                }
            }
            else
            {
                //Lo mismo que antes.
                static_assert(!std::is_same<decltype(t), decltype(empty_fn)>::value, "A function needs to be implemented to transform OtoI and ItoO");
                t(iTypeData, oTypeData);
            }
            return true;
        };

        bool OtoI(const O &oTypeData, I &iTypeData, std::function<void(I &, const O &)> t = empty_fn)
        {

            if constexpr (std::is_same<I, O>::value || std::is_convertible<I, O>::value)
                iTypeData = oTypeData;
            else if constexpr (is_iterable<I>::value && is_iterable<O>::value)
            {

                using I_T = typename std::decay<decltype(*iTypeData.begin())>::type;
                using O_T = typename std::decay<decltype(*oTypeData.begin())>::type;

                if constexpr (std::is_convertible<O_T, I_T>::value)
                {
                    iTypeData = std::vector<I_T>(oTypeData.begin(), oTypeData.end()); // No funciona con copy;
                }
                else
                {
                    static_assert(!std::is_same<decltype(t), decltype(empty_fn)>::value,
                                "A function needs to be implemented to transform OtoI and ItoO");
                    t(iTypeData, oTypeData);
                }
            }
            else
            {
                static_assert(!std::is_same<decltype(t), decltype(empty_fn)>::value,
                            "A function needs to be implemented to transform OtoI and ItoO");
                t(iTypeData, oTypeData);
            }
            return true;
        };
        //virtual bool clear(O & oTypeData)=0;
};

template <class I, class O>
class DoubleBuffer
{
    private:
        mutable std::shared_mutex bufferMutex;
        O bufferA; 
        O bufferB;
        O &readBuffer = bufferA;
        O &writeBuffer = bufferB;
        Converter<I, O> *converter; //Hay necesidad de que esto sea un puntero?
        bool to_clear = false;
        std::atomic_bool empty = true;

    public:
        DoubleBuffer()
        {
            converter = new Converter<I, O>();
        };

        void init(Converter<I,O> &_converter)
        {
            converter = &_converter;
            to_clear = false;
        }
        void clear()
        {
            // std::unique_lock lock(bufferMutex);
            // this->to_clear = true;
            // if( converter->clear(writeBuffer))
            // {
            // 	std::swap(writeBuffer,readBuffer);
            // }
        }

        std::optional<O> get()
        {
            if (empty.load())
                return {};
            std::shared_lock lock(bufferMutex);
            empty.store(true);
            return readBuffer;
        }

        void put(const I &d, std::function<void(const I &, O &)> t = empty_fn)
        {
            std::unique_lock lock(bufferMutex);
            if (converter->ItoO(d, writeBuffer, t))
            {
                std::swap(writeBuffer, readBuffer);
                empty.store(false);
            }
        }
};
