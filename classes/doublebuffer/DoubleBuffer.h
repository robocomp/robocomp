//
// Created by robolab on 24/07/18.
// This class is a generic container for a thread safe doublebuffer used to transfer data between threads.
// For example, between the main thread of a component and the (threaded) middleware stubs
// Example of DoubleBuffer creation with default converters between input and output types:
//      decl: DoubleBuffer<RoboCompLaser::TLaserData, RoboCompLaser::TLaserData> laser_buffer;
//      use: laser_buffer.put(std::move(laserData));
//      use: auto ldata = laser_buffer.get();
// Example of DoubleBuffer creation with user-defined converter from input to output types
//      decl: DoubleBuffer<RoboCompLaser::TLaserData, RoboCompLaser::TLaserData> laser_buffer;
//      use:  laser_buffer.put(std::move(laserData), [](auto &I, auto &T){ for(auto &&i , I){ T.append(i/2);}});

#include <shared_mutex>
#include <mutex>
#include <vector>
#include <iostream>
#include <string>
#include <functional>
#include <atomic>

using namespace std::chrono_literals;

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
class DoubleBuffer
{
private:
    mutable std::shared_mutex bufferMutex;
    std::condition_variable_any cv;

    O bufferA;
    O bufferB;
    O &readBuffer;// = bufferA;
    O &writeBuffer;// = bufferB;
    //bool to_clear = false;
    std::atomic_bool empty;// = true;

public:
    DoubleBuffer() : readBuffer(bufferA), writeBuffer(bufferB), empty(true) {};

    void init() {}
    void clear() {}


    O get(std::chrono::milliseconds t = 200ms ) {
        std::shared_lock lock(bufferMutex);

        //cambiar esto cuando se implemente atomic wait/notify
        if (!cv.wait_until(bufferMutex,
                           std::chrono::steady_clock::now() + t ,
                           [this]() { return !empty.load();})){
            throw std::runtime_error("Timeout");
        }

        empty.store(true);
        return readBuffer;
    }

    std::optional<O> try_get()
    {
        if (empty.load())
            return {};

        std::shared_lock lock(bufferMutex);
        empty.store( true);
        return readBuffer;

    }

    void put(const I &d, std::function<void(const I &, O &)> t = empty_fn)
    {
        thread_local O temp;
        if (ItoO(d, temp, t))
        {
            std::unique_lock lock(bufferMutex);
            writeBuffer = std::move(temp);
            std::swap(writeBuffer, readBuffer);
            empty.store(false);
            cv.notify_all();
        }
    }

    void put(const I &&d, std::function<void(const I &, O &)> t = empty_fn)
    {
        thread_local O temp;

        if (ItoO(std::move(d), temp, t))
        {
            std::unique_lock lock(bufferMutex);
            writeBuffer = std::move(temp);
            std::swap(writeBuffer, readBuffer);
            empty.store(false);
            cv.notify_all();
        }
    }

private:
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
                oTypeData = O(iTypeData.begin(), iTypeData.end());
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

    bool ItoO(const I &&iTypeData, O &oTypeData, std::function<void(const I &, O &)> t = empty_fn)
    {
        //Si es el mismo tipo o es convertible de I a O
        if constexpr (std::is_same<I, O>::value || std::is_convertible<I, O>::value)
        {
            oTypeData = std::move(iTypeData);
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
                oTypeData = O(iTypeData.begin(), iTypeData.end());
            }
            else
            {
                //Si no es convertible necesitamos una función para convertir de I a O.
                //Comprobamos que el tipo de la función t es distinto del tipo de empty_fn (Las lambdas tienen tipos únicos).
                static_assert(!std::is_same<decltype(t), decltype(empty_fn)>::value, "A function needs to be implemented to transform ItoO");
                //Llamamos a la función
                t(iTypeData, oTypeData);
            }
        }
        else
        {
            //Lo mismo que antes.
            static_assert(!std::is_same<decltype(t), decltype(empty_fn)>::value, "A function needs to be implemented to transform ItoO");
            t(iTypeData, oTypeData);
        }
        return true;
    };
};