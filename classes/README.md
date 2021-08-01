# Robocomp support classes
During the years of Robocomp development, a series of classes and libraries have been generated 
that are of common use among the components and tools that compose it.

In this directory you can find these classes defined.


## [DoubleBuffer](./DoubleBuffer)
This class is a generic container for a thread-safe, timed, threaded doublebuffer used to transfer data between threads.
For example, between the main thread of a component and the (threaded) middleware stubs.  

Example of DoubleBuffer creation with default converters between input and output types:  
· decl: `DoubleBuffer<RoboCompLaser::TLaserData, RoboCompLaser::TLaserData> laser_buffer;`  
· use: `laser_buffer.put(std::move(laserData));`  
· use: `auto ldata = laser_buffer.get();` Returns the object stored in DoubleBuffer. If there is no object, waits until the timeout indicated in the DoubleBuffer constructor (default 250ms).  
· use: `auto ldata = laser_buffer.try_get();` Returns the object stored in DoubleBuffer in a std::optional. If there is no object, it return a std::nullopt.

Example of DoubleBuffer creation with user-defined converter from input to output types
· decl: `DoubleBuffer<RoboCompLaser::TLaserData, RoboCompLaser::TLaserData> laser_buffer;`  
· use:  `laser_buffer.put(std::move(laserData), [](auto &&I, auto &T){ for(auto &&i , I){ T.append(i/2);}});`  
· decl: `auto rgb_buffer = new DoubleBuffer<std::vector<std::uint8_t>, cv:::Mat>(std::chrono::milliseconds(100));`

DoubleBuffer only accepts rvalue references. You can get an rvalue by moving the object (invalidating de old object) `laser_buffer.put(std::move(laserData))`, or by calling a copy constructor in the function `laser_buffer.put(RoboCompLaser::TLaserData{laserData})`.

## [threadpool](./threadpool)
This class is a simple threadpool with a thread-safe interface.
The interface offers methods to execute either asynchronous tasks from which
no result is expected using the spawn_task method, or asynchronous tasks from
which a result is expected, which return a future object (https://en.cppreference.com/w/cpp/thread/future)
with the method spawn_task_waitable.

It can be used as a set of worker threads for executing tasks sent by a thread that read periodically
from the network (currently used in dsr).

It is also used to launch tasks that the user does not need to wait for to complete (Used in doublebuffer).
Another use would be to spawn multiple tasks that can be executed in parallel and wait for all of them to
be completed after that.

ThreadPool only accepts rvalue references to avoid lifetime issues with the objects passed to the threadpool. You can get an rvalue by moving the object (invalidating de old object) or by calling a copy constructor in the function or the lambda capture. When using lambdas to spawn a task, you should follow the same rules with the exception of pointers (move unique_ptrs, clone shared_ptrs, handling raw pointers properly) and the situations indicated in the examples below.

### Example 1
```c++
void print_a(std::string  &&a) {
   std::cout << a << std::endl;
}

std::string str = "hello";
//...
ThreadPool worker(1); //Creates a threadpool with one worker thread.
worker.spawn_task(print_a, std::string{str}); //Add task to the work queue. This is ok because we copy the object.
worker.spawn_task(print_a, std::move(str)); //Add task to the work queue.  This is ok because we move the object.
worker.spawn_task(print_a, "hello"); //Add task to the work queue.  This is ok because literals can be used as rvalues.

worker.spawn_task(print_a, str); // This is NOT ok because the invocation of the funcion is asynchronous and the funcion may execute out of the scope of the object str.

```

### Example 2
When using a lambda is important to capture by this, move or copy the values
that are going to be used to avoid problems with objets lifetimes.
```c++
//...
void do_something(const std::vector<int> &vec) {
    //   this is ok.
    worker.spawn_task([vec = vec](){
       auto x = 0;
       for (auto &e : vec) {
           x+=e;
       }
    });
   //
   //this is not ok. The task may be executed when vec is no longer valid.
   worker.spawn_task([&](){
       auto x = 0;
       for (auto &e : vec) {
           x+=e;
       }
   });
    //
   //this is not ok. The task can be executed when vec is no longer valid.
   worker.spawn_task([](const std::vector<int> &vec){
       auto x = 0;
       for (auto &e : vec) {
           x+=e;
       }
   }, vec);
}
```
### Example 3: 
Whe should move values when possible. We change the ownership of the objects so they not are destroyed.
```c++
BifObject bo;
worker.spawn_task([std::move(bo)]() mutable {
     //...;
});
//or
worker.spawn_task([](BigObject &&bo) {
    //...;
}, std::move(bo));
```

### Example 4
Waitable tasks. 

When you want to execute a function asynchronously and you want to wait for it to finish before continuing you can use spawn_task_waitable. This method also allows you to access the results of the function execution.
```c++
ThreadPool tp();

std::vector<std::future<int>> futs;
for (int i = 0; i < 10; i++ ) {
    futs.push_back(tp.spawn_task_waitable([]() -> int {
        // ...
    }));
}
// Wait until all tasks return.
for (auto &f : futs) {
    f.get();
}

```

Although it has been said that it should not be possible to use lvalues in the threadpool. It is ok to capture variables as references in lambdas if you wait for its finish in the same scope where it was created.

``` c++
{ //Scope starts here.

std::string big_string = "...";
ThreadPool tp();
auto future = tp.spawn_task_waitable([&]()  {
    //Do something with big string.

}));

//Do something while the task is running.

future.wait(); //This is ok. We are ensuring that the task will end before big_string is invalid.

} //Scope end here.

future.wait(); //This is Undefined Behavior and is NOT ok. If the task has not finished its execution before exiting the big_string scope, big_string is an invalid reference.
```

### Example 5 
When we want to execute a member function of an object we have to do it like this.
```c++
std::string s = "aaa";
auto f = tp.spawn_task_waitable(empty, s);
//or if need to use this.
tp.spawn_task(DSRGraph::join_delta_node_att, this, ...);
//and in a lambda:
tp.spawn_task([this]() { this->join_delta_node_att(...); });
```