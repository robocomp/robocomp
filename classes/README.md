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
· use: `auto ldata = laser_buffer.get();`  

Example of DoubleBuffer creation with user-defined converter from input to output types
· decl: `DoubleBuffer<RoboCompLaser::TLaserData, RoboCompLaser::TLaserData> laser_buffer;`  
· use:  `laser_buffer.put(std::move(laserData), [](auto &&I, auto &T){ for(auto &&i , I){ T.append(i/2);}});`  
· decl: `auto rgb_buffer = new DoubleBuffer<std::vector<std::uint8_t>, cv:::Mat>(std::chrono::milliseconds(100));`

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

### Example 1
```c++
void print_a(int a) {
   std::cout << a << std::endl;
}
//...
ThreadPool worker(1); //Creates a threadpool with one worker thread.
wroker.spawn task(print_a, 24); //Add task to the work queue.
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
   //this is not ok. The tasks can be executed when vec is no longer valid.
   worker.spawn_task([](){
       auto x = 0;
       for (auto &e : vec) {
           x+=e;
       }
   });
    //
   //this is not ok. The tasks can be executed when vec is no longer valid.
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

### Example 5 
When we want to execute a member function of an object we have to do it like this.
```c++
std::string s = "aaa";
auto f = tp.spawn_task_waitable(empty, s);
//or if need to use this.
tp.spawn_task(DSRGraph::join_delta_node_att, this, ...);
//and in a lambda:
tp.spawn_task([this]() { ... });
```