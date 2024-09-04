

Memory Segments: Stack vs Heap
Stack Memory:
  Linear memory area, managed in a Last In First Out (LIFO) manner.
  Used for function management, storing local variables, function return addresses, etc.
  Each function call allocates a specific stack region (stack frame).
  Stack size is limited and cannot be modified.
  Automatically managed by the CPU.
  When the function terminates, its stack is deleted, including local variables.

- Heap Memory:
  Non-linear memory area used for dynamic memory allocation.
  Larger than the stack and can grow as needed, limited only by system resources (physical/virtual memory).
  Requires manual management.
  Memory allocated on the heap persists until explicitly freed or until the program ends.
  Suitable for dynamic data structures (e.g., objects, arrays, lists, trees, and graphs).


Parallel Computing
- Parallel computing involves decomposing a process into multiple software modules (threads) that run simultaneously and independently.
Typically utilizes multiple processors or cores working simultaneously.
- Multithreading and OS
Multithreading is managed by the operating system (OS).
The OS is responsible for creating, scheduling, and terminating threads.
Modern OS provides APIs for multithreading, allowing concurrent task execution.
The OS scheduler determines the execution order of threads and ensures efficient CPU utilization.
Context switching is managed by the OS, storing and restoring the state of a thread.


- Scheduler Types
  Cooperative Round-Robin Scheduling:
  Tasks voluntarily yield the CPU.
  Scheduler selects the next task in a round-robin fashion (queue ordered by priorities).
  
  Preemptive Round-Robin Scheduling:
  Each task is given a fixed time slice (quantum).
  After the time slice, the task is preempted, and the next task in the round-robin queue is allocated CPU.
  Higher-priority tasks can preempt lower-priority tasks.


Thread States
  Ready: The default state after task creation.
  Running: The task is allocated CPU and its instructions are being executed.
  Blocked: The task is waiting for a specific condition or event to occur.
  Suspended: The task is paused by an external action and resumes only when explicitly instructed.
  In both blocked and suspended states, the task remains in memory but does not use CPU.
  
FreeRTOS Overview
*  System Timer
In FreeRTOS, it's recommended to change the system base timer from SysTick to another timer when using both HAL and FreeRTOS.
* CMSIS-OS API
The CMSIS-OS API is part of the ARM Cortex Microcontroller Software Interface Standard, offering standardized functions for RTOS on ARM Cortex microcontrollers.
FreeRTOS is supported by CMSIS-OS, making it easier to write portable embedded system code.


- Memory Management
Each task has its own stack (used for storing local variables and task-related info).
The stack can be allocated from the heap or from other available RAM.
FreeRTOS provides several memory management schemes for managing the FreeRTOS heap:
heap_1.c: Simple, no memory freeing support.
heap_2.c: Supports freeing memory, but does not coalesce free blocks.
heap_3.c: Wraps standard malloc() and free().
heap_4.c: Coalesces adjacent free blocks, reducing fragmentation.
heap_5.c: Allows memory allocation across multiple non-contiguous regions.


- FreeRTOS Interrupts
  
  - PendSV Interrupt (Pending Supervisor Call)
  Used for context switching.
  Triggered when a context switch is needed (e.g., a task yields or a higher-priority task is ready).
  Low priority: Ensures it runs only when no higher-priority interrupts are pending.
  
  - SVC Interrupt (Supervisor Call)
  Used to request privileged OS operations (e.g., system calls).
  Higher priority than PendSV, but lower than hardware interrupts.

  - SysTick Interrupt
  Software interrupt triggered at the end of a countdown.
  Can be used for counting time slices.
  SysTick and PendSV have the lowest NVIC priorities.


Interrupt Types
  - Non-OS IRQs: These interrupt handlers cannot execute OS API functions (i.e., no system calls).
  - OS IRQs: These interrupts can be configured to interact with the RTOS.

Context Switching
  The process of switching CPU allocation from one thread to another.
  Managed by the OS to ensure tasks resume execution from the correct point.



FreeRTOS Tasks
A task in FreeRTOS is a function that runs in a loop, independently and simultaneously with other threads.
Task Components
Stack:
A memory area that stores parameters, local variables, temporary calculations, and return addresses for function calls.
Task Control Block (TCB):
A data structure that stores status information about the task (e.g., task priority, stack pointer, etc.).
Task Scheduling
The scheduler maintains a list of all tasks in various states (e.g., ready, blocked).
When a task is deleted, it is removed from the scheduler's lists, and its memory is freed during the next execution of the idle task.
Task Creation
Task Attribute Structure
Tasks are created using an attribute structure of type osThreadAttr_t, which holds basic task information:
Pointers to the stack and TCB.
Stack size, priority, and other relevant details.
Task Attribute Initialization Example
c
Copy code
const osThreadAttr_t Task_attributes = {
    .name = "task name",
    .stack_size = stack_size,
    .priority = (osPriority_t) osPriority,
};
Task Creation Function
After defining the task attributes, a task is created using the osThreadNew function:
c
Copy code
TaskHandle = osThreadNew(Task_function, arguments, &Task_attributes);
Task function: The function that represents the task, running in a loop. Its type is osThreadFunc_t:
c
Copy code
typedef void (*osThreadFunc_t) (void *argument);

void Task_function(void *argument) {
    // Thread code here
}
TaskHandle: A variable of type osThreadId_t that holds the task's ID. If TaskHandle is NULL, it indicates an error in task creation, typically related to memory issues.
c
Copy code
osThreadId_t TaskHandle;
Argument: A generic pointer (void*) passed to the task function, allowing for flexibility in passing different data types.
Task Delays
osDelay():

Moves the task to a blocked state for a specified duration, putting it in the list of delayed tasks.
osDelayUntil():

Stops task execution until a specified time has passed since the last update of the time reference. It is useful for ensuring task periodicity.
HAL_Delay():

Unlike osDelay(), HAL_Delay() creates an empty loop that consumes CPU resources, preventing other tasks from running.
FreeRTOS Idle Task
Once the FreeRTOS scheduler starts, program flow is entirely managed by the RTOS.
Idle Task:
Automatically created when the scheduler starts.
Has the lowest priority and runs only when there are no other tasks in the ready state.
One of its functions is to delete the memory resources of deleted tasks.





Queues in FreeRTOS
Queue: A linear memory region used to transfer data between tasks. The data can be managed as FIFO (First In, First Out) or LIFO (Last In, First Out).
Queue Characteristics
Type: The type of data the queue stores, defined during its creation.
Message Queue: Stores single data values.
Mail Queue: Stores data structures.
Size: The number of data items the queue can store.
Queue Operations
Send/Receive Operations: These are blocking operations:
If sending/receiving is immediately possible, the task proceeds.
If not possible, the task enters the blocked state for a specified timeout duration, allowing the CPU to execute other tasks.
If multiple tasks are blocked and waiting for access to the queue, the task with the highest priority is granted access. If they have the same priority, access is granted based on waiting time.
Reasons Tasks Get Blocked by Queues
Sender: Blocked when there is no free space in the queue.
Receiver: Blocked when there is no data in the queue.
Queue Creation (CMSIS-RTOS v2)
Define a queue attribute structure:

c
Copy code
const osMessageQueueAttr_t queue_attr = { .name = "QueueName" };
Create the queue:

c
Copy code
osMessageQueueId_t queue_handle;
queue_handle = osMessageQueueNew(msg_count, msg_size, &queue_attr);
Task Synchronization: Semaphores
Semaphores are used to synchronize access to shared resources to avoid conflicts during concurrent operations.

Semaphore Types
Binary Semaphore / Mutex:
Has two states: locked and unlocked.
Used to manage resources that can only be accessed by one task at a time.
Initially unlocked.
Semaphore Operation:
If a task wants to access the resource, it checks the semaphore's state:
If unlocked, the task locks the semaphore, accesses the resource, and unlocks the semaphore after finishing.
If locked, the task enters the blocked state and waits for the semaphore to be unlocked.
Counting Semaphore:
Manages resources that can be accessed by multiple tasks simultaneously.
The semaphore's value equals the number of tasks that can access the resource at once.
Tasks decrement the semaphore value when they acquire it and increment it when they release the resource.
Semaphore Creation (CMSIS-RTOS v2)
Define the semaphore attribute structure:

c
Copy code
const osSemaphoreAttr_t semaphore_attr = { .name = "SemaphoreName" };
Create the semaphore:

c
Copy code
osSemaphoreId_t semaphore_handle;
semaphore_handle = osSemaphoreNew(max_count, initial_count, &semaphore_attr);
Mutex: Special Semaphore for Ownership Control
A mutex is similar to a binary semaphore but adds ownership control:
It can only be released by the task that acquired it.
Handles priority inversion through priority inheritance.
In the case of deadlock (when two tasks wait for each other’s mutexes), tasks remain in blocked states.
Timers in FreeRTOS
Software Timer: Executes a callback function after a predefined period (single-shot timer) or at regular intervals (periodic timer).
Managed by the RTOS scheduler through a timer service task and a queue.
Timer Creation (CMSIS-RTOS v2)
The process follows the same steps as semaphores and queues.

The priority of the timer service task should be high enough to prevent excessive preemption by other tasks, ensuring timely execution of timer callback functions.

Interrupts
Definition: An interrupt is a hardware or software signal that temporarily halts the main program’s execution and triggers a specific function called the Interrupt Service Routine (ISR) or interrupt handler.
Purpose: Interrupts enable the system to respond to important events immediately, such as hardware failures or time-sensitive operations.
Events
Definition: An event is a software signal that indicates that a certain condition has been met. Unlike interrupts, events are processed by the main program or RTOS scheduler and do not interrupt the current execution.
Key Points:
Events are purely software-based and are not tied to hardware registers.
Events are used in synchronization and coordination between tasks or threads.
Event Groups (CMSIS-RTOS v2)
Definition: An event group is a data structure that consists of 32 bits (or sometimes 16 bits). Each bit (excluding some control bits) can represent an event.

How it works:

Each bit or group of bits can be associated with an event. When an event occurs, the corresponding bit(s) is set.
A task can check the status of these bits to see if an event has occurred. If the event has occurred, the task continues; if not, the task enters the blocked state, waiting for the event to happen.
APIs:

Set Bits: The event group bits can be set by the system or the programmer to indicate that an event has occurred.
Clear Bits: Manually clear specified event bits when needed.
Event Group Creation (CMSIS-RTOS v2):
Define an event group attribute structure:
c
Copy code
const osEventFlagsAttr_t event_flags_attr = { .name = "EventGroupName" };
Create the event group:
c
Copy code
osEventFlagsId_t eventgroup_handler;
eventgroup_handler = osEventFlagsNew(&event_flags_attr);
Note: The function xEventGroupSetBits in FreeRTOS may be considered non-deterministic because its execution time varies depending on factors like the number of tasks waiting on the event group. Therefore, it may be hard to predict the Worst Case Execution Time (WCET).
Thread Flags
Definition: Each thread (task) in FreeRTOS has a 32-bit thread flag. Each bit, excluding the most significant bit (MSB), can represent an event.

How it works:

Similar to event groups, thread flags allow programmers to associate specific bits with events. When an event occurs, the corresponding bit(s) is set.
Tasks can check these thread flags for the status of events. If the event occurs, the task continues execution; if not, it enters the blocked state until the event happens.
Important Details:

A thread flag can have a maximum of 24 flags (excluding the control bits).
Functions are available to manually clear bits when needed.
Example Workflow:
Event Occurs: Set a specific bit in the thread flag when an event occurs.
Task Check: The task checks the thread flag to determine if the event has occurred.
Blocked State: If the event hasn't occurred, the task is blocked until the event flag is set.


Idle Task
Definition: The idle task is executed when no other tasks are in the Ready state.

Idle Period: The time when the idle task is running is referred to as the idle period, indicating that no other tasks are ready for execution.

Low Power Opportunities: During the idle period, it's an ideal time to put the system into low power modes to conserve energy.

Modifications: Although you cannot modify the idle task itself, FreeRTOS provides an Idle Hook mechanism. The Idle Hook is a callback function that is executed during each iteration of the idle task. It allows developers to perform actions during idle time, such as entering low power modes.

Wake-up Conditions: The system is awakened from idle by:

A task of higher priority than idle transitioning to the Ready state.
An interrupt, which executes its Interrupt Service Routine (ISR).
Tickless Idle Mode
Definition: Tickless Idle Mode is a low-power feature in FreeRTOS that reduces unnecessary wake-ups by stopping the tick interrupts (usually generated by a timer like SysTick or software interrupts) during idle periods.

Enabling Tickless Idle Mode: To enable Tickless Idle Mode, define configUSE_TICKLESS_IDLE as 1 in FreeRTOSConfig.h.

c
Copy code
#define configUSE_TICKLESS_IDLE 1
Behavior:

When the Idle Task is the only task able to run, and:
At least n complete tick periods will pass before any task changes into the Ready state, where n is determined by the timeouts of blocked tasks.
The system enters tickless idle mode, where the kernel will call the function portSUPPRESS_TICKS_AND_SLEEP(). This stops the tick interrupts and allows the system to remain in a low power state until it needs to wake up.

configEXPECTED_IDLE_TIME_BEFORE_SLEEP: This setting specifies the minimum idle time (in tick periods) before the system enters tickless idle mode.

c
Copy code
#define configEXPECTED_IDLE_TIME_BEFORE_SLEEP <value>
Wake-up Conditions: The system wakes up when:

A higher-priority task than the idle task transitions to the Ready state.
An interrupt (other than a tick interrupt) occurs.
Low Power Timer and Tickless Mode
Low Power Timer: In tickless idle mode, a low-power timer (other than the standard tick timer) should be used to keep track of time while the system is in low-power mode.

NVIC Configuration: The NVIC (Nested Vectored Interrupt Controller) must be configured to handle interrupts from the low-power timer that can wake up the system when needed.

Pre-Sleep and Post-Sleep Processing
FreeRTOS provides two functions for managing power transitions during tickless idle:

Pre-Sleep Processing: This function is used to prepare the system for entering low-power mode. Common actions include suspending the HAL (Hardware Abstraction Layer) timer and activating the low-power timer.

c
Copy code
void PreSleepProcessing(uint32_t expectedIdleTime);
Post-Sleep Processing: This function is called when the system wakes up, to restore the HAL timer and suspend the low-power timer.

c
Copy code
void PostSleepProcessing(uint32_t expectedIdleTime);
User-Defined Tickless Idle Mode
If custom tickless behavior is required, it can be implemented by defining configUSE_TICKLESS_IDLE as 2 in FreeRTOSConfig.h. This allows the developer to manage tickless idle functionality manually.

c
Copy code
#define configUSE_TICKLESS_IDLE 2
