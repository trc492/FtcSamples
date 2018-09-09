/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package trclib;

import java.util.ArrayList;
import java.util.HashSet;

/**
 * This class provides methods for the callers to register/unregister cooperative multi-tasking tasks. It manages
 * these tasks and will work with the cooperative multi-tasking scheduler to run these tasks.
 */
public class TrcTaskMgr
{
    private static final String moduleName = "TrcTaskMgr";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private static final long taskNanoTimeThreshold = 10000000; // 10 msec

    /**
     * These are the task type TrcTaskMgr supports:
     */
    public enum TaskType
    {
        /**
         * START_TASK is called one time before a competition mode is about to start.
         */
        START_TASK(0),

        /**
         * STOP_TASK is called one time before a competition mode is about to end.
         */
        STOP_TASK(1),

        /**
         * PREPERIODIC_TASK is called periodically at a rate about 50Hz before runPeriodic().
         */
        PREPERIODIC_TASK(2),

        /**
         * POSTPERIODIC_TASK is called periodically at a rate about 50Hz after runPeriodic().
         */
        POSTPERIODIC_TASK(3),

        /**
         * PRECONTINUOUS_TASK is called periodically at a rate as fast as the scheduler is able to loop and is run
         * before runContinuous() typically 10 msec interval.
         */
        PRECONTINUOUS_TASK(4),

        /**
         * POSTCONTINUOUS_TASK is called periodically at a rate as fast as the schedule is able to loop and is run
         * after runContinuous() typically 10 msec interval.
         */
        POSTCONTINUOUS_TASK(5);

        public int value;

        private TaskType(int value)
        {
            this.value = value;
        }   //TaskType

    }   //enum TaskType

    /**
     * Any class that is registering as a cooperative multi-tasking task must implement this interface.
     */
    public interface Task
    {
        /**
         * This method is called at the appropriate time this task is registered for.
         *
         * StartTask:
         *  This contains code that will initialize the task before a competition mode is about to start.
         *  Typically, if the task is a robot subsystem, you may put last minute mode specific initialization code
         *  here. Most of the time, you don't need to register StartTask because all initialization is done in
         *  initRobot(). But sometimes, you may want to delay a certain initialization until right before competition
         *  starts. For example, you may want to reset the gyro heading right before competition starts to prevent
         *  drifting.
         *
         * StopTask:
         *  This contains code that will clean up the task before a competition mode is about to end. Typically,
         *  if the task is a robot subsystem, you may put code to stop the robot here. Most of the time, you don't
         *  need to register StopTask because the system will cut power to all the motors after a competition mode
         *  has ended.
         *
         * PrePeriodicTask:
         *  This contains code that will run before runPeriodic() is called. Typically, you will put code that deals
         *  with any input or sensor readings here so that the code in runPeriodic() will be able to make use of the
         *  input/sensor readings produced by the code here.
         *
         * PostPeriodicTask:
         *  This contains code that will run after runPeriodic() is called. Typically, you will put code that deals
         *  with actions such as programming the motors here.
         *
         * PreContinuousTask:
         *  This contains code that will run before runContinuous() is called. Typically, you will put code that deals
         *  with any input or sensor readings that requires more frequent processing here such as integrating the gyro
         *  rotation rate to heading.
         *
         * PostContinuousTask:
         *  This contains code that will run after runContinuous() is called. Typically, you will put code that deals
         *  with actions that requires more frequent processing.
         *
         * @param taskType specifies the type of task being run. This may be useful for handling multiple task types.
         * @param runMode specifies the competition mode that is about to end (e.g. Autonomous, TeleOp, Test).
         */
        void runTask(TaskType taskType, TrcRobot.RunMode runMode);

    }   //interface Task

    /**
     * This class implements TaskObject that will be created whenever a class is registered as a cooperative
     * multi-tasking task. The created task objects will be entered into an array list of task objects to be
     * scheduled by the scheduler.
     */
    public static class TaskObject
    {
        private HashSet<TaskType> taskTypes;
        private final String taskName;
        private Task task;
        private long[] taskTotalNanoTimes = new long[TaskType.values().length];
        private int[] taskTimeSlotCounts = new int[TaskType.values().length];

        /**
         * Constructor: Creates an instance of the task object with the given name
         * and the given task type.
         *
         * @param taskName specifies the instance name of the task.
         * @param task specifies the object that implements the TrcTaskMgr.Task interface.
         */
        private TaskObject(final String taskName, Task task)
        {
            taskTypes = new HashSet<>();
            this.taskName = taskName;
            this.task = task;
            for (int i = 0; i < TaskType.values().length; i++)
            {
                taskTotalNanoTimes[i] = 0;
                taskTimeSlotCounts[i] = 0;
            }
        }   //TaskObject

        /**
         * This method returns the instance name of the task.
         *
         * @return instance name of the class.
         */
        public String toString()
        {
            return taskName;
        }   //toString

        /**
         * This method adds the given task type to the task object.
         *
         * @param type specifies the task type.
         * @return true if successful, false if the task with that task type is already registered in the task list.
         */
        public boolean registerTask(TaskType type)
        {
            boolean added = false;

            if (!taskTypes.contains(type))
            {
                added = taskTypes.add(type);
            }

            return added;
        }   //registerTask

        /**
         * This method removes the given task type from the task object.
         *
         * @param type specifies the task type.
         * @return true if successful, false if the task with that type is not found the task list.
         */
        public boolean unregisterTask(TaskType type)
        {
            return taskTypes.remove(type);
        }   //unregisterTask

        /**
         * This method checks if the given task is associated with this task object.
         *
         * @param taskObj specifies the task object to be checked against.
         * @return true if it is the same task, false otherwise.
         */
        public boolean isSame(TaskObject taskObj)
        {
            return taskObj == this;
        }   //isSame

        /**
         * This method checks if the given task type is registered with this task object.
         *
         * @param type specifies the task type to be checked against.
         * @return true if this task is registered as the given type, false otherwise.
         */
        public boolean hasType(TaskType type)
        {
            return taskTypes.contains(type);
        }   //hasType

        /**
         * This method checks if this task object has no registered task type.
         *
         * @return true if this task has no task type, false otherwise.
         */
        public boolean hasNoType()
        {
            return taskTypes.isEmpty();
        }   //hasNoType

        /**
         * This method returns the class object that was associated with this task object.
         *
         * @return class object associated with the task.
         */
        public Task getTask()
        {
            return task;
        }   //getTask

    }   //class TaskObject

    private static TrcTaskMgr instance = null;
    private ArrayList<TaskObject> taskList = new ArrayList<>();

    /**
     * Constructor: Creates an instance of the task manager. Typically, there is only one global instance of
     * task manager. Any class that needs to call task manager can call its static method getInstance().
     */
    public TrcTaskMgr()
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        instance = this;
    }   //TrcTaskMgr

    /**
     * This method returns the global instance of TrcTaskMgr.
     *
     * @return global instance of TrcTaskMgr.
     */
    public static TrcTaskMgr getInstance()
    {
        return instance;
    }   //getInstance

    public TaskObject createTask(final String taskName, Task task)
    {
        final String funcName = "createTask";
        TaskObject taskObj = null;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "taskName=%s", taskName);
        }

        taskObj = new TaskObject(taskName, task);
        taskList.add(taskObj);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", taskObj);
        }

        return taskObj;
    }   //createTask

    public boolean removeTask(TaskObject taskObj)
    {
        return taskList.remove(taskObj);
    }   //removeTask

    /**
     * This method enumerates the task list and calls all the tasks that matches the given task type.
     *
     * @param type specifies the task type to be executed.
     * @param mode specifies the robot run mode.
     */
    public void executeTaskType(TaskType type, TrcRobot.RunMode mode)
    {
        final String funcName = "executeTaskType";

        for (int i = 0; i < taskList.size(); i++)
        {
            TaskObject taskObj = taskList.get(i);
            if (taskObj.hasType(type))
            {
                Task task = taskObj.getTask();
                long startNanoTime = TrcUtil.getCurrentTimeNanos();

                switch (type)
                {
                    case START_TASK:
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Executing StartTask %s", taskObj.toString());
                        }
                        task.runTask(TaskType.START_TASK, mode);
                        break;

                    case STOP_TASK:
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Executing StopTask %s", taskObj.toString());
                        }
                        task.runTask(TaskType.STOP_TASK, mode);
                        break;

                    case PREPERIODIC_TASK:
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Executing PrePeriodicTask %s", taskObj.toString());
                        }
                        task.runTask(TaskType.PREPERIODIC_TASK, mode);
                        break;

                    case POSTPERIODIC_TASK:
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Executing PostPeriodicTask %s", taskObj.toString());
                        }
                        task.runTask(TaskType.POSTPERIODIC_TASK, mode);
                        break;

                    case PRECONTINUOUS_TASK:
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Executing PreContinuousTask %s", taskObj.toString());
                        }
                        task.runTask(TaskType.PRECONTINUOUS_TASK, mode);
                        break;

                    case POSTCONTINUOUS_TASK:
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Executing PostContinuousTask %s", taskObj.toString());
                        }
                        task.runTask(TaskType.POSTCONTINUOUS_TASK, mode);
                        break;
                }

                long elapsedTime = TrcUtil.getCurrentTimeNanos() - startNanoTime;
                taskObj.taskTotalNanoTimes[type.value] += elapsedTime;
                taskObj.taskTimeSlotCounts[type.value]++;

                if (debugEnabled)
                {
                    if (elapsedTime > taskNanoTimeThreshold)
                    {
                        dbgTrace.traceWarn(funcName, "%s.%s takes too long (%.3f)",
                            taskObj.taskName, type, elapsedTime/1000000000.0);
                    }
                }
            }
        }
    }   //executeTaskType

    /**
     * This method prints the performance metrics of all tasks with the given tracer.
     *
     * @param tracer specifies the tracer to be used for printing the task performance metrics.
     */
    public void printTaskPerformanceMetrics(TrcDbgTrace tracer)
    {
        for (TaskObject taskObj: taskList)
        {
            tracer.traceInfo(
                    "TaskPerformance",
                    "%16s: PrePeriodic=%.6f, PostPeriodic=%.6f, PreContinuous=%.6f, PostContinous=%.6f",
                    taskObj.taskName,
                    (double)taskObj.taskTotalNanoTimes[TaskType.PREPERIODIC_TASK.value]/
                            taskObj.taskTimeSlotCounts[TaskType.PREPERIODIC_TASK.value]/1000000000,
                    (double)taskObj.taskTotalNanoTimes[TaskType.POSTPERIODIC_TASK.value]/
                            taskObj.taskTimeSlotCounts[TaskType.POSTPERIODIC_TASK.value]/1000000000,
                    (double)taskObj.taskTotalNanoTimes[TaskType.PRECONTINUOUS_TASK.value]/
                            taskObj.taskTimeSlotCounts[TaskType.PRECONTINUOUS_TASK.value]/1000000000,
                    (double)taskObj.taskTotalNanoTimes[TaskType.POSTCONTINUOUS_TASK.value]/
                            taskObj.taskTimeSlotCounts[TaskType.POSTCONTINUOUS_TASK.value]/1000000000);
        }
    }   //printTaskPerformanceMetrics

}   //class TaskMgr
