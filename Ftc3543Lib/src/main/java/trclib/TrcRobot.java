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

/**
 * This class is a container. It contains some robot related types and interfaces.
 */
public class TrcRobot
{
    public enum RunMode
    {
        /**
         * The robot is only in this mode very briefly during initialization.
         */
        INVALID_MODE,

        /**
         * In FRC, the robot is in this mode before competition starts and between mode transitions from Autonomous
         * to TeleOp, for example. (i.e. DISABLED_MODE->AUTO_MODE->DISABLED_MODE->TELEOP_MODE->DISABLED_MODE).
         * This mode does not exist in FTC.
         */
        DISABLED_MODE,

        /**
         * The robot is in this mode during the autonomous period.
         */
        AUTO_MODE,

        /**
         * The robot is in this mode during the operator control period.
         */
        TELEOP_MODE,

        /**
         * The robot is in this mode when Test Mode is selected on the DriverStation.
         */
        TEST_MODE

    }   //enum RunMode

    /**
     * This interface is normally implemented by the main robot code. The methods in this interface is called by
     * the Cooperative Multitasking Scheduler.
     */
    public interface RobotMode
    {
        /**
         * This method is called periodically after initRobot() is called but before competition starts. Typically,
         * you override this method and put code that will check and display robot status in this method. For example,
         * one may monitor the gyro heading in this method to make sure there is no major gyro drift before competition
         * starts.
         */
        void initPeriodic();

        /**
         * This method is called when the competition mode is about to start. In FTC, this is called when the "Play"
         * button on the Driver Station phone is pressed. Typically, you put code that will prepare the robot for
         * start of competition here such as resetting the encoders/sensors and enabling some sensors to start
         * sampling.
         */
        void startMode();

        /**
         * This method is called when competition mode is about to end. Typically, you put code that will do clean
         * up here such as disabling the sampling of some sensors.
         */
        void stopMode();

        /**
         * This method is called periodically about 50 times a second. Typically, you put code that doesn't require
         * frequent update here. For example, TeleOp joystick code can be put here since human responses are
         * considered slow.
         * 
         * @param elapsedTime specifies the elapsed time since the mode started.
         */
        void runPeriodic(double elapsedTime);

        /**
         * This method is called periodically as fast as the control system allows. Typically, you put code that
         * requires servicing at a higher frequency here. To make the robot as responsive and as accurate as possible
         * especially in autonomous mode, you will typically put that code here.
         * 
         * @param elapsedTime specifies the elapsed time since the mode started.
         */
        void runContinuous(double elapsedTime);

    }   //interface RobotMode

    /**
     * This interface is used to implement a robot command. A robot command consists of a sequence of actions
     * typically executed using a state machine. It can be used to implement an entire autonomous strategy or
     * it can implement a subset of an autonomous strategy that can be reused as a common portion of multiple
     * autonomous strategies. This helps to reduce code duplication and improves code maintenance.
     */
    public interface RobotCommand
    {
        /**
         * This method is call periodically to perform the robot command. Typically, it is implemented by using
         * a state machine.
         *
         * @param elapsedTime specifies the elapsed time of the period in seconds.
         * @return true if the command is done, false otherwise.
         */
        boolean cmdPeriodic(double elapsedTime);

    }   //interface RobotCommand

}   //class TrcRobot
