/*
 * Copyright (c) 2020 Titan Robotics Club (http://www.titanrobotics.com)
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

package org.firstinspires.ftc.teamcode.common;

import java.util.Locale;

import hallib.HalDashboard;
import trclib.TrcDbgTrace;
import trclib.TrcEvent;
import trclib.TrcMotor;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

/**
 * This class implements the drive base motors test. This test will spin each of the drive base motors at the
 * specified drive power for the specified period of time.
 */
public class CmdDriveMotorsTest implements TrcRobot.RobotCommand
{
    private enum State
    {
        START,
        DONE
    }   //enum State

    private static final String moduleName = "CmdDriveMotorsTest";
    private static final HalDashboard dashboard = HalDashboard.getInstance();
    private static final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();

    private final TrcMotor[] motors;
    private final double driveTime;
    private final double drivePower;
    private final TrcEvent event;
    private final TrcTimer timer;
    private final TrcStateMachine<State> sm;
    private int motorIndex = 0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param motors specifies the array of motors on the drive base.
     * @param driveTime specifies the amount of drive time in seconds.
     * @param drivePower specifies the motor power.
     */
    public CmdDriveMotorsTest(TrcMotor[] motors, double driveTime, double drivePower)
    {
        this.motors = motors;
        this.driveTime = driveTime;
        this.drivePower = drivePower;
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START);
    }   //CmdDriveMotorsTest

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    /**
     * This method checks if the current RobotCommand  is running.
     *
     * @return true if the command is running, false otherwise.
     */
    @Override
    public boolean isActive()
    {
        return sm.isEnabled();
    }   //isActive

    /**
     * This method cancels the command if it is active.
     */
    @Override
    public void cancel()
    {
        stopAllWheels();
        sm.stop();
    }   //cancel

    /**
     * This method must be called periodically by the caller to drive the command sequence forward.
     *
     * @param elapsedTime specifies the elapsed time in seconds since the start of the robot mode.
     * @return true if the command sequence is completed, false otherwise.
     */
    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        State state = sm.checkReadyAndGetState();

        if (state == null)
        {
            dashboard.displayPrintf(1, "State: disabled or waiting...");
        }
        else
        {
            StringBuilder msg = new StringBuilder("Enc:");

            for (int i = 0; i < motors.length; i++)
            {
                msg.append(String.format(Locale.US, " [%d]=%6.2f", i, motors[i].getPosition()));
            }
            dashboard.displayPrintf(1, "Motors Test: state=%s, index=%d", state, motorIndex);
            dashboard.displayPrintf(2, "%s", msg);
    
            switch (state)
            {
                case START:
                    //
                    // Spin a wheel at drivePower for driveTime seconds.
                    //
                    for (int i = 0; i < motors.length; i++)
                    {
                        if (i == motorIndex)
                        {
                            motors[i].set(drivePower);
                        }
                        else
                        {
                            motors[i].set(0.0);
                        }
                    }
                    motorIndex++;
                    timer.set(driveTime, event);
                    sm.waitForSingleEvent(event, motorIndex < motors.length ? State.START : State.DONE);
                    break;

                case DONE:
                    //
                    // We are done, stop all wheels.
                    //
                    stopAllWheels();
                    sm.stop();
                    break;
            }

            globalTracer.traceStateInfo(state);
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

    /**
     * This method stops all motors on the drive base.
     */
    private void stopAllWheels()
    {
        for (TrcMotor motor: motors)
        {
            motor.set(0.0);
        }
    }   //stopAllWheels

}   //class CmdDriveMotorsTest
