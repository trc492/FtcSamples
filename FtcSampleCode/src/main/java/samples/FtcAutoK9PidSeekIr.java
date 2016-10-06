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

package samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;

import ftclib.FtcDcMotor;
import ftclib.FtcOpMode;
import hallib.HalDashboard;
import trclib.TrcDriveBase;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcStateMachine;

@Autonomous(name="Auto: K9Bot PID Seek IR", group="3543AutoSamples")
//@Disabled
public class FtcAutoK9PidSeekIr extends FtcOpMode implements TrcPidController.PidInput
{
    //
    // State machine states.
    //
    private enum State
    {
        SEEK_IR,
        DONE
    }

    //
    // PID drive constants.
    //
    private static final double IRDRIVE_KP              = 0.8;
    private static final double IRDRIVE_KI              = 0.0;
    private static final double IRDRIVE_KD              = 0.0;
    private static final double IRDRIVE_KF              = 0.0;
    private static final double IRDRIVE_TOLERANCE       = 0.1;
    private static final double IRDRIVE_SETTLING        = 0.2;

    //
    // PID seek IR constants.
    //
    private static final double IRTURN_KP               = 0.1;
    private static final double IRTURN_KI               = 0.0;
    private static final double IRTURN_KD               = 0.0;
    private static final double IRTURN_KF               = 0.0;
    private static final double IRTURN_TOLERANCE        = 1.0;
    private static final double IRTURN_SETTLING         = 0.2;

    private HalDashboard dashboard;
    //
    // Sensors.
    //
    private IrSeekerSensor irSeeker;
    private double prevIrAngle = 0.0;
    private double prevIrStrength = 0.0;
    //
    // DriveBase subsystem.
    //
    private FtcDcMotor motorLeft;
    private FtcDcMotor motorRight;
    private TrcDriveBase driveBase;
    //
    // PID drive.
    //
    private TrcPidController irDrivePidCtrl;
    private TrcPidController irTurnPidCtrl;
    private TrcPidDrive seekIrDrive;
    //
    // Event and state machine.
    //
    private TrcEvent event;
    private TrcStateMachine sm;

    //
    // Implements FtcOpMode abstract method.
    //

    @Override
    public void initRobot()
    {
        hardwareMap.logDevices();
        dashboard = getDashboard();
        //
        // Sensors.
        //
        irSeeker = hardwareMap.irSeekerSensor.get("ir_seeker");
        //
        // DriveBase subsystem.
        //
        motorLeft = new FtcDcMotor("motor_1");
        motorRight = new FtcDcMotor("motor_2");
        motorLeft.setInverted(true);
        driveBase = new TrcDriveBase(motorLeft, motorRight);
        //
        // PID drive.
        //
        irDrivePidCtrl = new TrcPidController(
                "irDrivePid",
                IRDRIVE_KP, IRDRIVE_KI, IRDRIVE_KD, IRDRIVE_KF,
                IRDRIVE_TOLERANCE, IRDRIVE_SETTLING, this);
        irDrivePidCtrl.setAbsoluteSetPoint(true);
        irTurnPidCtrl = new TrcPidController(
                "irTurnPid",
                IRTURN_KP, IRTURN_KI, IRTURN_KD, IRTURN_KF,
                IRTURN_TOLERANCE, IRTURN_SETTLING, this);
        irTurnPidCtrl.setAbsoluteSetPoint(true);
        seekIrDrive = new TrcPidDrive(
                "seekIrDrive", driveBase, null, irDrivePidCtrl, irTurnPidCtrl);
        //
        // State machine.
        //
        event = new TrcEvent("seekIrEvent");
        sm = new TrcStateMachine("seekIrSM");
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode()
    {
        dashboard.clearDisplay();
        driveBase.resetPosition();
        //
        // Start state machine at state FIND_LINE.
        //
        sm.start(State.SEEK_IR);
    }   //startMode

    @Override
    public void runContinuous(double elapsedTime)
    {
        irDrivePidCtrl.displayPidInfo(1);
        irTurnPidCtrl.displayPidInfo(3);
        //
        // Run state machine.
        //
        if (sm.isReady())
        {
            State state = (State)sm.getState();
            dashboard.displayPrintf(5, "State: %s", state.toString());
            switch (state)
            {
                case SEEK_IR:
                    //
                    // Go towards IR beacon until IR strength reaches 0.8.
                    //
                    seekIrDrive.setTarget(0.8, 0.0, false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done, stop!
                    //
                    sm.stop();
                    break;
            }
        }
    }   //runContinuous

    //
    // Implements TrcPidController.PidInput interface.
    //

    @Override
    public double getInput(TrcPidController pidCtrl)
    {
        double input = 0.0;

        if (pidCtrl == irDrivePidCtrl)
        {
            //
            // Get the IR strength.
            //
            if (irSeeker.signalDetected())
            {
                input = irSeeker.getStrength();
                prevIrStrength = input;
            }
            else
            {
                input = prevIrStrength;
            }
        }
        else if (pidCtrl == irTurnPidCtrl)
        {
            //
            // Get the IR direction.
            //
            if (irSeeker.signalDetected())
            {
                input = irSeeker.getAngle();
                prevIrAngle = input;
            }
            else
            {
                input = prevIrAngle;
            }
        }

        return input;
    }   //getInput

}   //class FtcAutoK9PidSeekIr
