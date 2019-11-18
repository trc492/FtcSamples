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

import android.widget.TextView;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import FtcSampleCode.R;
import ftclib.FtcDcMotor;
import ftclib.FtcGamepad;
import ftclib.FtcOpMode;
import hallib.HalDashboard;
import trclib.TrcGameController;
import trclib.TrcMecanumDriveBase;
import trclib.TrcRobot;

@TeleOp(name="TeleOp: Mecanum Drive", group="FtcTeleOpSamples")
@Disabled
public class FtcTeleOpMecanumDrive extends FtcOpMode implements TrcGameController.ButtonHandler
{
    private HalDashboard dashboard;
    private FtcGamepad gamepad;
    private FtcDcMotor lfWheel;
    private FtcDcMotor rfWheel;
    private FtcDcMotor lrWheel;
    private FtcDcMotor rrWheel;
    private TrcMecanumDriveBase driveBase;

    //
    // Implements FtcOpMode abstract methods.
    //

    @Override
    public void initRobot()
    {
        hardwareMap.logDevices();
        dashboard = HalDashboard.getInstance();
        //
        // Initializing sensors.
        //

        //
        // Initializing gamepads.
        //
        gamepad = new FtcGamepad("Gamepad", gamepad1, this);
        gamepad.setYInverted(true);
        //
        // DriveBase subsystem.
        //
        lfWheel = new FtcDcMotor("lfWheel");
        rfWheel = new FtcDcMotor("rfWheel");
        lrWheel = new FtcDcMotor("lrWheel");
        rrWheel = new FtcDcMotor("rrWheel");
        rfWheel.setInverted(true);
        rrWheel.setInverted(true);
        driveBase = new TrcMecanumDriveBase(lfWheel, lrWheel, rfWheel, rrWheel, null);
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        dashboard.clearDisplay();
        driveBase.resetOdometry();
    }   //startMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        //
        // DriveBase subsystem.
        //
        double x = gamepad.getLeftStickX(true);
        double y = gamepad.getRightStickY(true);
        double rotation = gamepad.getRightTrigger(true) - gamepad.getLeftTrigger(true);
        driveBase.holonomicDrive(x, y, rotation, false);

        dashboard.displayPrintf(1, "Text: *** Robot Data ***");
        dashboard.displayPrintf(2, "x: %.2f", x);
        dashboard.displayPrintf(3, "y: %.2f", y);
        dashboard.displayPrintf(4, "rotation: %.2f", rotation);
    }   //runPeriodic

    //
    // Implements TrcGameController.ButtonHandler interface.
    //

    @Override
    public void buttonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        if (gamepad == this.gamepad)
        {
            switch (button)
            {
                case FtcGamepad.GAMEPAD_A:
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    break;

                case FtcGamepad.GAMEPAD_X:
                    break;

                case FtcGamepad.GAMEPAD_B:
                    break;

                case FtcGamepad.GAMEPAD_RBUMPER:
                    break;
            }
        }
    }   //buttonEvent

}   //class FtcTeleOpMecanumDrive
