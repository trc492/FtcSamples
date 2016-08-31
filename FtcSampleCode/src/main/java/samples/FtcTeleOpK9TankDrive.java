/*
 * Titan Robotics Framework Library
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.net)
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftclib.FtcDcMotor;
import ftclib.FtcGamepad;
import ftclib.FtcOpMode;
import ftclib.FtcServo;
import hallib.HalDashboard;
import trclib.TrcDriveBase;
import trclib.TrcEnhancedServo;

@TeleOp(name="TeleOp: K9Bot Tank Drive", group="Ftc3543Sample")
//@Disabled
public class FtcTeleOpK9TankDrive extends FtcOpMode implements FtcGamepad.ButtonHandler
{
    private static final double ARM_MIN_RANGE   = 0.2;
    private static final double ARM_MAX_RANGE   = 0.9;
    private static final double CLAW_MIN_RANGE  = 0.2;
    private static final double CLAW_MAX_RANGE  = 0.7;
    private static final double SERVO_STEPRATE  = 2.0;

    private HalDashboard dashboard;
    private FtcGamepad gamepad;
    private FtcDcMotor motorLeft;
    private FtcDcMotor motorRight;
    private TrcDriveBase driveBase;
    private FtcServo armServo;
    private TrcEnhancedServo arm;
    private FtcServo clawServo;
    private TrcEnhancedServo claw;

    //
    // Implements FtcOpMode abstract method.
    //

    @Override
    public void initRobot()
    {
        hardwareMap.logDevices();
        dashboard = getDashboard();
        //
        // Initializing Gamepads.
        //
        gamepad = new FtcGamepad("Gamepad", gamepad1, this);
        gamepad.setYInverted(true);
        //
        // DriveBase subsystem.
        //
        motorLeft = new FtcDcMotor("motor_1");
        motorRight = new FtcDcMotor("motor_2");
        motorLeft.setInverted(true);
        driveBase = new TrcDriveBase(motorLeft, motorRight);
        //
        // Arm subsystem.
        //
        armServo = new FtcServo("servo_1");
        armServo.setLogicalRange(ARM_MIN_RANGE, ARM_MAX_RANGE);
        arm = new TrcEnhancedServo("arm", armServo);
        arm.setPosition(ARM_MIN_RANGE);
        //
        // Claw subsystem.
        //
        clawServo = new FtcServo("servo_6");
        clawServo.setLogicalRange(CLAW_MIN_RANGE, CLAW_MAX_RANGE);
        claw = new TrcEnhancedServo("claw", clawServo);
        claw.setPosition(CLAW_MIN_RANGE);
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
        // There is an issue with the gamepad objects that may not be valid
        // before waitForStart() is called. So we call the setGamepad method
        // here to update their references in case they have changed.
        //
        gamepad.setGamepad(gamepad1);
    }   //startMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        //
        // DriveBase subsystem.
        //
        double left = gamepad.getLeftStickY(true);
        double right = gamepad.getRightStickY(true);
        driveBase.tankDrive(left, right);

        dashboard.displayPrintf(1, "Text       : *** Robot Data ***");
        dashboard.displayPrintf(2, "arm        : %.2f", armServo.getPosition());
        dashboard.displayPrintf(3, "claw       : %.2f", clawServo.getPosition());
        dashboard.displayPrintf(4, "left power : %.2f", left);
        dashboard.displayPrintf(5, "right power: %.2f", right);
    }   //runPeriodic

    //
    // Implements FtcGamepad.ButtonHandler interface.
    //

    @Override
    public void gamepadButtonEvent(FtcGamepad gamepad, int button, boolean pressed)
    {
        if (gamepad == this.gamepad)
        {
            switch (button)
            {
                case FtcGamepad.GAMEPAD_A:
                    if (pressed)
                    {
                        arm.setPosition(ARM_MAX_RANGE, SERVO_STEPRATE);
                    }
                    else
                    {
                        arm.stop();
                    }
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    if (pressed)
                    {
                        arm.setPosition(ARM_MIN_RANGE, SERVO_STEPRATE);
                    }
                    else
                    {
                        arm.stop();
                    }
                    break;

                case FtcGamepad.GAMEPAD_X:
                    if (pressed)
                    {
                        claw.setPosition(CLAW_MAX_RANGE, SERVO_STEPRATE);
                    }
                    else
                    {
                        claw.stop();
                    }
                    break;

                case FtcGamepad.GAMEPAD_B:
                    if (pressed)
                    {
                        claw.setPosition(CLAW_MIN_RANGE, SERVO_STEPRATE);
                    }
                    else
                    {
                        claw.stop();
                    }
                    break;
            }
        }
    }   //gamepadButtonEvent

}   //class FtcTeleOpK9TankDrive
