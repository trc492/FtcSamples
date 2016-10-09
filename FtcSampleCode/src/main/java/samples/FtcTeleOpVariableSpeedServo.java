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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftclib.FtcGamepad;
import ftclib.FtcOpMode;
import ftclib.FtcServo;
import hallib.HalDashboard;
import trclib.TrcEnhancedServo;

@TeleOp(name="TeleOp: Variable Speed Servo", group="3543TeleOpSamples")
//@Disabled
public class FtcTeleOpVariableSpeedServo extends FtcOpMode
{
    private static final double ARM_DOWN_POSITION   = 0.0;
    private static final double ARM_UP_POSITION     = 1.0;
    private static final double ARM_MAX_STEPRATE    = 0.2;

    private HalDashboard dashboard;
    //
    // Gamepad.
    //
    private FtcGamepad gamepad;
    //
    // Arm subsystem.
    //
    private FtcServo armServo;
    private TrcEnhancedServo arm;

    //
    // Implements FtcOpMode abstract methods.
    //

    @Override
    public void initRobot()
    {
        hardwareMap.logDevices();
        dashboard = getDashboard();
        //
        // Initializing Gamepad.
        //
        gamepad = new FtcGamepad("gamepad", gamepad1);
        gamepad.setYInverted(true);
        //
        // Arm subsystem.
        //
        armServo = new FtcServo("armServo");
        arm = new TrcEnhancedServo("arm", armServo);
        arm.setPosition(ARM_DOWN_POSITION);
        arm.setStepMode(ARM_MAX_STEPRATE, ARM_DOWN_POSITION, ARM_UP_POSITION);
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode()
    {
        dashboard.clearDisplay();
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
        // Arm subsystem.
        //
        double armPower = gamepad.getRightStickY(true);
        arm.setPower(armPower);
        dashboard.displayPrintf(2, "Arm:power=%.2f,position=%.2f",
                                armPower, armServo.getPosition());
    }   //runPeriodic

}   //class FtcTeleOpVariableSpeedServo
