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
import hallib.HalDashboard;

@TeleOp(name="TeleOp: PID Elevator", group="3543TeleOpSamples")
@Disabled
public class FtcTeleOpPidElevator extends FtcOpMode implements FtcGamepad.ButtonHandler
{
    private static final double ELEVATOR_CAL_POWER = 0.3;

    private HalDashboard dashboard;
    //
    // Gamepad.
    //
    private FtcGamepad gamepad;
    //
    // Elevator.
    //
    private Elevator elevator;

    //
    // Implements FtcOpMode abstract methods.
    //

    @Override
    public void initRobot()
    {
        hardwareMap.logDevices();
        dashboard = HalDashboard.getInstance();
        //
        // Initializing Gamepad.
        //
        gamepad = new FtcGamepad("gamepad", gamepad1, this);
        gamepad.setYInverted(true);
        //
        // Elevator subsystem.
        //
        elevator = new Elevator();
        elevator.zeroCalibrate(ELEVATOR_CAL_POWER);
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
        // Elevator subsystem.
        //
        double elevatorPower = gamepad.getRightStickY(true);
        elevator.setPower(elevatorPower);
        dashboard.displayPrintf(1, "Elevator:power=%.2f,height=%.2f",
                                elevatorPower, elevator.getPosition());
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
                case FtcGamepad.GAMEPAD_RBUMPER:
                    elevator.setManualOverride(pressed);
                    break;

                case FtcGamepad.GAMEPAD_START:
                    if (pressed)
                    {
                        elevator.zeroCalibrate(ELEVATOR_CAL_POWER);
                    }
                    break;
            }
        }
    }   //gamepadButtonEvent

}   //class FtcTeleOpPidElevator
