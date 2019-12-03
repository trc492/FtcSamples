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
import ftclib.FtcMotorActuator;
import ftclib.FtcOpMode;
import hallib.HalDashboard;
import trclib.TrcGameController;
import trclib.TrcRobot;

@TeleOp(name="TeleOp: PID Elevator", group="FtcTeleOpSamples")
@Disabled
public class FtcTeleOpPidElevator extends FtcOpMode implements TrcGameController.ButtonHandler
{
    //
    // Elevator constants.
    //
    private static final double ELEVATOR_KP                     = 0.5;
    private static final double ELEVATOR_KI                     = 0.0;
    private static final double ELEVATOR_KD                     = 0.0;
    private static final double ELEVATOR_TOLERANCE              = 0.2;
    private static final double ELEVATOR_MIN_HEIGHT             = 0.0;
    private static final double ELEVATOR_MAX_HEIGHT             = 23.5;
    private static final double ELEVATOR_INCHES_PER_COUNT       = (23.5/9700.0);
    private static final double ELEVATOR_OFFSET                 = 0.0;
    private static final double ELEVATOR_CAL_POWER              = 0.3;
    private static final boolean ELEVATOR_INVERTED              = false;
    private static final boolean ELEVATOR_HAS_UPPER_LIMIT_SWITCH= true;

    private HalDashboard dashboard;
    //
    // Gamepad.
    //
    private FtcGamepad gamepad;
    //
    // Elevator.
    //
    private FtcMotorActuator elevator;

    //
    // Implements FtcOpMode abstract methods.
    //

    @Override
    public void initRobot()
    {
        final FtcMotorActuator.Parameters elevatorParams = new FtcMotorActuator.Parameters()
                .setPosRange(ELEVATOR_MIN_HEIGHT, ELEVATOR_MAX_HEIGHT)
                .setScaleOffset(ELEVATOR_INCHES_PER_COUNT, ELEVATOR_OFFSET)
                .setPidParams(ELEVATOR_KP, ELEVATOR_KI, ELEVATOR_KD, ELEVATOR_TOLERANCE)
                .setMotorParams(ELEVATOR_INVERTED, ELEVATOR_HAS_UPPER_LIMIT_SWITCH, ELEVATOR_CAL_POWER);

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
        elevator = new FtcMotorActuator("elevator", elevatorParams);
        elevator.zeroCalibrate();
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        dashboard.clearDisplay();
    }   //startMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        //
        // Elevator subsystem.
        //
        double elevatorPower = gamepad.getRightStickY(true);
        elevator.setPower(elevatorPower);
        dashboard.displayPrintf(
                1, "Elevator:power=%.2f,height=%.2f,lowerLimit=%s,upperLimit=%s",
                elevatorPower, elevator.getPosition(), elevator.isLowerLimitSwitchActive(),
                elevator.isUpperLimitSwitchActive());
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
                case FtcGamepad.GAMEPAD_RBUMPER:
                    // Press and hold this button to enable manual override.
                    // Release this button to disable manual override.
                    elevator.setManualOverride(pressed);
                    break;

                case FtcGamepad.GAMEPAD_BACK:
                    // Press this button to start zero calibration.
                    if (pressed)
                    {
                        elevator.zeroCalibrate();
                    }
                    break;
            }
        }
    }   //buttonEvent

}   //class FtcTeleOpPidElevator
