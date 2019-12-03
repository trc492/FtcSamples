/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
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
import ftclib.FtcServoActuator;
import trclib.TrcEnhancedServo;
import trclib.TrcGameController;

@TeleOp(name="TeleOp: Dual Servo Grabber", group="FtcTeleOpSamples")
@Disabled
public class FtcTeleOpGrabber extends FtcOpMode implements TrcGameController.ButtonHandler
{
    //
    // Grabber constants.
    //
    static final double GRABBER_MAX_STEPRATE            = 0.0;
    static final double GRABBER_MIN_POS                 = 0.0;
    static final double GRABBER_MAX_POS                 = 1.0;
    static final double GRABBER_RELEASE_POS             = 0.0;
    static final double GRABBER_RELEASE_TIME            = 0.5;
    static final double GRABBER_GRAB_POS                = 1.0;
    static final double GRABBER_GRAB_TIME               = 0.5;

    private FtcGamepad gamepad;
    private FtcServoActuator grabber;

    //
    // Implements FtcOpMode abstract methods.
    //

    @Override
    public void initRobot()
    {
        final FtcServoActuator.Parameters grabberParams = new FtcServoActuator.Parameters()
                .setStepParams(GRABBER_MAX_STEPRATE, GRABBER_MIN_POS, GRABBER_MAX_POS)
                .setInverted(true, false)
                .setRetractParams(GRABBER_GRAB_POS, GRABBER_GRAB_TIME)
                .setExtendParams(GRABBER_RELEASE_POS, GRABBER_RELEASE_TIME);

        hardwareMap.logDevices();

        grabber = new FtcServoActuator("grabberLeftServo", "grabberRightServo", grabberParams);
        //
        // Initializing gamepads.
        //
        gamepad = new FtcGamepad("Gamepad", gamepad1, this);
        gamepad.setYInverted(true);
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void runPeriodic(double elapsedTime)
    {
        grabber.setPosition(gamepad.getRightTrigger());
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

}   //class FtcTeleOpGrabber
