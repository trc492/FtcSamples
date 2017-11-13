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

package ftclib;

import com.qualcomm.robotcore.hardware.Gamepad;

import trclib.TrcDbgTrace;
import trclib.TrcGameController;

/**
 * This class implements the platform dependent gamepad. It provides monitoring of the gamepad buttons. If the caller
 * of this class provides a button notification handler, it will call it when there are button events.
 */
public class FtcGamepad extends TrcGameController
{
    public static final int GAMEPAD_A           = ((int)1 << 0);
    public static final int GAMEPAD_B           = ((int)1 << 1);
    public static final int GAMEPAD_X           = ((int)1 << 2);
    public static final int GAMEPAD_Y           = ((int)1 << 3);
    public static final int GAMEPAD_BACK        = ((int)1 << 4);
    public static final int GAMEPAD_START       = ((int)1 << 5);
    public static final int GAMEPAD_LBUMPER     = ((int)1 << 6);
    public static final int GAMEPAD_RBUMPER     = ((int)1 << 7);
    public static final int GAMEPAD_LSTICK_BTN  = ((int)1 << 8);
    public static final int GAMEPAD_RSTICK_BTN  = ((int)1 << 9);
    public static final int GAMEPAD_DPAD_LEFT   = ((int)1 << 10);
    public static final int GAMEPAD_DPAD_RIGHT  = ((int)1 << 11);
    public static final int GAMEPAD_DPAD_UP     = ((int)1 << 12);
    public static final int GAMEPAD_DPAD_DOWN   = ((int)1 << 13);

    private Gamepad gamepad;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param gamepad specifies the gamepad associated with this instance.
     * @param buttonHandler specifies the object that will handle the button events. If none provided, it is set to
     *                      null.
     */
    public FtcGamepad(final String instanceName, Gamepad gamepad, ButtonHandler buttonHandler)
    {
        super(instanceName, 0.0, buttonHandler);
        this.gamepad = gamepad;
    }   //FtcGamepad

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param gamepad specifies the gamepad associated with this instance.
     */
    public FtcGamepad(final String instanceName, Gamepad gamepad)
    {
        this(instanceName, gamepad, null);
    }   //FtcGamepad

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param gamepad specifies the gamepad associated with this instance.
     * @param deadbandThreshold specifies the deadband of the gamepad analog sticks.
     * @param buttonHandler specifies the object that will handle the button events. If none provided, it is set
     *                      to null.
     */
    public FtcGamepad(
            final String instanceName, Gamepad gamepad, final double deadbandThreshold, ButtonHandler buttonHandler)
    {
        this(instanceName, gamepad, buttonHandler);
        gamepad.setJoystickDeadzone((float)deadbandThreshold);
    }   //FtcGamepad

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param gamepad specifies the gamepad associated with this instance.
     * @param deadbandThreshold specifies the deadband of the gamepad analog sticks.
     */
    public FtcGamepad(final String instanceName, Gamepad gamepad, final double deadbandThreshold)
    {
        this(instanceName, gamepad, deadbandThreshold, null);
    }   //FtcGamepad

    /**
     * This method returns the x-axis value of the left stick.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *              raised exponentially, it gives you more precise control on the low end values.
     *
     * @return x-axis value of the left stick.
     */
    public double getLeftStickX(boolean doExp)
    {
        final String funcName = "getLeftStickX";
        double value = adjustAnalogControl(gamepad.left_stick_x, doExp);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "exp=%s", Boolean.toString(doExp));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", value);
        }

        return value;
    }   //getLeftStickX

    /**
     * This method returns the x-axis value of the left stick.
     *
     * @return x-axis value of the left stick.
     */
    public double getLeftStickX()
    {
        return getLeftStickX(false);
    }   //getLeftStickX

    /**
     * This method returns the y-axis value of the left stick.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *              raised exponentially, it gives you more precise control on the low end values.
     *
     * @return y-axis value of the left stick.
     */
    public double getLeftStickY(boolean doExp)
    {
        final String funcName = "getLeftStickY";
        double value = adjustAnalogControl(gamepad.left_stick_y, doExp);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "exp=%s", Boolean.toString(doExp));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", value);
        }

        return value;
    }   //getLeftStickY

    /**
     * This method returns the y-axis value of the left stick.
     *
     * @return y-axis value of the left stick.
     */
    public double getLeftStickY()
    {
        return getLeftStickY(false);
    }   //getLeftStickY

    /**
     * This method returns the x-axis value of the right stick.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *              raised exponentially, it gives you more precise control on the low end values.
     *
     * @return x-axis value of the right stick.
     */
    public double getRightStickX(boolean doExp)
    {
        final String funcName = "getRightStickX";
        double value = adjustAnalogControl(gamepad.right_stick_x, doExp);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "exp=%s", Boolean.toString(doExp));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", value);
        }

        return value;
    }   //getRightStickX

    /**
     * This method returns the x-axis value of the right stick.
     *
     * @return x-axis value of the right stick.
     */
    public double getRightStickX()
    {
        return getRightStickX(false);
    }   //getRightStickX

    /**
     * This method returns the y-axis value of the right stick.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *              raised exponentially, it gives you more precise control on the low end values.
     *
     * @return y-axis value of the right stick.
     */
    public double getRightStickY(boolean doExp)
    {
        final String funcName = "getRightStickY";
        double value = adjustAnalogControl(gamepad.right_stick_y, doExp);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "exp=%s", Boolean.toString(doExp));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", value);
        }

        return value;
    }   //getRightStickY

    /**
     * This method returns the y-axis value of the right stick.
     *
     * @return y-axis value of the right stick.
     */
    public double getRightStickY()
    {
        return getRightStickY(false);
    }   //getRightStickY

    /**
     * This method returns the left trigger value.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *              raised exponentially, it gives you more precise control on the low end values.
     *
     * @return left trigger value.
     */
    public double getLeftTrigger(boolean doExp)
    {
        final String funcName = "getLeftTrigger";
        double value = adjustAnalogControl(gamepad.left_trigger, doExp);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "exp=%s", Boolean.toString(doExp));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", value);
        }

        return value;
    }   //getLeftTrigger

    /**
     * This method returns the left trigger value.
     *
     * @return left trigger value.
     */
    public double getLeftTrigger()
    {
        return getLeftTrigger(false);
    }   //getLeftTrigger

    /**
     * This method returns the right trigger value.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *              raised exponentially, it gives you more precise control on the low end values.
     *
     * @return right trigger value.
     */
    public double getRightTrigger(boolean doExp)
    {
        final String funcName = "getRightTrigger";
        double value = adjustAnalogControl(gamepad.right_trigger, doExp);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "exp=%s", Boolean.toString(doExp));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", value);
        }

        return value;
    }   //getRightTrigger

    /**
     * This method returns the right trigger value.
     *
     * @return right trigger value.
     */
    public double getRightTrigger()
    {
        return getRightTrigger(false);
    }   //getRightTrigger

    /**
     * This method returns the left stick magnitude combining the x and y axes.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *              raised exponentially, it gives you more precise control on the low end values.
     *
     * @return left stick magnitude.
     */
    public double getLeftStickMagnitude(boolean doExp)
    {
        return getMagnitude(getLeftStickX(doExp), getLeftStickY(doExp));
    }   //getLeftStickMagnitude

    /**
     * This method returns the left stick magnitude combining the x and y axes.
     *
     * @return left stick magnitude.
     */
    public double getLeftStickMagnitude()
    {
        return getLeftStickMagnitude(false);
    }   //getLeftStickMagnitude

    /**
     * This method returns the right stick magnitude combining the x and y axes.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *              raised exponentially, it gives you more precise control on the low end values.
     *
     * @return right stick magnitude.
     */
    public double getRightStickMagnitude(boolean doExp)
    {
        return getMagnitude(getRightStickX(doExp), getRightStickY(doExp));
    }   //getRightStickMagnitude

    /**
     * This method returns the right stick magnitude combining the x and y axes.
     *
     * @return right stick magnitude.
     */
    public double getRightStickMagnitude()
    {
        return getRightStickMagnitude(false);
    }   //getRightStickMagnitude

    /**
     * This method returns the left stick direction in radians combining the x and y axes.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *              raised exponentially, it gives you more precise control on the low end values.
     *
     * @return left stick direction in radians.
     */
    public double getLeftStickDirectionRadians(boolean doExp)
    {
        final String funcName = "getLeftStickDirectionRadians";
        double value = getDirection(gamepad.left_stick_x, gamepad.left_stick_y, doExp);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "exp=%s", Boolean.toString(doExp));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", value);
        }

        return value;
    }   //getLeftStickDirectionRadians

    /**
     * This method returns the left stick direction in radians combining the x and y axes.
     *
     * @return left stick direction in radians.
     */
    public double getLeftStickDirectionRadians()
    {
        return getLeftStickDirectionRadians(false);
    }   //getLeftStickDirectionRadians

    /**
     * This method returns the right stick direction in radians combining the x and y axes.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *              raised exponentially, it gives you more precise control on the low end values.
     *
     * @return right stick direction in radians.
     */
    public double getRightStickDirectionRadians(boolean doExp)
    {
        final String funcName = "getRightStickDirectionRadians";
        double value = getDirection(gamepad.right_stick_x, gamepad.right_stick_y, doExp);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "exp=%s", Boolean.toString(doExp));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", value);
        }

        return value;
    }   //getRightStickDirectionRadians

    /**
     * This method returns the right stick direction in radians combining the x and y axes.
     *
     * @return right stick direction in radians.
     */
    public double getRightStickDirectionRadians()
    {
        return getRightStickDirectionRadians(false);
    }   //getRightStickDirectionRadians

    /**
     * This method returns the left stick direction in degrees combining the x and y axes.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *              raised exponentially, it gives you more precise control on the low end values.
     *
     * @return left stick direction in degrees.
     */
    public double getLeftStickDirectionDegrees(boolean doExp)
    {
        return Math.toDegrees(getLeftStickDirectionRadians(doExp));
    }   //getLeftStickDirectionDegrees

    /**
     * This method returns the left stick direction in degrees combining the x and y axes.
     *
     * @return left stick direction in degrees.
     */
    public double getLeftStickDirectionDegrees()
    {
        return getLeftStickDirectionDegrees(false);
    }   //getLeftStickDirectionDegrees

    /**
     * This method returns the right stick direction in degrees combining the x and y axes.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *              raised exponentially, it gives you more precise control on the low end values.
     *
     * @return right stick direction in degrees.
     */
    public double getRightStickDirectionDegrees(boolean doExp)
    {
        return Math.toDegrees(getRightStickDirectionRadians(doExp));
    }   //getRightStickDirectionDegrees

    /**
     * This method returns the right stick direction in degrees combining the x and y axes.
     *
     * @return right stick direction in degrees.
     */
    public double getRightStickDirectionDegrees()
    {
        return getRightStickDirectionDegrees(false);
    }   //getRightStickDirectionDegrees

    //
    // Implements TrcGameController abstract methods.
    //

    /**
     * This method returns the button states in an integer by combining all the button states.
     *
     * @return button states.
     */
    @Override
    public int getButtons()
    {
        final String funcName = "getButtons";

        int buttons = 0;
        buttons |= gamepad.a? GAMEPAD_A: 0;
        buttons |= gamepad.b? GAMEPAD_B: 0;
        buttons |= gamepad.x? GAMEPAD_X: 0;
        buttons |= gamepad.y? GAMEPAD_Y: 0;
        buttons |= gamepad.back? GAMEPAD_BACK: 0;
        buttons |= gamepad.start? GAMEPAD_START: 0;
        buttons |= gamepad.left_bumper? GAMEPAD_LBUMPER: 0;
        buttons |= gamepad.right_bumper? GAMEPAD_RBUMPER: 0;
        buttons |= gamepad.left_stick_button? GAMEPAD_LSTICK_BTN: 0;
        buttons |= gamepad.right_stick_button? GAMEPAD_RSTICK_BTN: 0;
        buttons |= gamepad.dpad_left? GAMEPAD_DPAD_LEFT: 0;
        buttons |= gamepad.dpad_right? GAMEPAD_DPAD_RIGHT: 0;
        buttons |= gamepad.dpad_up? GAMEPAD_DPAD_UP: 0;
        buttons |= gamepad.dpad_down? GAMEPAD_DPAD_DOWN: 0;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC, "=%x", buttons);
        }

        return buttons;
    }   //getButtons

}   //class FtcGamepad
