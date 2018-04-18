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

import ftclib.FtcOpMode;
import trclib.TrcRobot;

@Autonomous(name="Auto: K9Bot Line Following", group="3543AutoSamples")
@Disabled
public class FtcAutoK9LineFollow extends FtcOpMode
{
    private static final double MOTOR_POWER = 0.15;
    private K9Robot robot;

    //
    // Implements FtcOpMode abstract methods.
    //

    @Override
    public void initRobot()
    {
        robot = new K9Robot(TrcRobot.RunMode.AUTO_MODE);
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode(TrcRobot.RunMode prevMode)
    {
        robot.startMode(TrcRobot.RunMode.AUTO_MODE);
    }   //startMode

    @Override
    public void stopMode(TrcRobot.RunMode nextMode)
    {
        robot.stopMode(TrcRobot.RunMode.AUTO_MODE);
    }   //stopMode

    @Override
    public void runContinuous(double elapsedTime)
    {
        final int LABEL_WIDTH = 200;
        double left = 0.0;
        double right = 0.0;
        double lightValue = robot.lightSensor.sensor.getRawLightDetected();
        //
        // Following the left edge of a white line.
        //
        if (lightValue < K9Robot.LIGHT_THRESHOLD)
        {
            //
            // We see the floor, turn right back to the line edge.
            //
            left = MOTOR_POWER;
            right = 0.0;
        }
        else
        {
            //
            // We see the line, turn left back to the line edge.
            //
            left = 0.0;
            right = MOTOR_POWER;
        }
        robot.driveBase.tankDrive(left, right);

        robot.dashboard.displayPrintf(1, LABEL_WIDTH, "Text: ", "*** Robot Data ***");
        robot.dashboard.displayPrintf(2, LABEL_WIDTH, "light: ", "%.2f", lightValue);
        robot.dashboard.displayPrintf(3, LABEL_WIDTH, "left power: ", "%.2f", left);
        robot.dashboard.displayPrintf(4, LABEL_WIDTH, "right power: ", "%.2f", right);
    }   //runContinuous

}   //class FtcAutoK9LineFollow
