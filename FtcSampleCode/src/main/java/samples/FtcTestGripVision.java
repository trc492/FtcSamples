/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
 * Based on sample code by Robert Atkinson.
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

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.core.Rect;

import ftclib.FtcOpMode;
import ftclib.FtcVuforia;
import hallib.HalDashboard;

@TeleOp(name="Test: Grip Vision", group="3543TestSamples")
//@Disabled
public class FtcTestGripVision extends FtcOpMode
{
    private static final int IMAGE_WIDTH = 640;
    private static final int IMAGE_HEIGHT = 480;
    private static final int FRAME_QUEUE_CAPACITY = 2;

    //
    // If you copy our code, please register your own account and generate your own free license key at this site:
    // https://developer.vuforia.com/license-manager
    //
    private static final String VUFORIA_LICENSE_KEY =
            "AdCwzDH/////AAAAGeDkDS3ukU9+lIXc19LMh+cKk29caNhOl8UqmZOymRGwVwT1ZN8uaPdE3Q+zceDu9AKNsqL9qLblSFV" +
            "/x8Y3jfOZdjMFs0CQSQOEyWv3xfJsdSmevXDQDQr+4KI31HY2YSf/KB/kyxfuRMk4Pi+vWS+oLl65o7sWPiyFgzoM74ENyb" +
            "j4FgteD/2b6B+UFuwkHWKBNpp18wrpkaiFfr/FCbRFcdWP5mrjlEZM6eOj171dybw97HPeZbGihnnxOeeUv075O7P167AVq" +
            "aiPy2eRK7OCubR32KXOqQKoyF6AXp+qu2cOTApXS5dqOOseEm+HE4eMF0S2Pld3i5AWBIR+JlPXDuc9LwoH2Q8iDwUK1+4g";
    private static final VuforiaLocalizer.CameraDirection CAMERA_DIR = VuforiaLocalizer.CameraDirection.BACK;

    private HalDashboard dashboard;
    private FtcVuforia vuforia = null;
    private GripVision gripVision = null;
    private Rect[] targetRects = new Rect[2];

    //
    // Implements FtcOpMode abstract methods.
    //

    @Override
    public void initRobot()
    {
        hardwareMap.logDevices();
        dashboard = HalDashboard.getInstance();

        int cameraViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        vuforia = new FtcVuforia(VUFORIA_LICENSE_KEY, cameraViewId, CAMERA_DIR);
        vuforia.configVideoSource(IMAGE_WIDTH, IMAGE_HEIGHT, FRAME_QUEUE_CAPACITY);

        targetRects = new Rect[2];
        gripVision = new GripVision("gripVision", vuforia);
        gripVision.setVideoOutEnabled(false);
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode()
    {
        dashboard.clearDisplay();
//        gripVision.setEnabled(true);
    }   //startMode

    @Override
    public void stopMode()
    {
//        gripVision.setEnabled(false);
    }   //stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        gripVision.retrieveTargetRects(targetRects);
        for (int i = 0; i < targetRects.length; i++)
        {
            if (targetRects[i] != null)
            {
                dashboard.displayPrintf(1 + i, "rect=%s", targetRects[i]);
            }
        }
    }   //runPeriodic

}   //class FtcTestGripVision