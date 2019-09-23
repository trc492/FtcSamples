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

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

import FtcSampleCode.R;
import ftclib.FtcOpMode;
import hallib.HalDashboard;
import trclib.TrcRobot;
import trclib.TrcVideoSource;

@TeleOp(name="Test: Grip Vision", group="FtcTestSamples")
@Disabled
public class FtcTestGripVision extends FtcOpMode
        implements CameraBridgeViewBase.CvCameraViewListener2, TrcVideoSource<Mat>
{
    private HalDashboard dashboard;
    private FtcRobotControllerActivity activity;
    private BaseLoaderCallback loaderCallback;
    private CameraBridgeViewBase cameraView;
    private Mat image = null;
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
        //
        // Initialize OpenCV.
        //
        activity = (FtcRobotControllerActivity)hardwareMap.appContext;
        loaderCallback = new BaseLoaderCallback(activity)
        {
            /**
             * This method is called when the OpenCV manager is connected. It loads the
             * OpenCV library.
             *
             * @param status specifies the OpenCV connection status.
             */
            @Override
            public void onManagerConnected(int status)
            {
                switch (status)
                {
                    case LoaderCallbackInterface.SUCCESS:
                        System.loadLibrary("opencv_java3");
                        cameraView.enableView();
                        break;

                    default:
                        super.onManagerConnected(status);
                        break;
                }
            }   //onManagerConnected
        };

        cameraView = (CameraBridgeViewBase)activity.findViewById(R.id.ImageView01);
        cameraView.setCameraIndex(1);   //use front camera
        cameraView.setCvCameraViewListener(this);

        targetRects = new Rect[2];
        gripVision = new GripVision("gripVision", this);
        gripVision.setVideoOutEnabled(false);
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        dashboard.clearDisplay();
        startCamera();
        gripVision.setEnabled(true);
    }   //startMode

    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        gripVision.setEnabled(false);
        stopCamera();
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

    //
    // Implements the CameraBridgeViewBase.CvCameraViewListener2 interface.
    //

    /**
     * This method is called when the camera view is started. It will allocate and initialize
     * some global resources.
     *
     * @param width specifies the width of the camera view.
     * @param height specifies the height of the camera view.
     */
    @Override
    public void onCameraViewStarted(int width, int height)
    {
    }   //onCameraViewStarted

    /**
     * This method is called when the camera view is stopped. It will clean up the allocated
     * global resources.
     */
    @Override
    public void onCameraViewStopped()
    {
        if (image != null) image.release();
    }   //onCameraViewStopped

    /**
     * This method is called on every captured camera frame. It will do face detection on the
     * captured frame.
     *
     * @param inputFrame specifies the captured frame object.
     */
    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame)
    {
        //
        // Get rid of the old unconsumed image if any.
        //
        if (image != null)
        {
            image.release();
            image = null;
        }
        //
        // Get a fresh image.
        //
        image = inputFrame.rgba();

        return image;
    }   //onCameraViewFrame

    //
    // Implements TrcVideoSource<Mat> interface.
    //

    /**
     * This method gets a frame from the frame queue and returns the image that matches the format specified by the
     * configVideoSource method.
     *
     * @param frame specifies the frame object to hold image.
     * @return true if success, false otherwise.
     */
    @Override
    public boolean getFrame(Mat frame)
    {
        boolean success;

        if (image != null)
        {
            // Consume and return the image.
            image.copyTo(frame);
            image.release();
            image = null;
            success = true;
        }
        else
        {
            success = false;
        }

        return success;
    }   //getFrame

    /**
     * This method draws the given image frame to the display surface.
     *
     * @param frame specifies the image frame to be displayed.
     */
    @Override
    public void putFrame(Mat frame)
    {
    }   //putFrame

    private void startCamera()
    {
        if (!OpenCVLoader.initDebug())
        {
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, activity, loaderCallback);
        }
        else
        {
            loaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }   //startCamera

    private void stopCamera()
    {
        cameraView.disableView();
    }   //stopCamera

}   //class FtcTestGripVision