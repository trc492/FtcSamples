/*
 * Copyright (c) 2016 Titan Robotics Club (http://www.titanrobotics.com)
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

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfRect;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.CascadeClassifier;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;

import FtcSampleCode.R;
import ftclib.FtcOpMode;
import hallib.HalDashboard;
import trclib.TrcDbgTrace;

/**
 * This class implements an OpenCV view for Face Detection.
 */
@TeleOp(name="Test: OpenCV Face Detection", group="3543TestSamples")
@Disabled
public class FtcTestOpenCv extends FtcOpMode implements CameraBridgeViewBase.CvCameraViewListener2
{
    private static final String moduleName = "FtcTestOpenCv";
    private TrcDbgTrace tracer = FtcOpMode.getGlobalTracer();
    private static final boolean perfCheckEnabled = true;
    private static final boolean cameraEnabled = true;
    private static final Scalar FACE_RECT_COLOR = new Scalar(0, 255, 0, 255);
    private static final int classifier = R.raw.lbpcascade_frontalface;

    private HalDashboard dashboard;
    private FtcRobotControllerActivity activity;
//    private GLSurfaceView cameraPreview;
//    private ImageView overlayView;
    private CameraBridgeViewBase cameraView;
    private Mat overlayImage;
    private CascadeClassifier faceDetector;
    private Mat image;
    private MatOfRect faceRects;
    private long totalProcessingTime = 0;
    private long framesProcessed = 0;
    private boolean doColor = true;
    private boolean doOverlayImage = true;
    private boolean overlayRectangle = false;

    /**
     * Implements the OpenCV manager loader callback methods.
     */
    private BaseLoaderCallback loaderCallback =
            new BaseLoaderCallback(activity)
            {
                /**
                 * This method is called when the OpenCV manager is connected. It loads the
                 * OpenCV library and the appropriate CascadeClassifier.
                 *
                 * @param status specifies the OpenCV connection status.
                 */
                @Override
                public void onManagerConnected(int status)
                {
                    final String funcName = "onManagerConnected";

                    switch (status)
                    {
                        case LoaderCallbackInterface.SUCCESS:
                            File cascadeFile = null;
                            tracer.traceInfo(funcName, "OpenCV loaded successfully.");
                            System.loadLibrary("opencv_java3");
                            try
                            {
                                // load cascade file from application resources
                                InputStream is =
                                        activity.getResources().openRawResource(classifier);
                                File cascadeDir = activity.getDir("cascade", Context.MODE_PRIVATE);
                                cascadeFile = new File(cascadeDir, "frontalface.xml");
                                FileOutputStream os = new FileOutputStream(cascadeFile);

                                byte[] buffer = new byte[4096];
                                int bytesRead;
                                while ((bytesRead = is.read(buffer)) != -1)
                                {
                                    os.write(buffer, 0, bytesRead);
                                }
                                is.close();
                                os.close();

                                faceDetector = new CascadeClassifier(cascadeFile.getAbsolutePath());
                                if (faceDetector.empty())
                                {
                                    tracer.traceErr(
                                            funcName, "Failed to load cascade classifier <%s>",
                                            cascadeFile.getAbsolutePath());
                                    faceDetector = null;
                                }
                                else
                                {
                                    tracer.traceInfo(
                                            funcName, "Cascade classifier <%s> loaded!",
                                            cascadeFile.getAbsolutePath());
                                }
                                cascadeDir.delete();
                            }
                            catch (IOException e)
                            {
                                e.printStackTrace();
                                tracer.traceErr(
                                        funcName,
                                        "Failed to load cascade file <%s>",
                                        cascadeFile.getAbsolutePath());
                            }
//                            cameraPreview.onResume();
                            cameraView.enableView();
                            break;

                        default:
                            super.onManagerConnected(status);
                            break;
                    }
                }   //onManagerConnected
            };

    //
    // Implements FtcOpMode abstract methods.
    //

    @Override
    public void initRobot()
    {
        hardwareMap.logDevices();
        dashboard = HalDashboard.getInstance();
        this.activity = (FtcRobotControllerActivity)hardwareMap.appContext;
//        cameraPreview = (GLSurfaceView)activity.findViewById(R.id.CameraPreview);
//        overlayView = (ImageView)activity.findViewById(R.id.OverlayView);
        cameraView = (CameraBridgeViewBase)activity.findViewById(R.id.ImageView01);
        cameraView.setCameraIndex(1);   //use front camera
        if (cameraEnabled)
        {
            cameraView.setCvCameraViewListener(this);
        }
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode()
    {
        dashboard.clearDisplay();
        startCamera();
    }   //startMode

    @Override
    public void stopMode()
    {
        stopCamera();
    }   //stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        if (perfCheckEnabled && framesProcessed > 0)
        {
            dashboard.displayPrintf(
                    1, "Avg Processing Time = %d msec", totalProcessingTime/framesProcessed);
        }
    }   //runPeriodic

    private void startCamera()
    {
        final String funcName = "startCamera";

        if (cameraEnabled)
        {
            if (!OpenCVLoader.initDebug())
            {
                tracer.traceInfo(
                        funcName,
                        "Internal OpenCV library not found, using OpenCV Manager for initialization");
                OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, activity, loaderCallback);
            }
            else
            {
                tracer.traceInfo(funcName, "OpenCV library found inside package, use it!");
                loaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
            }
//            setCameraDisplayOrientation(activity, Camera.CameraInfo.CAMERA_FACING_FRONT);
//            cameraPreview.onResume();
        }
    }   //startCamera

    private void stopCamera()
    {
        final String funcName = "stopCamera";

        if (cameraEnabled)
        {
            cameraView.disableView();
            tracer.traceInfo(funcName, "Stopping camera!");
        }
    }   //stopCamera

    /*
    private void setCameraDisplayOrientation(Activity activity, int cameraType)
    {
        final String funcName = "setCameraDisplayOrientation";
        int rotation = activity.getWindowManager().getDefaultDisplay().getRotation();
        int degrees =
                rotation == Surface.ROTATION_270? 270:
                rotation == Surface.ROTATION_180? 180:
                rotation == Surface.ROTATION_90? 90: 0;
        Camera.CameraInfo info = new Camera.CameraInfo();

        for (int i = 0; i < Camera.getNumberOfCameras(); i++)
        {
            Camera.getCameraInfo(i, info);
            tracer.traceInfo(funcName, "[%d] orientation=%d,facing=%s\n",
                    i, info.orientation,
                    info.facing == Camera.CameraInfo.CAMERA_FACING_FRONT? "front": "back");
            if (info.facing == Camera.CameraInfo.CAMERA_FACING_FRONT)
            {
                int orientation;
                if (info.facing == Camera.CameraInfo.CAMERA_FACING_FRONT)
                {
                    orientation = (info.orientation + degrees) % 360;
                    orientation = (360 - orientation) % 360;    // compensate the mirror
                }
                else
                {
                    orientation = (info.orientation - degrees + 360) % 360;
                }
                Camera camera = Camera.open(i);
                camera.setDisplayOrientation(orientation);
                tracer.traceInfo(funcName, "DisplayRotation=%d, Orientation=%d\n",
                        degrees, orientation);
            }
        }
    }   //setCameraDisplayOrientation
    */

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
        faceRects = new MatOfRect();
        totalProcessingTime = 0;
        framesProcessed = 0;

        overlayImage = new Mat();
        Bitmap overlayBitmap =
                BitmapFactory.decodeResource(activity.getResources(), R.drawable.mustache);
        Utils.bitmapToMat(overlayBitmap, overlayImage);
        //
        // Don't allow overlay unless overlay image has the rgba channels.
        //
        if (overlayImage.channels() < 4) doOverlayImage = false;
    }   //onCameraViewStarted

    /**
     * This method is called when the camera view is stopped. It will clean up the allocated
     * global resources.
     */
    @Override
    public void onCameraViewStopped()
    {
        if (image != null) image.release();
        if (faceRects != null) faceRects.release();
        if (overlayImage != null) overlayImage.release();
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
        final String funcName = "onCameraFrame";
        //
        // Subject the captured frame for face detection. The face detector produces an array
        // of rectangles representing faces detected.
        //
        if (doColor)
        {
            image = inputFrame.rgba();
        }
        else
        {
            image = inputFrame.gray();
            doOverlayImage = false;
        }
//        rotateImage(image, image, 90.0);
        long startTime = System.currentTimeMillis();
        faceDetector.detectMultiScale(image, faceRects);
        long elapsedTime = System.currentTimeMillis() - startTime;

        if (perfCheckEnabled)
        {
            totalProcessingTime += elapsedTime;
            framesProcessed++;
        }

        //
        // We may want to overlay a circle or rectangle on each detected faces or
        // we can overlay a fun image onto a detected face. Play with the code in
        // this for-loop and let your imagination run wild.
        //

        Rect[] rects = faceRects.toArray();
        int maxArea = 0;
        int maxIndex = -1;
        //
        // Draw rectangles on faces found and find the largest face.
        //
        for (int i = 0; i < rects.length; i++)
        {
            //
            // Overlay a rectangle on the detected faces.
            //
            if (overlayRectangle)
            {
                Imgproc.rectangle(image, rects[i].tl(), rects[i].br(), FACE_RECT_COLOR, 3);
            }

            //
            // Find the largest detected face.
            //
            if (doOverlayImage)
            {
                int area = rects[i].width * rects[i].height;
                if (area > maxArea)
                {
                    maxArea = area;
                    maxIndex = i;
                }
            }
        }

        //
        // Overlay an image only on the largest detected face.
        //
        if (doOverlayImage && maxIndex != -1)
        {
            //
            // Scale the fun image to the same size as the face.
            //
            Mat scaledOverlay = new Mat();
            Imgproc.resize(overlayImage, scaledOverlay, rects[maxIndex].size());
            //
            // Overlay the scaled image to the camera image.
            //
            combineImage(image, scaledOverlay, rects[maxIndex].x, rects[maxIndex].y);
        }

        return image;
    }   //onCameraViewFrame

    /**
     * This method rotate the image to the specified angle.
     *
     * @param src specifies the image to be rotated.
     * @param dst specifies the destination to put the rotated image.
     * @param angle specifies the rotation angle.
     */
    private void rotateImage(Mat src, Mat dst, double angle)
    {
        angle %= 360.0;
        if (angle == 0.0)
        {
            src.copyTo(dst);
        }
        else if (angle == 90.0 || angle == -270.0)
        {
            Core.transpose(src, dst);
            Core.flip(dst, dst, 1);
        }
        else if (angle == 180.0 || angle == -180.0)
        {
            Core.flip(src, dst, -1);
        }
        else if (angle == 270.0 || angle == -90.0)
        {
            Core.transpose(src, dst);
            Core.flip(dst, dst, 0);
        }
        else
        {
            Mat rotMat = Imgproc.getRotationMatrix2D(
                    new Point(src.cols()/2.0, src.rows()/2.0), angle, 1.0);
            Imgproc.warpAffine(src, dst, rotMat, src.size());
        }
    }   //rotateImage

    /**
     * This method combines an overlay image to the given background image at the specified location.
     * It is expecting both the background and overlay are color images. It also expects the overlay
     * image contains an alpha channel for opacity information.
     *
     * @param background specifies the background image.
     * @param overlay specifies the overlay image.
     * @param locX specifies the X location on the background image where the upper left corner of the overlay
     *        image should be at
     * @param locY specifies the Y location on the backgorund image where the upper left corner of the overlay
     *        image should be at.
     */
    private void combineImage(Mat background, Mat overlay, int locX, int locY)
    {
        //
        // Make sure the background image has at least 3 channels and the overlay image has
        // at least 4 channels.
        //
        if (background.channels() >= 3 && overlay.channels() >= 4)
        {
            //
            // For each row of the overlay image.
            //
            for (int row = 0; row < overlay.rows(); row++)
            {
                //
                // Calculate the corresponding row number of the background image.
                // Skip the row if it is outside of the background image.
                //
                int destRow = locY + row;
                if (destRow < 0 || destRow >= background.rows()) continue;
                //
                // For each column of the overlay image.
                //
                for (int col = 0; col < overlay.cols(); col++)
                {
                    //
                    // Calculate the corresponding column number of background image.
                    // Skip the column if it is outside of the background image.
                    //
                    int destCol = locX + col;
                    if (destCol < 0 || destCol >= background.cols()) continue;
                    //
                    // Get the source pixel from the overlay image and the destination pixel from the
                    // background image. Calculate the opacity as a percentage.
                    //
                    double[] srcPixel = overlay.get(row,  col);
                    double[] destPixel = background.get(destRow, destCol);
                    double opacity = srcPixel[3]/255.0;
                    //
                    // Merge the source pixel to the destination pixel with the proper opacity.
                    // Each color pixel consists of 3 channels: BGR (Blue, Green, Red).
                    // The fourth channel is opacity and is only applicable for the overlay image.
                    //
                    for (int channel = 0; channel < 3; channel++)
                    {
                        destPixel[channel] = destPixel[channel]*(1.0 - opacity) + srcPixel[channel]*opacity;
                    }
                    //
                    // Put the resulting pixel into the background image.
                    //
                    background.put(destRow, destCol, destPixel);
                }
            }
        }
        else
        {
            throw new RuntimeException(
                    "Invalid image format (src=" + overlay.channels() + ",dest="
                    + background.channels() + ").");
        }
    }    //combineImage

}   //class FtcTestOpenCv
