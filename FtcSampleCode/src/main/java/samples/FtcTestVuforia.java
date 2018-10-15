/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
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

import android.speech.tts.TextToSpeech;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import java.util.Locale;

import FtcSampleCode.R;
import ftclib.FtcOpMode;
import ftclib.FtcVuforia;
import hallib.HalDashboard;
import trclib.TrcRobot;
import trclib.TrcUtil;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

@TeleOp(name="Test: Vuforia Targets Tracking", group="3543TestSamples")
@Disabled
public class FtcTestVuforia extends FtcOpMode
{
    private static final boolean SPEECH_ENABLED = true;

    private static final int IMAGE_WIDTH = 640;
    private static final int IMAGE_HEIGHT = 480;
    private static final int FRAME_QUEUE_CAPACITY = 2;
    private static final int CAMERA_FORWARD_DISPLACEMENT  = 110;    //Camera is 110 mm in front of robot center
    private static final int CAMERA_VERTICAL_DISPLACEMENT = 200;    //Camera is 200 mm above ground
    private static final int CAMERA_LEFT_DISPLACEMENT     = 0;      //Camera is ON the robot's center line
    private static final float FTC_FIELD_WIDTH_MM = (float)(6.0*12.0*TrcUtil.MM_PER_INCH);
    private static final float TARGET_HEIGHT_MM = (float)(6.0*TrcUtil.MM_PER_INCH);
    //
    // If you copy our code, please register your own account and generate your own free license key at this site:
    // https://developer.vuforia.com/license-manager
    //
    private static final String VUFORIA_LICENSE_KEY =
            "AdCwzDH/////AAAAGeDkDS3ukU9+lIXc19LMh+cKk29caNhOl8UqmZOymRGwVwT1ZN8uaPdE3Q+zceDu9AKNsqL9qLblSFV" +
            "/x8Y3jfOZdjMFs0CQSQOEyWv3xfJsdSmevXDQDQr+4KI31HY2YSf/KB/kyxfuRMk4Pi+vWS+oLl65o7sWPiyFgzoM74ENyb" +
            "j4FgteD/2b6B+UFuwkHWKBNpp18wrpkaiFfr/FCbRFcdWP5mrjlEZM6eOj171dybw97HPeZbGihnnxOeeUv075O7P167AVq" +
            "aiPy2eRK7OCubR32KXOqQKoyF6AXp+qu2cOTApXS5dqOOseEm+HE4eMF0S2Pld3i5AWBIR+JlPXDuc9LwoH2Q8iDwUK1+4g";
    private static final int CAMERAVIEW_ID = R.id.cameraMonitorViewId;
    private static final VuforiaLocalizer.CameraDirection CAMERA_DIR = VuforiaLocalizer.CameraDirection.BACK;
    private static final String TRACKABLE_IMAGES_FILE = "RoverRuckus";

    private HalDashboard dashboard;
    private FtcVuforia vuforia;
    private VuforiaTrackable[] imageTargets = null;
    private boolean[] targetsFound = null;
    private OpenGLMatrix lastRobotLocation = null;
    private TextToSpeech textToSpeech = null;

    //
    // Implements FtcOpMode abstract methods.
    //

    @Override
    public void initRobot()
    {
        hardwareMap.logDevices();
        dashboard = HalDashboard.getInstance();
        FtcRobotControllerActivity activity = (FtcRobotControllerActivity)hardwareMap.appContext;
        dashboard.setTextView(activity.findViewById(R.id.textOpMode));

        vuforia = new FtcVuforia(VUFORIA_LICENSE_KEY, CAMERAVIEW_ID, CAMERA_DIR);
        vuforia.configVideoSource(IMAGE_WIDTH, IMAGE_HEIGHT, FRAME_QUEUE_CAPACITY);
        /*
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm
         * above ground level.
         */
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT,
                             CAMERA_LEFT_DISPLACEMENT,
                             CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_DIR == FRONT ? 90 : -90, 0, 0));
        /*
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot. These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        /*
         * To place the BlueRover target in the middle of the blue perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Then, we translate it along the Y axis to the blue perimeter wall.
         */
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, FTC_FIELD_WIDTH_MM, TARGET_HEIGHT_MM)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));

        /*
         * To place the RedFootprint target in the middle of the red perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative Y axis to the red perimeter wall.
         */
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -FTC_FIELD_WIDTH_MM, TARGET_HEIGHT_MM)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));

        /*
         * To place the FrontCraters target in the middle of the front perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative X axis to the front perimeter wall.
         */
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-FTC_FIELD_WIDTH_MM, 0, TARGET_HEIGHT_MM)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));

        /*
         * To place the BackSpace target in the middle of the back perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the X axis to the back perimeter wall.
         */
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(FTC_FIELD_WIDTH_MM, 0, TARGET_HEIGHT_MM)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));

        FtcVuforia.TargetInfo[] imageTargetsInfo =
        {
                new FtcVuforia.TargetInfo(0, "Blue-Rover", false, blueRoverLocationOnField),
                new FtcVuforia.TargetInfo(1, "Red-Footprint", false, redFootprintLocationOnField),
                new FtcVuforia.TargetInfo(2, "Front-Craters", false, frontCratersLocationOnField),
                new FtcVuforia.TargetInfo(3, "Back-Space", false, backSpaceLocationOnField)
        };

        vuforia.addTargetList(TRACKABLE_IMAGES_FILE, imageTargetsInfo, phoneLocationOnRobot);
        imageTargets = new VuforiaTrackable[imageTargetsInfo.length];
        for (int i = 0; i < imageTargets.length; i++)
        {
            imageTargets[i] = vuforia.getTarget(imageTargetsInfo[i].name);
        }

        //
        // Text To Speech.
        //
        if (SPEECH_ENABLED)
        {
            textToSpeech = new TextToSpeech(
                    hardwareMap.appContext,
                    status ->
                    {
                        if (status != TextToSpeech.ERROR)
                        {
                            textToSpeech.setLanguage(Locale.US);
                        }
                    });
            targetsFound = new boolean[imageTargets.length];
            for (int i = 0; i < targetsFound.length; i++)
            {
                targetsFound[i] = false;
            }
        }
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode(TrcRobot.RunMode prevMode)
    {
        dashboard.clearDisplay();
        vuforia.setTrackingEnabled(true);
    }   //startMode

    @Override
    public void stopMode(TrcRobot.RunMode nextMode)
    {
        vuforia.setTrackingEnabled(false);
        if (textToSpeech != null)
        {
            textToSpeech.stop();
            textToSpeech.shutdown();
        }
    }   //stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        final int LABEL_WIDTH = 120;

        for (int i = 0; i < imageTargets.length; i++)
        {
            boolean visible = vuforia.isTargetVisible(imageTargets[i]);

            if (SPEECH_ENABLED)
            {
                if (visible != targetsFound[i])
                {
                    targetsFound[i] = visible;
                    String sentence = String.format(
                            "%s is %s.", imageTargets[i].getName(), visible? "in view": "out of view");
                    textToSpeech.speak(sentence, TextToSpeech.QUEUE_FLUSH, null);
                }
            }

            OpenGLMatrix pose = vuforia.getTargetPose(imageTargets[i]);
            if (pose != null)
            {
                VectorF translation = pose.getTranslation();
                dashboard.displayPrintf(
                        i + 1, LABEL_WIDTH, imageTargets[i].getName() + " = ", "%6.2f,%6.2f,%6.2f",
                        translation.get(0)/TrcUtil.MM_PER_INCH,
                        translation.get(1)/TrcUtil.MM_PER_INCH,
                        -translation.get(2)/TrcUtil.MM_PER_INCH);
            }

            OpenGLMatrix robotLocation = vuforia.getRobotLocation(imageTargets[i]);
            if (robotLocation != null)
            {
                lastRobotLocation = robotLocation;
            }
        }

        if (lastRobotLocation != null)
        {
            dashboard.displayPrintf(5, LABEL_WIDTH, "RobotLoc = ",
                    lastRobotLocation.formatAsTransform());
        }
    }   //runPeriodic

}   //class FtcTestVuforia
