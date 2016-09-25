/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

package samples;

import android.widget.TextView;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import FtcSampleCode.R;
import ftclib.FtcOpMode;
import ftclib.FtcVuforia;
import hallib.HalDashboard;

@Autonomous(name="Test: Vuforia Targets Tracking", group="Ftc3543Sample")
//@Disabled
public class FtcTestVuforia extends FtcOpMode
{
    private final boolean trackRobotLocation = false;
    private final float MM_PER_INCH = 25.4f;
    private final float ROBOT_WIDTH = 18*MM_PER_INCH;               // in mm
    private final float FTC_FIELD_WIDTH = (12*12 - 2)*MM_PER_INCH;  // in mm
    private final float TARGET_HEIGHT = 160.0f;                     // in mm
    //
    // If you copy our code, please register your own account and generate your own license key at this site:
    // https://developer.vuforia.com/license-manager
    //
    private final String VUFORIA_LICENSE_KEY =
            "AdCwzDH/////AAAAGeDkDS3ukU9+lIXc19LMh+cKk29caNhOl8UqmZOymRGwVwT1ZN8uaPdE3Q+zceDu9AKNsqL9qLblSFV" +
            "/x8Y3jfOZdjMFs0CQSQOEyWv3xfJsdSmevXDQDQr+4KI31HY2YSf/KB/kyxfuRMk4Pi+vWS+oLl65o7sWPiyFgzoM74ENyb" +
            "j4FgteD/2b6B+UFuwkHWKBNpp18wrpkaiFfr/FCbRFcdWP5mrjlEZM6eOj171dybw97HPeZbGihnnxOeeUv075O7P167AVq" +
            "aiPy2eRK7OCubR32KXOqQKoyF6AXp+qu2cOTApXS5dqOOseEm+HE4eMF0S2Pld3i5AWBIR+JlPXDuc9LwoH2Q8iDwUK1+4g";
    private final int CAMERAVIEW_ID = R.id.cameraMonitorViewId;
    private final VuforiaLocalizer.CameraDirection CAMERA_DIR = VuforiaLocalizer.CameraDirection.BACK;
    private final String TRACKABLES_FILE = "FTC_2016-17";

    private HalDashboard dashboard;
    private FtcVuforia vuforia;
    private VuforiaTrackables targets;

    //
    // Implements FtcOpMode abstract method.
    //

    @Override
    public void initRobot()
    {
        hardwareMap.logDevices();
        dashboard = getDashboard();
        FtcRobotControllerActivity activity = (FtcRobotControllerActivity)hardwareMap.appContext;
        dashboard.setTextView((TextView)activity.findViewById(R.id.textOpMode));

        OpenGLMatrix phoneLocationOnRobot = null;
        OpenGLMatrix wheelsLocationOnField = null;
        OpenGLMatrix toolsLocationOnField = null;
        OpenGLMatrix legosLocationOnField = null;
        OpenGLMatrix gearsLocationOnField = null;

        vuforia = new FtcVuforia(VUFORIA_LICENSE_KEY, CAMERAVIEW_ID, CAMERA_DIR, TRACKABLES_FILE, 4);

        if (trackRobotLocation)
        {
            //
            // Camera location.
            //
            phoneLocationOnRobot =
                    OpenGLMatrix.translation(ROBOT_WIDTH / 2, 0, 0)
                            .multiplied(Orientation.getRotationMatrix(
                                    AxesReference.EXTRINSIC, AxesOrder.YZY, AngleUnit.DEGREES, -90, 0, 0));
            //
            // Red alliance beacon 1 location.
            //
            wheelsLocationOnField =
                    OpenGLMatrix.translation(-FTC_FIELD_WIDTH / 2, -12 * MM_PER_INCH, TARGET_HEIGHT)
                            .multiplied(Orientation.getRotationMatrix(
                                    AxesReference.EXTRINSIC, AxesOrder.XZX, AngleUnit.DEGREES, 90, 90, 0));
            //
            // Red alliance beacon 2 location.
            //
            toolsLocationOnField =
                    OpenGLMatrix.translation(-FTC_FIELD_WIDTH / 2, 30 * MM_PER_INCH, TARGET_HEIGHT)
                            .multiplied(Orientation.getRotationMatrix(
                                    AxesReference.EXTRINSIC, AxesOrder.XZX, AngleUnit.DEGREES, 90, 90, 0));
            //
            // Blue alliance beacon 1 location.
            //
            legosLocationOnField =
                    OpenGLMatrix.translation(12 * MM_PER_INCH, FTC_FIELD_WIDTH / 2, TARGET_HEIGHT)
                            .multiplied(Orientation.getRotationMatrix(
                                    AxesReference.EXTRINSIC, AxesOrder.XZX, AngleUnit.DEGREES, 90, 0, 0));
            //
            // Blue alliance beacon 2 location.
            //
            gearsLocationOnField =
                    OpenGLMatrix.translation(-30 * MM_PER_INCH, FTC_FIELD_WIDTH / 2, TARGET_HEIGHT)
                            .multiplied(Orientation.getRotationMatrix(
                                    AxesReference.EXTRINSIC, AxesOrder.XZX, AngleUnit.DEGREES, 90, 0, 0));
        }

        vuforia.setTarget(0, "wheels", wheelsLocationOnField, phoneLocationOnRobot);
        vuforia.setTarget(1, "tools", toolsLocationOnField, phoneLocationOnRobot);
        vuforia.setTarget(2, "legos", legosLocationOnField, phoneLocationOnRobot);
        vuforia.setTarget(3, "gears", gearsLocationOnField, phoneLocationOnRobot);

        targets = vuforia.getTargets();
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode()
    {
        dashboard.clearDisplay();
        vuforia.setTrackingEnabled(true);
    }   //startMode

    @Override
    public void stopMode()
    {
        vuforia.setTrackingEnabled(false);
    }   //stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        final int LABEL_WIDTH = 100;

        for (int i = 0; i < targets.size(); i++)
        {
            VuforiaTrackable target = targets.get(i);
            VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener)target.getListener();
            OpenGLMatrix pose = listener.getPose();

            dashboard.displayPrintf(
                    i*2 + 1, LABEL_WIDTH, target.getName() + ": ", "%s", listener.isVisible()? "Found": "NotFound");
            if (pose != null)
            {
                VectorF translation = pose.getTranslation();
                dashboard.displayPrintf(
                        i*2 + 2, LABEL_WIDTH, "x,y,z in. = ", "%6.2f,%6.2f,%6.2f",
                        translation.get(0)/MM_PER_INCH,
                        translation.get(1)/MM_PER_INCH,
                        -translation.get(2)/MM_PER_INCH);
            }
        }
    }   //runPeriodic

}   //class FtcTestVuforia
