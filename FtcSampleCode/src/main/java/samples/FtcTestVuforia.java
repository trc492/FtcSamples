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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import java.util.ArrayList;

import FtcSampleCode.R;
import ftclib.FtcOpMode;
import ftclib.FtcVuforia;
import hallib.HalDashboard;

@Autonomous(name="Test: Vuforia Navigation", group="Ftc3543Sample")
@Disabled
public class FtcTestVuforia extends FtcOpMode
{
    private final float MM_PER_INCH = 25.4f;
    private final float ROBOT_WIDTH = 18*MM_PER_INCH;               // in mm
    private final float FTC_FIELD_WIDTH = (12*12 - 2)*MM_PER_INCH;  // in mm

    private final String vuforiaLicenseKey = "ATsODcD/////AAAAAVw2lR...d45oGpdljdOh5LuFB9nDNfckoxb8COxKSFX";
    private final int cameraViewId = R.id.cameraMonitorViewId;
    private final VuforiaLocalizer.CameraDirection cameraDir = VuforiaLocalizer.CameraDirection.FRONT;
    private final String trackablesFile = "StonesAndChips";//"FTC_2016-17";

    private HalDashboard dashboard;
    private FtcVuforia vuforia;

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

        vuforia = new FtcVuforia(vuforiaLicenseKey, cameraViewId, cameraDir, trackablesFile);

        OpenGLMatrix phoneLocationOnRobot =
                OpenGLMatrix.translation(ROBOT_WIDTH/2, 0, 0)
                            .multiplied(Orientation.getRotationMatrix(
                                    AxesReference.EXTRINSIC, AxesOrder.YZY, AngleUnit.DEGREES, -90, 0, 0));
        vuforia.setPhoneLocationOnRobot(phoneLocationOnRobot);

        OpenGLMatrix stonesLocationOnField =
                OpenGLMatrix.translation(-FTC_FIELD_WIDTH/2, 0, 0)
                            .multiplied(Orientation.getRotationMatrix(
                                    AxesReference.EXTRINSIC, AxesOrder.XZX, AngleUnit.DEGREES, 90, 90, 0));
        vuforia.addTarget(0, "stones", stonesLocationOnField);

        OpenGLMatrix chipsLocationOnField =
                OpenGLMatrix.translation(0, FTC_FIELD_WIDTH/2, 0)
                            .multiplied(Orientation.getRotationMatrix(
                                    AxesReference.EXTRINSIC, AxesOrder.XZX, AngleUnit.DEGREES, 90, 0, 0));
        vuforia.addTarget(1, "chips", chipsLocationOnField);
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode()
    {
        dashboard.clearDisplay();
        vuforia.setEnabled(true);
    }   //startMode

    @Override
    public void stopMode()
    {
        vuforia.setEnabled(false);
    }   //stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        final int LABEL_WIDTH = 100;
        ArrayList<VuforiaTrackable> targets = vuforia.getTargets();

        for (int i = 0; i < targets.size(); i++)
        {
            VuforiaTrackable target = targets.get(i);
            VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener)(target.getListener());
            OpenGLMatrix locationTransform = listener.getUpdatedRobotLocation();

            dashboard.displayPrintf(
                    i + 1, LABEL_WIDTH, target.getName(),
                    "visible=%s, pos=%s",
                    Boolean.toString(listener.isVisible()),
                    locationTransform == null? "unknown": locationTransform.formatAsTransform());
        }
    }   //runPeriodic

}   //class FtcTestVuforia
