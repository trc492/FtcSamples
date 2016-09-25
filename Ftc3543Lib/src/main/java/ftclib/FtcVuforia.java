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

package ftclib;

import com.vuforia.HINT;
import com.vuforia.Trackable;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;

public class FtcVuforia
{
    private VuforiaLocalizer.Parameters vuforiaParams;
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables trackables;
    private ArrayList<VuforiaTrackable> allTargets = new ArrayList<>();
    private OpenGLMatrix phoneLocationOnRobot = null;

    public FtcVuforia(
            String licenseKey, int cameraViewId, VuforiaLocalizer.CameraDirection cameraDir,
            String trackablesFile, int numTargets)
    {
        vuforiaParams = new VuforiaLocalizer.Parameters(cameraViewId);
        vuforiaParams.vuforiaLicenseKey = licenseKey;
        vuforiaParams.cameraDirection = cameraDir;
        vuforia = ClassFactory.createVuforiaLocalizer(vuforiaParams);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, numTargets);
        trackables = vuforia.loadTrackablesFromAsset(trackablesFile);
    }   //FtcVuforia

    public void setEnabled(boolean enabled)
    {
        if (enabled)
        {
            trackables.activate();
        }
        else
        {
            trackables.deactivate();
        }
    }   //setEnable

    public void setPhoneLocationOnRobot(OpenGLMatrix phoneLocation)
    {
        phoneLocationOnRobot = phoneLocation;
    }   //setPhoneLocationOnRobot

    public void addTarget(int index, String name, OpenGLMatrix locationOnField)
    {
        VuforiaTrackable target = trackables.get(index);
        target.setName(name);

        if (locationOnField != null)
        {
            target.setLocation(locationOnField);
        }

        if (phoneLocationOnRobot != null)
        {
            ((VuforiaTrackableDefaultListener) target.getListener()).setPhoneInformation(
                    phoneLocationOnRobot, vuforiaParams.cameraDirection);
        }

        allTargets.add(target);
    }   //addTarget

    public void addTarget(int index, String name)
    {
        addTarget(index, name, null);
    }   //addTarget

    public VuforiaTrackable[] getTargets()
    {
        return allTargets.toArray(new VuforiaTrackable[allTargets.size()]);
    }   //getTargets

}   //class FtcVuforia
