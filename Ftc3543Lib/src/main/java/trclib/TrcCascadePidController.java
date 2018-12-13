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

package trclib;

/**
 * This class implements a Cascade PID Controller. A Cascade PID controller consists of two PID controllers in cascade.
 * The output of the primary PID controller feeds into the input of the secondary PID controller. If the motor is not
 * linear, it may be very difficult to get good performance out of a single PID controller. In Cascade PID control,
 * the distance set-point, for example, will produce a speed control as the primary output and feeds into the
 * secondary PID controller as input that will try to compensate for the non-linearity of the motor or even battery
 * level changes. The TrcCascadePidController class extends a regular PID control as its primary PID controller and
 * creates a second PID controller as its secondary controller.
 */
public class TrcCascadePidController extends TrcPidController
{
    public TrcPidController secondaryCtrl;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param primaryPidCoefficients specifies the PID coefficients of the primary PID controller.
     * @param primaryTolerance specifies the target tolerance of the primary PID controller.
     * @param primarySettlingTime specifies the target settling time of the primary PID controller.
     * @param secondaryPidCoefficients specifies the PID coefficients of the secondary PID controller.
     * @param secondaryTolerance specifies the target tolerance of the secondary PID controller.
     * @param secondarySettlingTime specifies the target settling time of the secondary PID controller.
     * @param primaryInput specifies the supplier of the primary PID input.
     * @param secondaryInput specifies the supplier of the secondary PID input.
     */
    public TrcCascadePidController(
            final String instanceName,
            PidCoefficients primaryPidCoefficients, double primaryTolerance, double primarySettlingTime,
            PidCoefficients secondaryPidCoefficients, double secondaryTolerance, double secondarySettlingTime,
            PidInput primaryInput, PidInput secondaryInput)
    {
        super(instanceName + ".primary",
              primaryPidCoefficients, primaryTolerance, primarySettlingTime, primaryInput);
        secondaryCtrl = new TrcPidController(
                instanceName + ".secondary", secondaryPidCoefficients, secondaryTolerance, secondarySettlingTime,
                secondaryInput);
    }   //TrcCascadePidController

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param primaryPidCoefficients specifies the PID coefficients of the primary PID controller.
     * @param primaryTolerance specifies the target tolerance of the primary PID controller.
     * @param secondaryPidCoefficients specifies the PID coefficients of the secondary PID controller.
     * @param secondaryTolerance specifies the target tolerance of the secondary PID controller.
     * @param primaryInput specifies the supplier of the primary PID input.
     * @param secondaryInput specifies the supplier of the secondary PID input.
     */
    public TrcCascadePidController(
            final String instanceName,
            PidCoefficients primaryPidCoefficients, double primaryTolerance,
            PidCoefficients secondaryPidCoefficients, double secondaryTolerance,
            PidInput primaryInput, PidInput secondaryInput)
    {
        this(instanceName,
             primaryPidCoefficients, primaryTolerance, DEF_SETTLING_TIME,
             secondaryPidCoefficients, secondaryTolerance, DEF_SETTLING_TIME,
             primaryInput, secondaryInput);
    }   //TrcCascadePidController

    /**
     * This method is called to reset the Cascade PID controller. It resets both the primary and secondary PID
     * controller.
     */
    @Override
    public synchronized void reset()
    {
        final String funcName = "reset";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        secondaryCtrl.reset();
        super.reset();
    }   //reset

    /**
     * This method calculates the Cascade PID control output by calling the primary PID controller, feeding its
     * output to the secondary PID controller and finally returning the output of the secondary PID controller.
     * @return output of the Cascade PID controller.
     */
    @Override
    public synchronized double getOutput()
    {
        final String funcName = "getOutput";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        double primaryOutput = super.getOutput();
        secondaryCtrl.setTarget(primaryOutput);
        double secondaryOutput = secondaryCtrl.getOutput();

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(primary:%f,secondary:%f", primaryOutput, secondaryOutput);
        }

        return secondaryOutput;
    }   //getOutput

}   //class TrcCascadePidController
