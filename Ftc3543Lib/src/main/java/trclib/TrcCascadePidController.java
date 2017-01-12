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
public class TrcCascadePidController extends TrcPidController implements TrcPidController.PidInput
{
    private static final String moduleName = "TrcCascadePidController";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    /**
     * This interface provides methods allowing this class to get the input of either the primary or secondary
     * PID controllers. For example, the primary PID controller input may be the distance traveled and the secondary
     * PID controller input may be the velocity.
     */
    public interface CascadeInput
    {
        /**
         * This method is called to get the primary PID controller input.
         *
         * @param cascadeCtrl specifies this cascade controller that needs the input.
         * @return primary PID controller input value.
         */
        double getPrimaryInput(TrcCascadePidController cascadeCtrl);

        /**
         * This method is called to get the secondary PID controller input.
         *
         * @param cascadeCtrl specifies this cascade controller that needs the input.
         * @return secondary PID controller input value.
         */
        double getSecondaryInput(TrcCascadePidController cascadeCtrl);

    }   //interface CascadeInput

    public TrcPidController secondaryCtrl;
    private CascadeInput cascadeInput;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param primaryKp specifies the Proportional constant of the primary PID controller.
     * @param primaryKi specifies the Integral constant of the primary PID controller.
     * @param primaryKd specifies the Differential constant of the primary PID controller.
     * @param primaryKf specifies the Feed Forward constant of the primary PID controller.
     * @param primaryTolerance specifies the target tolerance of the primary PID controller.
     * @param primarySettlingTime specifies the target settling time of the primary PID controller.
     * @param secondaryKp specifies the Proportional constant of the secondary PID controller.
     * @param secondaryKi specifies the Integral constant of the secondary PID controller.
     * @param secondaryKd specifies the Differential constant of the secondary PID controller.
     * @param secondaryKf specifies the Feed Forward constant of the secondary PID controller.
     * @param secondaryTolerance specifies the target tolerance of the secondary PID controller.
     * @param secondarySettlingTime specifies the target settling time of the secondary PID controller.
     * @param cascadeInput specifies the object that will provide the Cascade control inputs.
     */
    public TrcCascadePidController(
            final String instanceName,
            double primaryKp, double primaryKi, double primaryKd, double primaryKf,
            double primaryTolerance, double primarySettlingTime,
            double secondaryKp, double secondaryKi, double secondaryKd, double secondaryKf,
            double secondaryTolerance, double secondarySettlingTime,
            CascadeInput cascadeInput)
    {
        super(instanceName + ".primary",
              primaryKp, primaryKi, primaryKd, primaryKf, primaryTolerance, primarySettlingTime);
        super.setPidInput(this);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        secondaryCtrl = new TrcPidController(instanceName + ".secondary",
                                             secondaryKp, secondaryKi, secondaryKd, secondaryKf,
                                             secondaryTolerance, secondarySettlingTime, this);
        this.cascadeInput = cascadeInput;
    }   //TrcCascadePidController

    /**
     * This method is called to reset the Cascade PID controller. It resets both the primary and secondary PID
     * controller.
     */
    @Override
    public void reset()
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
    public double getOutput()
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

    //
    // Implements TrcPidController.PidInput
    //

    /**
     * This method is called to get the input of either the primary of the secondary controller.
     *
     * @param pidCtrl specifies whether the primary or the secondary PID controller that is answer for input.
     * @return primary or secondary PID controller input.
     */
    @Override
    public double getInput(TrcPidController pidCtrl)
    {
        double input = 0.0;

        if (pidCtrl == this)
        {
            input = cascadeInput.getPrimaryInput(this);
        }
        else if (pidCtrl == secondaryCtrl)
        {
            input = cascadeInput.getSecondaryInput(this);
        }

        return input;
    }   //getInput

}   //class TrcCascadePidController
