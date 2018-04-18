/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
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
 * This class implements a platform independent generic analog sensor. Anything that produces analog data can use
 * this class to make itself an analog sensor that conforms to the TrcSensor class which can be used as an analog
 * trigger. To make itself an analog sensor, it inherits from the AnalogInput class and thus must provide the
 * getRawData abstract method required by TrcAnalogInput. In getRawData, it will call the AnalogDataSource
 * provided in its constructor to get the raw data.
 */
public class TrcAnalogSensor extends TrcAnalogInput
{
    /**
     * This interface is used by this class to get the analog data from the provider.
     */
    public interface AnalogDataSource
    {
        /**
         * This method returns the raw data from the analog data source.
         *
         * @return raw analog data.
         */
        Double getData();
    }   //interface AnalogDataSource

    private final AnalogDataSource dataSource;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param dataSource specifies the analog data provider.
     */
    public TrcAnalogSensor(final String instanceName, final AnalogDataSource dataSource)
    {
        super(instanceName, 1, 0, null);

        this.dataSource = dataSource;
    }   //TrcAnalogSensor

    /**
     * This abstract method returns the raw data with the specified index and type.
     *
     * @param index specifies the data index (not used because AnalogSensor has only one axis).
     * @param dataType specifies the data type (not used because AnalogSensor only returns raw data).
     * @return raw data from the analog data source.
     */
    public SensorData<Double> getRawData(int index, DataType dataType)
    {
        final String funcName = "getRawData";
        Double rawData = dataSource.getData();
        SensorData<Double> data = rawData != null? new SensorData<>(TrcUtil.getCurrentTime(), rawData): null;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "index=%d,type=%s", index, dataType);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=(%.3f,%s)",
                data.timestamp, data == null? "null": "" + data.value);
        }

        return data ;
    }   //getRawData

}   //class TrcAnalogSensor
