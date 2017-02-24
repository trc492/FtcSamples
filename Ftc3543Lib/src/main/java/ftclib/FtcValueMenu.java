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

package ftclib;

import trclib.TrcDbgTrace;

/**
 * This class implements a value menu where a default value is displayed. The user can press the UP and DOWN button
 * to increase or decrease the value and press the ENTER button to select the value. The user can also press the
 * BACK button to cancel the menu and go back to the parent menu.
 */
public class FtcValueMenu extends FtcMenu
{
    private double minValue = 0.0;
    private double maxValue = 0.0;
    private double valueStep = 0.0;
    private double currValue = 0.0;
    private String valueFormat = null;
    private FtcMenu childMenu = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param menuTitle specifies the title of the menu. The title will be displayed as the first line in the menu.
     * @param parent specifies the parent menu to go back to if the BACK button is pressed. If this is the root menu,
     *               it can be set to null.
     * @param menuButtons specifies the object that implements the MenuButtons interface.
     * @param minValue specifies the minimum value of the value range.
     * @param maxValue specifies the maximum value of the value range.
     * @param valueStep specifies the value step.
     * @param defaultValue specifies the default value.
     * @param valueFormat specifies the format string for the value.
     */
    public FtcValueMenu(
            String menuTitle, FtcMenu parent, MenuButtons menuButtons, double minValue, double maxValue,
            double valueStep, double defaultValue, String valueFormat)
    {
        super(menuTitle, parent, menuButtons);
        this.minValue = minValue;
        this.maxValue = maxValue;
        this.valueStep = valueStep;
        this.currValue = defaultValue;
        this.valueFormat = valueFormat;
    }   //FtcValueMenu

    /**
     * This method sets the next menu to go to after pressing ENTER on the value menu.
     *
     * @param childMenu specifies the child menu.
     */
    public void setChildMenu(FtcMenu childMenu)
    {
        final String funcName = "setChildMenu";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "childMenu=%s", childMenu.getTitle());
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.childMenu = childMenu;
    }   //setChildMenu

    /**
     * This method returns the current value of the value menu. Every value menu has a current value even if the menu
     * hasn't been displayed and the user hasn't changed the value. In that case, the current value is the default
     * value.
     *
     * @return current value of the value menu.
     */
    public double getCurrentValue()
    {
        final String funcName = "getCurrentValue";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", currValue);
        }

        return currValue;
    }   //getCurrentValue

    //
    // Implements FtcMenu abstract methods.
    //

    /**
     * This method increases the current value by valueStep. If the value exceeds maxValue, it is capped at maxValue.
     */
    public void menuUp()
    {
        final String funcName = "menuUp";

        currValue += valueStep;
        if (currValue > maxValue)
        {
            currValue = maxValue;
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "! (value=%f)", currValue);
        }
    }   //menuUp

    /**
     * This method decreases the current value by valueStep. If the value is below minValue, it is capped at minValue.
     */
    public void menuDown()
    {
        final String funcName = "menuDown";

        currValue -= valueStep;
        if (currValue < minValue)
        {
            currValue = minValue;
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "! (value=%f)", currValue);
        }
    }   //menuDown

    /**
     * This method returns the child menu.
     *
     * @return child menu.
     */
    public FtcMenu getChildMenu()
    {
        final String funcName = "getChildMenu";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=%s", childMenu != null? childMenu.getTitle(): "null");
        }

        return childMenu;
    }   //getChildMenu

    /**
     * This method displays the menu on the dashboard with the current value in the specified format.
     */
    public void displayMenu()
    {
        final String funcName = "displayMenu";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        dashboard.clearDisplay();
        dashboard.displayPrintf(0, "%s" + valueFormat + "%s", getTitle(), currValue, childMenu != null? " ...": "");
    }   //displayMenu

}   //class FtcValueMenu
