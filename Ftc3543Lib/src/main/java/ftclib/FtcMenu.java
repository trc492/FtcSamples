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

package ftclib;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import hallib.HalDashboard;
import trclib.TrcDbgTrace;
import trclib.TrcUtil;

/**
 * This class is intended to be inherited by a specific menu class such as FtcChoiceMenu or FtcValueMenu. Therefore,
 * this class cannot be instantiated by itself. It implements a display menu system. It allows you to construct a
 * menu tree structure where a menu is displayed on the Driver Station using the Dashboard class. On a choice menu,
 * the user can press the up and down buttons to change the highlighted choice on the menu and then press enter to
 * select the highlighted choice. On a value menu, the user can press the up and down button to increase or decrease
 * the value and then press enter to select the current value. After the choice is made, it will move on to the next
 * menu in the menu tree. Or if the user presses the back button to cancel the menu, it will go back to the previous
 * menu in the menu tree. This is very useful in autonomous allowing the user to select from different autonomous
 * strategies and also select the options for each autonomous strategy. For example, one could have a menu to select
 * between being in the RED alliance or the BLUE alliance. A menu to select the robot starting position. A menu to
 * select the autonomous strategy. A menu to select the delay for starting the strategy etc.
 */
public abstract class FtcMenu
{
    protected static final String moduleName = "FtcMenu";
    protected static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    protected TrcDbgTrace dbgTrace = null;

    /**
     * This method allows this class to displays the menu on the Driver Station.
     */
    public abstract void displayMenu();

    /**
     * This method allows this class to get the child menu.
     *
     * @return child menu.
     */
    public abstract FtcMenu getChildMenu();

    /**
     * This method allows this class to signal to the subclass that a menu UP button has been pressed so it will
     * perform the necessary operation on it.
     */
    public abstract void menuUp();

    /**
     * This method allows this class to signal to the subclass that a menu DOWN button has been pressed so it will
     * perform the necessary operation on it.
     */
    public abstract void menuDown();

    /**
     * The user of this class is required to implement the MenuButtons interface. The methods in this interface
     * allows this class to check for button activities the user made without hard coding what particular buttons
     * are associated with up/down/enter/back. So you can associate the activities with gamepad buttons or even
     * other input devices.
     */
    public interface MenuButtons
    {
        /**
         * This method is called by this class to check if the UP button is pressed.
         *
         * @return true if the UP button is pressed, false otherwise.
         */
        boolean isMenuUpButton();

        /**
         * This method is called by this class to check if the DOWN button is pressed.
         *
         * @return true if the DOWN button is pressed, false otherwise.
         */
        boolean isMenuDownButton();

        /**
         * This method is called by this class to check if the ENTER button is pressed.
         *
         * @return true if the ENTER button is pressed, false otherwise.
         */
        boolean isMenuEnterButton();

        /**
         * This method is called by this class to check if the BACK button is pressed.
         *
         * @return true if the BACK button is pressed, false otherwise.
         */
        boolean isMenuBackButton();

    }   //interface MenuButtons

    private static final long LOOP_INTERVAL     = 20;       //in msec.

    private static final int MENUBUTTON_BACK    = (1 << 0);
    private static final int MENUBUTTON_ENTER   = (1 << 1);
    private static final int MENUBUTTON_UP      = (1 << 2);
    private static final int MENUBUTTON_DOWN    = (1 << 3);

    protected HalDashboard dashboard;
    private String menuTitle;
    private FtcMenu parent;
    private MenuButtons menuButtons;

    private static int prevButtonStates = 0;
    private static FtcMenu currMenu = null;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param menuTitle specifies the title of the menu. The title will be displayed as the first line in the menu.
     * @param parent specifies the parent menu to go back to if the BACK button is pressed. If this is the root menu,
     *               it can be set to null.
     * @param menuButtons specifies the object that implements the MenuButtons interface.
     */
    protected FtcMenu(String menuTitle, FtcMenu parent, MenuButtons menuButtons)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + menuTitle, tracingEnabled, traceLevel, msgLevel);
        }

        if (menuButtons == null || menuTitle == null)
        {
            throw new NullPointerException("menuTitle/menuButtons cannot be null.");
        }

        dashboard = HalDashboard.getInstance();
        this.menuTitle = menuTitle;
        this.parent = parent;
        this.menuButtons = menuButtons;
    }   //FtcMenu

    /**
     * This method returns the parent menu of this menu.
     *
     * @return parent menu (can be null if this menu is the root menu).
     */
    public FtcMenu getParentMenu()
    {
        final String funcName = "getParentMenu";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", parent.getTitle());
        }

        return parent;
    }   //getParentMenu

    /**
     * This method returns the title text of this menu.
     *
     * @return title text.
     */
    public String getTitle()
    {
        final String funcName = "getTitle";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", menuTitle);
        }

        return menuTitle;
    }   //getTitle

    /**
     * This method sets the current menu to the specified root menu. Typically, this is called in conjunction with
     * the runMenus() method to use the FtcMenu module in a non-blocking environment.
     *
     * @param rootMenu specifies the root menu.
     */
    public static void setRootMenu(FtcMenu rootMenu)
    {
        currMenu = rootMenu;
    }   //setRootMenu

    /**
     *
     */
    /**
     * This method traverses the menu tree from the given root menu displaying each menu and waiting for the user
     * to respond to a menu. After the user responded to a menu, it will go to the next menu in the tree. If the
     * user cancels the menu, it will go back to the parent menu where it came from. If there is no next menu, the
     * traversal is ended. Note: this is a static method, meaning you can call it without a menu instance. Also note
     * that this is a blocking call so this should not be called in a multitasking robot loop such as in the execution
     * of a state machine. To use the menus in a multitasking environment, you must use the runMenus() method instead.
     *
     * @param rootMenu specifies the root of the menu tree.
     * @param opmode specifies the linear opmode that calls this.
     */
    public static void walkMenuTree(FtcMenu rootMenu, LinearOpMode opmode)
    {
        setRootMenu(rootMenu);
        rootMenu.displayMenu();
        while (!runMenus() && !opmode.isStopRequested())
        {
            TrcUtil.sleep(LOOP_INTERVAL);
        }
    }   //walkMenuTree

    /**
     * This method walks the menu tree in a non-blocking environment. It means this method must be called periodically,
     * so that the caller can perform other tasks if necessary.
     *
     * @return true if the user traverses to the leave node of the menu tree, false if the caller must call this again
     *         in a loop.
     */
    public static boolean runMenus()
    {
        boolean done = false;

        if (currMenu == null)
        {
            done = true;
        }
        else
        {
            int currButtonStates = currMenu.getMenuButtons();
            int changedButtons = currButtonStates^prevButtonStates;
            //
            // Check if any menu buttons changed states.
            //
            if (changedButtons != 0)
            {
                int buttonsPressed = currButtonStates & changedButtons;

                if ((buttonsPressed & MENUBUTTON_BACK) != 0)
                {
                    //
                    // MenuBack is pressed, goto parent menu unless it's already the root menu. If at root menu,
                    // stay on it.
                    //
                    FtcMenu parentMenu = currMenu.getParentMenu();
                    if (parentMenu != null)
                    {
                        currMenu = parentMenu;
                    }
                }
                else if ((buttonsPressed & MENUBUTTON_ENTER) != 0)
                {
                    //
                    // MenuEnter is pressed, goto the child menu. If there is none, we are done.
                    //
                    currMenu = currMenu.getChildMenu();
                }
                else if ((buttonsPressed & MENUBUTTON_UP) != 0)
                {
                    currMenu.menuUp();
                }
                else if ((buttonsPressed & MENUBUTTON_DOWN) != 0)
                {
                    currMenu.menuDown();
                }
                //
                // Refresh the display to update the menu state.
                //
                if (currMenu != null)
                {
                    currMenu.displayMenu();
                }
                else
                {
                    //
                    // We are done with the menus. Let's clear the dashboard.
                    //
                    HalDashboard.getInstance().clearDisplay();
                }
                prevButtonStates = currButtonStates;
            }
        }

        return done;
    }   //runMenus

    /**
     * This method checks all the menu button states and combine them into an integer, one bit for each button.
     *
     * @return an integer representing the states of all the menu buttons.
     */
    private int getMenuButtons()
    {
        final String funcName = "getMenuButtons";

        int buttons = 0;

        if (menuButtons.isMenuBackButton()) buttons |= MENUBUTTON_BACK;
        if (menuButtons.isMenuEnterButton()) buttons |= MENUBUTTON_ENTER;
        if (menuButtons.isMenuUpButton()) buttons |= MENUBUTTON_UP;
        if (menuButtons.isMenuDownButton()) buttons |= MENUBUTTON_DOWN;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC, "=%x", buttons);
        }

        return buttons;
    }   //getMenuButtons

}   //class FtcMenu
