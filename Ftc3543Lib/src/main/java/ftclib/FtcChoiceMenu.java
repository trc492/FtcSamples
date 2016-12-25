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

import java.util.ArrayList;

import hallib.HalDashboard;
import trclib.TrcDbgTrace;

/**
 * This class implements a choice menu where a number of choices are presented to the user.
 * The user can press the UP and DOWN button to navigate the different choices and press the
 * ENTER button to select the choice. The user can also press the BACK button to cancel the
 * menu and go back to the parent menu.
 */
public class FtcChoiceMenu<T> extends FtcMenu
{
    /**
     * This class defines a choice item in a choice menu.
     */
    private class ChoiceItem
    {
        private String choiceText;
        private T choiceObject;
        private FtcMenu childMenu;

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param choiceText specifies the text to be displayed in the choice menu.
         * @param choiceObject specifies the object to be returned if the choice is selected.
         * @param childMenu specifies the next menu to go to if the choice is selected. It can
         *                  be null if this is the end (i.e. leaf node of the menu tree).
         */
        public ChoiceItem(String choiceText, T choiceObject, FtcMenu childMenu)
        {
            this.choiceText = choiceText;
            this.choiceObject = choiceObject;
            this.childMenu = childMenu;
        }   //ChoiceItem

        /**
         * This method returns the choice text.
         *
         * @return choice text.
         */
        public String getText()
        {
            return choiceText;
        }   //getText;

        /**
         * This method returns the choice object.
         *
         * @return choice object.
         */
        public T getObject()
        {
            return choiceObject;
        }   //getObject

        /**
         * This method returns the child menu.
         *
         * @return child menu.
         */
        public FtcMenu getChildMenu()
        {
            return childMenu;
        }   //getChildMenu

    }   //class ChoiceItem

    private ArrayList<ChoiceItem> choiceItems = new ArrayList<ChoiceItem>();
    private int currChoice = -1;
    private int firstDisplayedChoice = 0;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param menuTitle specifies the title of the menu. The title will be displayed
     *                  as the first line in the menu.
     * @param parent specifies the parent menu to go back to if the BACK button
     *               is pressed. If this is the root menu, it can be set to null.
     * @param menuButtons specifies the object that implements the MenuButtons interface.
     */
    public FtcChoiceMenu(String menuTitle, FtcMenu parent, MenuButtons menuButtons)
    {
        super(menuTitle, parent, menuButtons);
    }   //FtcMenu

    /**
     * This method adds a choice to the menu. The choices will be displayed in the
     * order of them being added.
     *
     * @param choiceText specifies the choice text that will be displayed on the dashboard.
     * @param choiceObject specifies the object to be returned if the choice is selected.
     * @param childMenu specifies the next menu to go to when this choice is selected.
     *                  If this is the last menu (a leaf node in the tree), it can be set
     *                  to null.
     */
    public void addChoice(String choiceText, T choiceObject, FtcMenu childMenu)
    {
        final String funcName = "addChoice";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "text=%s,obj=%s,child=%s",
                    choiceText, choiceObject.toString(),
                    childMenu == null? "null": childMenu.getTitle());
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        choiceItems.add(new ChoiceItem(choiceText, choiceObject, childMenu));
        if (currChoice == -1)
        {
            //
            // This is the first added choice in the menu.
            // Make it the default choice by highlighting it.
            //
            currChoice = 0;
        }
    }   //addChoice

    /**
     * This method adds a choice to the menu. The choices will be displayed in the
     * order of them being added.
     *
     * @param choiceText specifies the choice text that will be displayed on the dashboard.
     * @param choiceObj specifies the object to be returned if the choice is selected.
     */
    public void addChoice(String choiceText, T choiceObj)
    {
        addChoice(choiceText, choiceObj, null);
    }   //addChoice

    /**
     * This method returns the choice text of the given choice index.
     *
     * @param choice specifies the choice index in the menu.
     * @return text of the choice if choice index is valid, null otherwise.
     */
    public String getChoiceText(int choice)
    {
        final String funcName = "getChoiceText";
        String text = null;
        int tableSize = choiceItems.size();

        if (tableSize > 0 && choice >= 0 && choice < tableSize)
        {
            text = choiceItems.get(choice).getText();
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "choice=%d", choice);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=%s", text != null? text: "null");
        }

        return text;
    }   //getChoiceText

    /**
     * This method returns the choice object of the given choice index.
     *
     * @param choice specifies the choice index in the menu.
     * @return object of the given choice if choice index is valid, null otherwise.
     */
    public T getChoiceObject(int choice)
    {
        final String funcName = "getChoiceObject";
        T obj = null;
        int tableSize = choiceItems.size();

        if (tableSize > 0 && choice >= 0 && choice < tableSize)
        {
            obj = choiceItems.get(choice).getObject();
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "choice=%d", choice);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=%s", obj != null? obj.toString(): "null");
        }

        return obj;
    }   //getChoiceObject

    /**
     * This method returns the index of the current choice. Every menu has a
     * current choice even if the menu hasn't been displayed and the user
     * hasn't picked a choice. In that case, the current choice is the
     * highlighted selection of the menu which is the first choice in the menu.
     * If the menu is empty, the current choice index is -1.
     *
     * @return current choice index, -1 if menu is empty.
     */
    public int getCurrentChoice()
    {
        final String funcName = "getCurrentChoice";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", currChoice);
        }

        return currChoice;
    }   //getCurrentChoice

    /**
     * This method returns the text of the current choice. Every menu has a
     * current choice even if the menu hasn't been displayed and the user
     * hasn't picked a choice. In that case, the current choice is the
     * highlighted selection of the menu which is the first choice in the menu.
     * If the menu is empty, the current choice index is -1.
     *
     * @return current choice text, null if menu is empty.
     */
    public String getCurrentChoiceText()
    {
        return getChoiceText(currChoice);
    }   //getCurrentChoiceText

    /**
     * This method returns the object of the current choice. Every menu has a
     * current choice even if the menu hasn't been displayed and the user
     * hasn't picked a choice. In that case, the current choice is the
     * highlighted selection of the menu which is the first choice in the menu.
     * If the menu is empty, the current choice index is -1.
     *
     * @return current choice object, null if menu is empty.
     */
    public T getCurrentChoiceObject()
    {
        return getChoiceObject(currChoice);
    }   //getCurrentChoiceObject

    //
    // Implements FtcMenu abstract methods.
    //

    /**
     * This method moves the current selection to the previous choice in the menu.
     * If it is already the first choice, it will wraparound back to the last choice.
     */
    public void menuUp()
    {
        final String funcName = "menuUp";

        if (choiceItems.size() == 0)
        {
            currChoice = -1;
        }
        else
        {
            currChoice--;
            if (currChoice < 0)
            {
                currChoice = choiceItems.size() - 1;
            }

            if (currChoice < firstDisplayedChoice)
            {
                //
                // Scroll up.
                //
                firstDisplayedChoice = currChoice;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "! (choice=%d)", currChoice);
        }
    }   //menuUp

    /**
     * This method moves the current selection to the next choice in the menu.
     * If it is already the last choice, it will wraparound back to the first choice.
     */
    public void menuDown()
    {
        final String funcName = "menuDown";

        if (choiceItems.size() == 0)
        {
            currChoice = -1;
        }
        else
        {
            currChoice++;
            if (currChoice >= choiceItems.size())
            {
                currChoice = 0;
            }

            int lastDisplayedChoice =
                    Math.min(firstDisplayedChoice + HalDashboard.MAX_NUM_TEXTLINES - 2,
                             choiceItems.size() - 1);
            if (currChoice > lastDisplayedChoice)
            {
                //
                // Scroll down.
                //
                firstDisplayedChoice = currChoice - (HalDashboard.MAX_NUM_TEXTLINES - 2);
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "! (choice=%d)", currChoice);
        }
    }   //menuDown

    /**
     * This method returns the child menu of the current choice.
     *
     * @return child menu of the current choice.
     */
    public FtcMenu getChildMenu()
    {
        final String funcName = "getChildMenu";
        FtcMenu childMenu = choiceItems.get(currChoice).childMenu;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=%s", childMenu != null? childMenu.getTitle(): "null");
        }

        return childMenu;
    }   //getChildMenu

    /**
     * This method displays the menu on the dashboard with the current
     * selection highlighted. The number of choices in the menu may
     * exceed the total number of lines on the dashboard. In that case,
     * it will only display all the choices that will fit on the
     * dashboard. If the user navigates to a choice outside of the
     * dashboard display, the choices will scroll up or down to bring
     * the new selection into the dashboard.
     */
    public void displayMenu()
    {
        final String funcName = "displayMenu";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        //
        // Determine the choice of the last display line on the dashboard.
        //
        int lastDisplayedChoice =
                Math.min(firstDisplayedChoice + HalDashboard.MAX_NUM_TEXTLINES - 2,
                         choiceItems.size() - 1);
        dashboard.clearDisplay();
        dashboard.displayPrintf(0, getTitle());
        //
        // Display all the choices that will fit on the dashboard.
        //
        for (int i = firstDisplayedChoice; i <= lastDisplayedChoice; i++)
        {
            ChoiceItem item = choiceItems.get(i);
            dashboard.displayPrintf(
                    i - firstDisplayedChoice + 1,
                    i == currChoice? ">>\t%s%s": "%s%s",
                    item.getText(), item.getChildMenu() != null? " ...": "");
        }
    }   //displayMenu

}   //class FtcChoiceMenu
