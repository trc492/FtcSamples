/*
 * Copyright (c) 2019 Titan Robotics Club (http://www.titanrobotics.com)
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
 * This interface defines methods for the subsystems to implement exclusive ownership support. A subsystem can be
 * accessed by multiple callers unaware of each other. Exclusive ownership can be acquired before access will be
 * granted. This will prevent other callers from interfering an in-progress operation by a different caller.
 */
public interface TrcExclusiveSubsystem
{
    /**
     * This method checks if the caller has exclusive ownership of the subsystem. For backward compatibility with
     * older code that's not aware of subsystem exclusive ownership, the owner parameter can be null. If the
     * subsystem has no owner currently and the caller is not aware of exclusive ownership, the caller is considered
     * to have acquired ownership. This means the caller is allowed to proceed with controlling the subsystem but if
     * a new caller comes in and acquires the ownership of the subsystem while an operation is in progress, the
     * operation will be interrupted and preempted by the new caller's operation. Therefore, callers unaware of
     * exclusive ownership can start an operation on the subsystem but their operations are not guaranteed exclusive
     * ownership.
     *
     * @param owner specifies the ID string of the caller, can be null if caller is unaware of exclusive ownership.
     * @return true if caller has exclusive ownership of the subsystem, false otherwise.
     */
    default boolean hasOwnership(String owner)
    {
        return TrcOwnershipManager.getInstance().hasOwnership(owner, this);
    }   //hasOwnership

    /**
     * This method checks if the caller has exclusive ownership of the subsystem. If not, it throws an exception.
     * It throws an exception only if the caller is aware of exclusive ownership and the it doesn't currently own
     * the subsystem. If the caller is unaware of exclusive ownership and the subsystem is owned by somebody else,
     * it will just return false and not throw an exception. This is to ensure older code that's unaware of exclusive
     * ownership will not hit an unexpected exception and will just fail quietly.
     *
     * @param owner specifies the ID string of the caller, can be null if caller is unaware of exclusive ownership.
     * @return true if the caller currently owns the subsystem, false otherwise.
     * @throws IllegalStateException if caller is not the owner of the subsystem.
     */
    default boolean validateOwnership(String owner)
    {
        return TrcOwnershipManager.getInstance().validateOwnership(owner, this);
    }   //validateOnwership

    /**
     * This method acquires exclusive ownership of the subsystem if it's not already owned by somebody else.
     *
     * @param owner specifies the ID string of the caller requesting ownership.
     * @return true if successfully acquired ownership, false otherwise.
     */
    default boolean acquireExclusiveAccess(String owner)
    {
        return TrcOwnershipManager.getInstance().acquireOwnership(owner, this);
    }   //acquireExclusiveAccess

    /**
     * This method release exclusive ownership of the subsystem if the caller is indeed the owner.
     *
     * @param owner specifies the ID string of the caller releasing ownership.
     * @return true if successfully releasing ownership, false otherwise.
     */
    default boolean releaseExclusiveAccess(String owner)
    {
        return TrcOwnershipManager.getInstance().releaseOwnership(owner, this);
    }   //releaseExclusiveAccess

}   //interface TrcExclusiveSubsystem
