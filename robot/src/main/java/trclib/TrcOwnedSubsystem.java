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
 * granted. This will prevent other callers from interfering an unfinished operation by a different caller. 
 */
public interface TrcOwnedSubsystem
{
    /**
     * This method acquires exclusive ownership of the subsystem.
     *
     * @param owner specifies the ID string of the caller requesting ownership.
     * @return true if successfully acquired ownership, false otherwise.
     */
    default boolean acquireExclusiveAccess(String owner)
    {
        return TrcOwnershipManager.getInstance().acquireOwnership(owner, this);
    }

    /**
     * This method checks if the caller has exclusive ownership of the subsystem. If not, it throws an exception.
     */
    default boolean validateOwnership(String owner)
    {
        return TrcOwnershipManager.getInstance().validateOwnership(owner, this);
    }   //validateOnwership

    /**
     * This method release exclusive ownership of the subsystem.
     *
     * @param owner specifies the ID string of the caller releasing ownership.
     * @return true if successfully releasing ownership, false otherwise.
     */
    default boolean releaseExclusiveAccess(String owner)
    {
        return TrcOwnershipManager.getInstance().releaseOwnership(owner, this);
    }

    /**
     * This method checks if the caller has exclusive ownership of the subsystem.
     *
     * @param owner specifies the ID string of the caller.
     * @return true if caller has exclusive ownership of the subsystem, false otherwise.
     */
    default boolean hasOwnership(String owner)
    {
        return TrcOwnershipManager.getInstance().hasOwnership(owner, this);
    }

}   //interface TrcOwnedSubsystem
