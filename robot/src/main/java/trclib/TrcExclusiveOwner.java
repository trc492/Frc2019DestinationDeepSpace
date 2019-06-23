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
 * This class implements methods to support subsystem exclusive ownership. The subsystem should instantiate this
 * class and implements the TrcExclusiveAccess interface by calling methods in this class.
 */
public class TrcExclusiveOwner
{
    private String owner = null;

    /**
     * This method returns the current owner of the subsystem.
     *
     * @return owner ID string.
     */
    public synchronized String getOwner()
    {
        return owner;
    }   //getOwner

    /**
     * This method checks if the caller has exclusive ownership of the subsystem.
     *
     * @param owner specifies the ID string of the caller, can be null if caller is unaware of exclusive ownership.
     * @return true if caller has exclusive ownership of the subsystem, false otherwise.
     */
    public synchronized boolean hasOwnership(String owner)
    {
        return owner == null && this.owner == null || owner != null && this.owner != null && this.owner.equals(owner);
    }   //hasOwnership

    /**
     * This method checks if the caller has exclusive ownership of the subsystem. If not, it throws an exception.
     */
    public synchronized boolean validateOwnership(String owner)
    {
        boolean success = hasOwnership(owner);

        if (!success && owner != null)
        {
            throw new IllegalStateException(
                String.format("%s does not have exclusive ownership of the drive base.", owner));
        }

        return success;
    }   //validateOnwership

    /**
     * This method acquires exclusive ownership of the subsystem.
     *
     * @param owner specifies the ID string of the caller requesting ownership.
     * @return true if successfully acquired ownership, false otherwise.
     */
    public synchronized boolean acquireOwnership(String owner)
    {
        boolean success = false;

        if (this.owner == null)
        {
            this.owner = owner;
            success = true;
        }

        return success;
    }   //acquireOwnership

    /**
     * This method release exclusive ownership of the subsystem.
     *
     * @param owner specifies the ID string of the caller releasing ownership.
     * @return true if successfully releasing ownership, false otherwise.
     */
    public synchronized boolean releaseOwnership(String owner)
    {
        boolean success = false;

        if (hasOwnership(owner))
        {
            owner = null;
            success = true;
        }

        return success;
    }   //releaseOwnership

}   //class TrcExclusiveOwner
