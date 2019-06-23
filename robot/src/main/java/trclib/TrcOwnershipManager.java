package trclib;

import java.util.HashMap;

public class TrcOwnershipManager
{
    private static final TrcOwnershipManager instance = new TrcOwnershipManager();

    public static TrcOwnershipManager getInstance()
    {
        return instance;
    }

    private HashMap<TrcOwnedSubsystem,String> ownershipMap;

    public TrcOwnershipManager()
    {
        ownershipMap = new HashMap<>();
    }

    /**
     * This method returns the current owner of the subsystem.
     *
     * @return owner ID string.
     */
    public synchronized String getOwner(TrcOwnedSubsystem owned)
    {
        return ownershipMap.get(owned);
    }   //getOwner

    /**
     * This method checks if the caller has exclusive ownership of the subsystem.
     *
     * @param owner specifies the ID string of the caller, can be null if caller is unaware of exclusive ownership.
     * @return true if caller has exclusive ownership of the subsystem, false otherwise.
     */
    public synchronized boolean hasOwnership(String owner, TrcOwnedSubsystem owned)
    {
        String currOwner = getOwner(owned);
        return owner == null && currOwner == null || currOwner != null && currOwner.equals(owner);
    }   //hasOwnership

    /**
     * This method checks if the caller has exclusive ownership of the subsystem. If not, it throws an exception.
     */
    public synchronized boolean validateOwnership(String owner, TrcOwnedSubsystem owned)
    {
        boolean success = hasOwnership(owner, owned);

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
    public synchronized boolean acquireOwnership(String owner, TrcOwnedSubsystem owned)
    {
        boolean success = false;

        if (!ownershipMap.containsKey(owned))
        {
            ownershipMap.put(owned, owner);
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
    public synchronized boolean releaseOwnership(String owner, TrcOwnedSubsystem owned)
    {
        boolean success = false;

        if (hasOwnership(owner, owned))
        {
            owner = null;
            success = true;
        }

        return success;
    }   //releaseOwnership
}
