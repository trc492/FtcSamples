package trclib;

public abstract class TrcTankMotionProfileFollower
{
    protected final String instanceName;

    public TrcTankMotionProfileFollower(String instanceName)
    {
        this.instanceName = instanceName;
    }

    /**
     * Get the instance name of this object
     *
     * @return Instance name
     */
    public String getInstanceName()
    {
        return instanceName;
    }

    /**
     * Start following the supplied motion profile.
     *
     * @param profile TrcTankMotionProfile object representing the path to follow. Remember to match units!
     */
    public void start(TrcTankMotionProfile profile)
    {
        start(profile, null, 0.0);
    }

    /**
     * Start following the supplied motion profile.
     *
     * @param profile TrcTankMotionProfile object representing the path to follow. Remember to match units!
     * @param event   Event to signal when path has been followed
     */
    public void start(TrcTankMotionProfile profile, TrcEvent event)
    {
        start(profile, event, 0.0);
    }

    /**
     * Start following the supplied motion profile.
     *
     * @param profile TrcTankMotionProfile object representing the path to follow. Remember to match units!
     * @param event   Event to signal when path has been followed
     * @param timeout Maximum number of seconds to spend following the path. 0.0 means no timeout.
     */
    public abstract void start(TrcTankMotionProfile profile, TrcEvent event, double timeout);

    /**
     * The motion profile currently being followed by the follower.
     *
     * @return The profile object currently being followed. null if not following any profile.
     */
    public abstract TrcTankMotionProfile getActiveProfile();

    /**
     * Is path currently being followed?
     *
     * @return True if yes, false otherwise
     */
    public abstract boolean isActive();

    /**
     * Has this task been cancelled?
     *
     * @return True if someone has called the cancel() method while it was running, false otherwise
     */
    public abstract boolean isCancelled();

    /**
     * Stop following the path and cancel the event.
     */
    public abstract void cancel();
}
