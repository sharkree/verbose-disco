package org.firstinspires.ftc.teamcode.kurio.purepursuit.actions;

public class DelayedAction {
    public long systemActionTime;
    public Actions.DoOnceAction action;
    public String tag;

    public DelayedAction(long timeFromNow, Actions.DoOnceAction action) {
        this(timeFromNow, action, System.currentTimeMillis(), null);
    }

    public DelayedAction(long timeFromNow, Actions.DoOnceAction action, String tag) {
        this(timeFromNow, action, System.currentTimeMillis(), tag);
    }

    public DelayedAction(long timeFromNow, Actions.DoOnceAction action, long currentTime) {
        this(timeFromNow, action, currentTime, null);
    }

    public DelayedAction(long timeFromNow, Actions.DoOnceAction action, long currentTime, String tag) {
        this.systemActionTime = timeFromNow + currentTime;
        this.action = action;
        this.tag = tag;
    }
}