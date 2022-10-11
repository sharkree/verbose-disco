package org.firstinspires.ftc.teamcode.kurio.mecanumpursuit.waypoints;

import androidx.annotation.NonNull;

public class HeadingControlledWaypoint extends WayPoint {
    public double targetHeading;
    public HeadingControlledWaypoint(double x, double y, double followDistance, double targetHeading) {
        super(x, y, followDistance);
        this.targetHeading = targetHeading;
    }

    @NonNull
    @Override
    public HeadingControlledWaypoint clone() {
        return new HeadingControlledWaypoint(x, y, followDistance, targetHeading);
    }
}
