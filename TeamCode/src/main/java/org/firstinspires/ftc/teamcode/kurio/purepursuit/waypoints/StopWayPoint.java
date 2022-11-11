package org.firstinspires.ftc.teamcode.kurio.purepursuit.waypoints;

import androidx.annotation.NonNull;

public class StopWayPoint extends HeadingControlledWaypoint {
    public double allowedPositionError;

    public StopWayPoint(double x, double y, double followDistance, double targetHeading, double allowedPositionError) {
        super(x, y, followDistance, targetHeading);
        this.allowedPositionError = allowedPositionError;
    }

    @NonNull
    @Override
    public StopWayPoint clone() {
        return new StopWayPoint(x, y, followDistance, targetHeading, allowedPositionError);
    }
}
