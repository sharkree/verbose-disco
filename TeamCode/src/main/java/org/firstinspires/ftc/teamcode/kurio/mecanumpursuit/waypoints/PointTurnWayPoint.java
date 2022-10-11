package org.firstinspires.ftc.teamcode.kurio.mecanumpursuit.waypoints;

import androidx.annotation.NonNull;

public class PointTurnWayPoint extends HeadingControlledWaypoint {
    public double allowedHeadingError;

    public PointTurnWayPoint(double x, double y, double followDistance, double targetHeading, double allowedHeadingError) {
        super(x, y, followDistance, targetHeading);
        this.allowedHeadingError = allowedHeadingError;
    }

    @NonNull
    @Override
    public PointTurnWayPoint clone() {
        return new PointTurnWayPoint(x, y, followDistance, targetHeading, allowedHeadingError);
    }
}
