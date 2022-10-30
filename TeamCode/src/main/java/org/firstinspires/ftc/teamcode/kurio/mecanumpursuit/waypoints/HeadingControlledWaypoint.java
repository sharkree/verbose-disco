package org.firstinspires.ftc.teamcode.kurio.mecanumpursuit.waypoints;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.kurio.math.Pose;

public class HeadingControlledWaypoint extends WayPoint {
    public double targetHeading;

    public HeadingControlledWaypoint(Pose pose, double followdistance) {
        this(pose.x, pose.y, followdistance, pose.heading);
    }

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
