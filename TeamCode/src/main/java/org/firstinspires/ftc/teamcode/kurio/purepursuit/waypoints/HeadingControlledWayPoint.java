package org.firstinspires.ftc.teamcode.kurio.purepursuit.waypoints;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.kurio.math.Pose;

public class HeadingControlledWayPoint extends WayPoint {
    public double targetHeading;

    public HeadingControlledWayPoint(Pose pose, double followdistance) {
        this(pose.x, pose.y, followdistance, pose.heading);
    }

    public HeadingControlledWayPoint(double x, double y, double followDistance, double targetHeading) {
        super(x, y, followDistance);
        this.targetHeading = targetHeading;
    }

    @NonNull
    @Override
    public HeadingControlledWayPoint clone() {
        return new HeadingControlledWayPoint(x, y, followDistance, targetHeading);
    }
}
