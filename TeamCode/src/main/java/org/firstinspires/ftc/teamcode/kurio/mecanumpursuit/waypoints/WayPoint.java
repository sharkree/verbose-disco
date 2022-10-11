package org.firstinspires.ftc.teamcode.kurio.mecanumpursuit.waypoints;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.kurio.math.Point;

import java.util.Arrays;
import java.util.LinkedList;

public class WayPoint extends Point implements Cloneable {
    public double followDistance;

    public WayPoint(Point p, double followDistance) {
        this(p.x, p.y, followDistance);
    }

    public WayPoint(double x, double y, double followDistance) {
        super(x, y);
        this.followDistance = followDistance;
    }

    @NonNull
    @Override
    public WayPoint clone() {
        return new WayPoint(x, y, followDistance);
    }

    public static LinkedList<WayPoint> collate(WayPoint... waypoints) {
        return new LinkedList<>(Arrays.asList(waypoints));
    }
}