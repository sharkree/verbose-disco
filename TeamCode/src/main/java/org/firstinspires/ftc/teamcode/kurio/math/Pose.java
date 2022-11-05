package org.firstinspires.ftc.teamcode.kurio.math;

import static org.firstinspires.ftc.teamcode.kurio.math.MathUtil.angleWrap;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.Locale;

public class Pose extends Point implements Cloneable {
    public double heading;

    public static Pose ZERO = new Pose(0, 0, 0);

    private static final float FULL_FIELD = 144.0f;

    public Pose(double x, double y, double heading) {
        super(x, y);
        this.heading = heading;
    }

    public Pose(Point p, double heading) {
        this(p.x, p.y, heading);
    }

    public Pose add(Pose p2) {
        return new Pose(x + p2.x, y + p2.y, heading + p2.heading);
    }

    public Pose multiplyEachComp(Pose p2) {
        return new Pose(x * p2.x, y * p2.y, heading * p2.heading);
    }

    public Pose divideEachComp(Pose p2) {
        return new Pose(x / p2.x, y / p2.y, heading / p2.heading);
    }
    public Pose minus(Pose p2) {
        return new Pose(x - p2.x, y - p2.y, heading - p2.heading);
    }

    public Pose scale(double d) {return new Pose(x * d, y * d, heading * d);}

    public Pose negate() {
        return this.scale(-1.0);
    }

    public void applyFriction(Pose friction) {
        x = reduceUpToZero(x, friction.x);
        y = reduceUpToZero(y, friction.y);
        heading = reduceUpToZero(heading, friction.heading);
    }

    private double reduceUpToZero(double d, double reduction) {
        return d - minAbs(d, Math.copySign(reduction, d));
    }

    private double minAbs(double a, double b) {
        return Math.abs(a) < Math.abs(b) ? a : b;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        if (!super.equals(o)) return false;
        Pose pose = (Pose) o;
        return MathUtil.approxEquals(pose.heading, heading);
    }

    public Vector2d getHeadingVector() {
        return new Vector2d(cos(heading), sin(heading));
    }

    public Pose toFTCSystem() {
        double x = -this.y + (FULL_FIELD / 2.);
        double y = this.x - (FULL_FIELD / 2.);
        double heading = angleWrap(Math.PI - this.heading);
        return new Pose(x, y, heading);
    }

    @Override
    public String toString() {
        return String.format(Locale.US, "{x: %.3f, y: %.3f, θ: %.3f}", x, y, heading);
    }

    @Override
    public Pose clone() {
        return new Pose(x, y, heading);
    }
}
