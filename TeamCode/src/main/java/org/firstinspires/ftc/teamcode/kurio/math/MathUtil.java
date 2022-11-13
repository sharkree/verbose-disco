package org.firstinspires.ftc.teamcode.kurio.math;

public class MathUtil {
    public static final double EPSILON = 1e-6;

    public static double angleWrap(double angle) {
        double tmp = angle % (Math.PI * 2);

        if (Math.abs(tmp) > Math.PI) tmp -= Math.copySign(Math.PI * 2, tmp);

        return tmp;
    }
    public static double clip(double d) {
        return Math.min(Math.max(d, -1), 1);
    }

    public static double clampAbove(double d, double threshold) {
        return Math.abs(d) < threshold ? Math.copySign(threshold, d) : d;
    }

    public static double clampBelow(double d, double threshold) {
        return Math.abs(d) < threshold ? 0 : d;
    }

    public static double powKeepSign(double d, double power) {
        // In case d is super small, just make it zero
        if (Math.abs(d) < 1e-12) {
            return 0;
        }
        return Math.copySign(Math.pow(Math.abs(d), power), d);
    }

    public static double zeroify(double d, double thresh) {
        return (Math.abs(d) < thresh) ? 0 : d;
    }

    public static Pose relativeOdometryUpdate(Pose fieldPose, Pose robotPoseDelta) {
        double dtheta = robotPoseDelta.heading;
        double sineTerm, cosTerm;

        if (approxEquals(dtheta, 0)) {
            sineTerm = 1.0 - dtheta * dtheta / 6.0;
            cosTerm = dtheta / 2.0;
        } else {
            sineTerm = Math.sin(dtheta) / dtheta;
            cosTerm = (1 - Math.cos(dtheta)) / dtheta;
        }

        Point fieldPositionDelta = new Point(
                sineTerm * robotPoseDelta.x - cosTerm * robotPoseDelta.y,
                cosTerm * robotPoseDelta.x + sineTerm * robotPoseDelta.y
        );

        Pose fieldPoseDelta = new Pose(fieldPositionDelta.rotated(fieldPose.heading), robotPoseDelta.heading);

        return fieldPose.add(fieldPoseDelta);
    }

    public static boolean approxEquals(double d1, double d2) {
        if (Double.isInfinite(d1)) {
            // Infinity - infinity is NaN, so we need a special case
            return d1 == d2;
        } else {
            return Math.abs(d1 - d2) < EPSILON;
        }
    }

    public static Point lineSegmentCircleIntersection(Point pointA, Point pointB, Point center, double radius) {
        double baX = pointB.x - pointA.x;
        double baY = pointB.y - pointA.y;
        double caX = center.x - pointA.x;
        double caY = center.y - pointA.y;

        double a = baX * baX + baY * baY;
        double bBy2 = baX * caX + baY * caY;
        double c = caX * caX + caY * caY - radius * radius;

        double pBy2 = bBy2 / a;
        double q = c / a;

        double disc = pBy2 * pBy2 - q;
        if (disc < 0) return null;
        // if disc == 0 ... dealt with later
        double tmpSqrt = Math.sqrt(disc);
        double abScalingFactor1 = -pBy2 + tmpSqrt;
        double abScalingFactor2 = -pBy2 - tmpSqrt;

        Point p1 = new Point(pointA.x - baX * abScalingFactor1, pointA.y
                - baY * abScalingFactor1);
        if (disc == 0) return p1;
        Point p2 = new Point(pointA.x - baX * abScalingFactor2, pointA.y
                - baY * abScalingFactor2);
        return p1.distanceTo(pointA) < p2.distanceTo(pointA) ? p1 : p2;
    }

    private static int sign(double n) {
        return n < 0 ? -1 : 1;
    }

    public static boolean between(double r1, double r2, double val, double threshold) {
        return val > (Math.min(r1, r2) - threshold) && val < (Math.max(r1, r2) + threshold);
    }
}

