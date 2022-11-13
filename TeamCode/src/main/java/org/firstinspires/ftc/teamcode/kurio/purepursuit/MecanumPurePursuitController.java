package org.firstinspires.ftc.teamcode.kurio.purepursuit;

import static org.firstinspires.ftc.teamcode.kurio.math.MathUtil.angleWrap;
import static org.firstinspires.ftc.teamcode.kurio.math.MathUtil.approxEquals;

import android.util.Log;

import org.firstinspires.ftc.teamcode.kurio.math.MathUtil;
import org.firstinspires.ftc.teamcode.kurio.math.Point;
import org.firstinspires.ftc.teamcode.kurio.math.Pose;
import org.firstinspires.ftc.teamcode.kurio.purepursuit.waypoints.HeadingControlledWaypoint;
import org.firstinspires.ftc.teamcode.kurio.purepursuit.waypoints.StopWayPoint;
import org.firstinspires.ftc.teamcode.kurio.purepursuit.waypoints.WayPoint;

public class MecanumPurePursuitController {
    // How far we slip if we're moving 1 in/sec (or 1 rad/sec) in each of these directions
    public static double UNDERSHOOT_DIST = 6.0; // Aim to stop 2 in away from target, and use small motions to finish it
    public static Pose GUNNING_REDUCTION_DISTANCES = new Pose(18, 18, 18);
    public static Pose ONE_AWAY_POWERS = new Pose(0.004, 0.006, 0.005);
    public static double EXPONENT = 1.0 / 6.0;

    public static Pose rDistanceToTarget(Pose robot, Point target) {
        double distance = target.minus(robot).distToOrigin();
        double relAngle = angleWrap(target.minus(robot).atan() - robot.heading);
        double relX = distance * Math.cos(relAngle);
        double relY = distance * Math.sin(relAngle);

        return new Pose(relX, relY, relAngle);
    }

    public static Pose rDistanceToTarget(Pose robot, HeadingControlledWaypoint target) {
        double distance = target.minus(robot).distToOrigin();
        double relAngle = angleWrap(target.targetHeading - robot.heading);
        double relX = distance * Math.cos(relAngle);
        double relY = distance * Math.sin(relAngle);

        return new Pose(relX, relY, relAngle);
    }

    public static MecanumPowers goToPosition(Pose robotPose, Pose robotVelocity, WayPoint target, StopWayPoint finalTarget) {
        // Sometimes we need to move accurately to a position, while many other times we just have
        // to get "about" somewhere - our waypoints are approximations anyway. We'll assume we only
        // need to be exact if we're stopping somewhere - otherwise we can increase accuracy by just
        // using a shorter look ahead distance. If finalTarget is set, we'll try to finely adjust
        // speed and slippage to hit that point. Otherwise, we'll just YEET over there.

        Log.v("PP", "COPE BUCKET data:");
        Log.v("PP", "Current Pose: " + robotPose.toString());
        Log.v("PP", "Current velocity: " + robotVelocity.toString());
        Log.v("PP", "Current Target: " + target.toString());
        if (finalTarget != null) {
            Log.v("PP", "Final Target: " + finalTarget.toString());
        } else {
            Log.v("PP", "Final Target: null");
        }

        if (finalTarget == null || robotPose.distanceTo(finalTarget) > 18.0) {
            if (target instanceof HeadingControlledWaypoint) {
                Log.v("Mode", "1a");
                Pose distance = rDistanceToTarget(robotPose, (HeadingControlledWaypoint) target);

                // We want to rotate as less as possible to reach the desired heading.
                double forwardAngle = distance.heading;
                double backwardAngle = angleWrap(forwardAngle + Math.PI);
                double angleToForward = angleWrap(forwardAngle);
                double angleToBackward = angleWrap(backwardAngle);

                distance.heading = Math.abs(angleToForward) < Math.abs(angleToBackward) ? forwardAngle : backwardAngle;
                distance = distance.divideEachComp(GUNNING_REDUCTION_DISTANCES);
                Log.v("Translational Powers", distance.toString());
                return new MecanumPowers(distance);
            } else {
                Log.v("Mode", "1b");
                Pose distance = rDistanceToTarget(robotPose, target);
                // We want to rotate as less as possible to reach the desired heading.
                double forwardAngle = distance.heading;
                double backwardAngle = angleWrap(forwardAngle + Math.PI);
                double angleToForward = angleWrap(forwardAngle);
                double angleToBackward = angleWrap(backwardAngle);

                distance.heading = Math.abs(angleToForward) < Math.abs(angleToBackward) ? forwardAngle : backwardAngle;
                distance = distance.divideEachComp(GUNNING_REDUCTION_DISTANCES);
                Log.v("Translational Powers", distance.toString());
                return new MecanumPowers(distance);
            }
        } else if (robotPose.distanceTo(finalTarget) > UNDERSHOOT_DIST) {
            Log.v("Mode", "2");
            // We don't want to aim quite for our target, we want to undershoot a fair bit
            Point t = MathUtil.lineSegmentCircleIntersection(robotPose, finalTarget, finalTarget, UNDERSHOOT_DIST);
            Pose relAbsTarget = rDistanceToTarget(robotPose, t);
            Log.v("Intersection", t.toString());
            Log.v("RelAbsTarget", relAbsTarget.toString());

//            relAbsTarget.heading = MathUtil.angleWrap(finalTarget.targetHeading - robotPose.heading);

            Pose translationPowers = relAbsTarget.divideEachComp(GUNNING_REDUCTION_DISTANCES);
            Log.v("Translational Powers", translationPowers.toString());
            return new MecanumPowers(translationPowers);
        } else {
            Log.v("Mode", "3");
            // Now we just need to nudge the robot. We'll hold our heading with a simple P-loop,
            // and adjust our position with a special function
            Pose relAbsTarget = rDistanceToTarget(robotPose, finalTarget);

            // Now, we're going to use the polynomial function x^1/6 to compute our powers

            Pose dirPowers = new Pose(
                    MathUtil.powKeepSign(relAbsTarget.x, EXPONENT),
                    MathUtil.powKeepSign(relAbsTarget.y, EXPONENT),
                    -MathUtil.powKeepSign(relAbsTarget.heading, EXPONENT)
            );
            Log.v("Translational Powers", dirPowers.toString());
//            if (robotPose.distanceTo(finalTarget) < finalTarget.allowedPositionError && approxEquals(relAbsTarget.heading, 0)) return MecanumPowers.REST;
            return new MecanumPowers(dirPowers.multiplyEachComp(ONE_AWAY_POWERS));
        }
    }
}
