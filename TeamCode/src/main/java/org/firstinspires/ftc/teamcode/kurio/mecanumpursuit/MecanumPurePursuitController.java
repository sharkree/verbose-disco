package org.firstinspires.ftc.teamcode.kurio.mecanumpursuit;

import android.util.Log;

import org.firstinspires.ftc.teamcode.kurio.math.MathUtil;
import org.firstinspires.ftc.teamcode.kurio.math.Point;
import org.firstinspires.ftc.teamcode.kurio.math.Pose;
import org.firstinspires.ftc.teamcode.kurio.mecanumpursuit.waypoints.HeadingControlledWaypoint;
import org.firstinspires.ftc.teamcode.kurio.mecanumpursuit.waypoints.StopWayPoint;
import org.firstinspires.ftc.teamcode.kurio.mecanumpursuit.waypoints.WayPoint;

public class MecanumPurePursuitController {
    // How far we slip if we're moving 1 in/sec (or 1 rad/sec) in each of these directions
    public static Pose SLIP_DISTANCES = new Pose(1.5, 0, 0);
    public static double UNDERSHOOT_DIST = 6.0; // Aim to stop 2 in away from target, and use small motions to finish it
    public static double MIN_SLIP_SPEED = 10.0;
    public static Pose GUNNING_REDUCTION_DISTANCES = new Pose(12, 12, Math.PI);
    public static Pose ONE_AWAY_POWERS = new Pose(0.10, 0.15, 0.125);
    public static double EXPONENT = 1.0 / 8.0;

    public static Pose rDistanceToTarget(Pose robot, Point target) {
        double distance = target.minus(robot).distToOrigin();
        double relAngle = robot.minus(target).atan() - robot.heading;
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

        if (finalTarget == null || robotPose.distanceTo(finalTarget) > 18.0) { // 12sqrt(2) is 16.97, but we'll round up. It will be scaled anyway at the end :)
            Pose distance = rDistanceToTarget(robotPose, target);

            // We negate x and y power because we want to move in the opposite direction of our error
            Pose translationPowers = distance.negate().divideEachComp(GUNNING_REDUCTION_DISTANCES);

            // Heading always wants to stop at a point, so we'll treat this the same regardless if we're
            // at a stop waypoint or a normal one. We want to rotate as less as possible to reach the desired heading.
            double forwardAngle = target.minus(robotPose).atan();
            double backwardAngle = forwardAngle + Math.PI;
            double angleToForward = MathUtil.angleWrap(forwardAngle - robotPose.heading);
            double angleToBackward = MathUtil.angleWrap(backwardAngle - robotPose.heading);
            double autoAngle = Math.abs(angleToForward) < Math.abs(angleToBackward) ? forwardAngle : backwardAngle;
            double desiredAngle = target instanceof HeadingControlledWaypoint ?
                    ((HeadingControlledWaypoint) target).targetHeading : autoAngle;

            double angleToTarget = MathUtil.angleWrap(desiredAngle - robotPose.heading);
            translationPowers.heading = -angleToTarget / GUNNING_REDUCTION_DISTANCES.heading;
            return new MecanumPowers(translationPowers);
        } else if (robotVelocity.distToOrigin() > MIN_SLIP_SPEED && robotPose.distanceTo(finalTarget) > UNDERSHOOT_DIST) { // If we're moving more than 6 in/sec and we're close to our target
            // We don't want to aim quite for our target - we want to undershoot a fair bit
            // We won't use this very often - only when we need to line up somewhere exactly. Everywhere else,
            // we'll just allow a lot of error in our stopwaypoint
            Point t = MathUtil.lineSegmentCircleIntersection(robotPose, finalTarget, finalTarget, UNDERSHOOT_DIST);

            // We're approaching a point, and we need to not overshoot
            Pose relVelocity = new Pose(robotVelocity.rotated(-robotPose.heading), robotVelocity.heading);
            Pose relSlipDistances = relVelocity.multiplyEachComp(SLIP_DISTANCES);
            Pose relAbsTarget = rDistanceToTarget(robotPose, t).add(relSlipDistances);
            // We negate this here so our negation in translationPowers is cancelled out
            relAbsTarget.heading = MathUtil.angleWrap(finalTarget.targetHeading - robotPose.heading - relSlipDistances.heading);

            Pose translationPowers = relAbsTarget.negate().divideEachComp(GUNNING_REDUCTION_DISTANCES);
            return new MecanumPowers(translationPowers);
        } else {
            // Now we just need to nudge the robot. We'll hold our heading with a simple P-loop,
            // and adjust our position with a special function
            Pose relAbsTarget = rDistanceToTarget(robotPose, finalTarget);
            double angleToTarget = MathUtil.angleWrap(finalTarget.targetHeading - robotPose.heading);

            // Now, we're going to use the polynomial function x^1/8 to compute our powers

            Pose dirPowers = new Pose(
                    MathUtil.powKeepSign(relAbsTarget.x, EXPONENT),
                    MathUtil.powKeepSign(relAbsTarget.y, EXPONENT),
                    MathUtil.powKeepSign(angleToTarget, EXPONENT)
            ).negate().multiplyEachComp(ONE_AWAY_POWERS);
            Log.v("PP", dirPowers.toString());
            if (robotPose.distToOrigin() < finalTarget.allowedPositionError) return MecanumPowers.REST; // might need to divide by 2
            else return new MecanumPowers(dirPowers);
        }
    }
}
