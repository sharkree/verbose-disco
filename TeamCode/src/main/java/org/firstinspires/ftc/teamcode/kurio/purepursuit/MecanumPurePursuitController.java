package org.firstinspires.ftc.teamcode.kurio.purepursuit;

import android.util.Log;

import org.firstinspires.ftc.teamcode.kurio.math.MathUtil;
import org.firstinspires.ftc.teamcode.kurio.math.Point;
import org.firstinspires.ftc.teamcode.kurio.math.Pose;
import org.firstinspires.ftc.teamcode.kurio.purepursuit.waypoints.HeadingControlledWayPoint;
import org.firstinspires.ftc.teamcode.kurio.purepursuit.waypoints.StopWayPoint;
import org.firstinspires.ftc.teamcode.kurio.purepursuit.waypoints.WayPoint;

public class MecanumPurePursuitController {
    public static Pose SLIP_DISTANCES = new Pose(0.25, 0, 0);
    public static double UNDERSHOOT_DIST = 6; // Aim to stop 2 in away from target, and use small motions to finish it
    public static double MIN_SLIP_SPEED = 9;
    public static Pose GUNNING_REDUCTION_DISTANCES = new Pose(24, 24, 24);
    public static Pose ONE_AWAY_POWERS = new Pose(0.10, 0.15, 0.125);
    public static double CLOSE_EXPONENT = 1.0/6.0;

    public static Pose rDistanceToTarget(Pose robot, Point target) {
        double distance = target.minus(robot).distToOrigin();
        double relAngle = robot.minus(target).atan() - robot.heading;
        double relX = distance * Math.cos(Math.PI / 2 - relAngle);
        double relY = distance * Math.sin(Math.PI / 2 - relAngle);
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
            Pose relPPTarget = rDistanceToTarget(robotPose, target);

            // We negate x and y power because we want to move in the opposite direction of our error
            Pose translationPowers = relPPTarget.scale(-1).divideEachComp(GUNNING_REDUCTION_DISTANCES);
            Log.v("PP", translationPowers.toString());

            // Heading always wants to stop at a point, so we'll treat this the same regardless if we're
            // at a stop waypoint or a normal one
            double forwardAngle = target.minus(robotPose).atan();
            double backwardAngle = forwardAngle + Math.PI;
            double angleToForward = MathUtil.angleWrap(forwardAngle - robotPose.heading);
            double angleToBackward = MathUtil.angleWrap(backwardAngle - robotPose.heading);
            double autoAngle = Math.abs(angleToForward) < Math.abs(angleToBackward) ? forwardAngle : backwardAngle;
            double desiredAngle = target instanceof HeadingControlledWayPoint ?
                    ((HeadingControlledWayPoint) target).targetHeading : autoAngle;

            double angleToTarget = MathUtil.angleWrap(desiredAngle - robotPose.heading);
            translationPowers.heading = angleToTarget / GUNNING_REDUCTION_DISTANCES.heading;
            return new MecanumPowers(translationPowers);
        } else if (robotVelocity.distToOrigin() > MIN_SLIP_SPEED && robotPose.distanceTo(finalTarget) > UNDERSHOOT_DIST) {
            Log.v("Mode", "2");
            // We don't want to aim quite for our target, we want to undershoot a fair bit
            Point t = MathUtil.lineSegmentCircleIntersectionFar(robotPose, finalTarget, finalTarget, UNDERSHOOT_DIST);

            // We're approaching a point, and we need to not overshoot
            Pose relVelocity = new Pose(robotVelocity.rotated(-robotPose.heading), robotVelocity.heading);
            Log.v("PP", relVelocity.toString());
            Pose relSlipDistances = relVelocity.multiplyEachComp(SLIP_DISTANCES);
            Log.v("PP", relSlipDistances.toString());
            Pose relAbsTarget = rDistanceToTarget(robotPose, t).add(relSlipDistances);
            Log.v("PP", relAbsTarget.toString());
            // We negate this here so our negation in translationPowers is cancelled out
            relAbsTarget.heading = -MathUtil.angleWrap(finalTarget.targetHeading - robotPose.heading - relSlipDistances.heading);

            Pose translationPowers = relAbsTarget.scale(-1).divideEachComp(GUNNING_REDUCTION_DISTANCES);
            return new MecanumPowers(translationPowers);
        } else {
            Log.v("Mode", "3");
            // Now we just need to nudge the robot. We'll hold our heading with a simple P-loop,
            // and adjust our position with a special function
            Pose relAbsTarget = rDistanceToTarget(robotPose, finalTarget);
            double angleToTarget = MathUtil.angleWrap(finalTarget.targetHeading - robotPose.heading);

            // Now, we're going to use the polynomial function x^1/6 to compute our powers
            Pose dirPowers = new Pose(
                    -MathUtil.powKeepSign(relAbsTarget.x, CLOSE_EXPONENT),
                    -MathUtil.powKeepSign(relAbsTarget.y, CLOSE_EXPONENT),
                    MathUtil.powKeepSign(angleToTarget, CLOSE_EXPONENT)
            );
            return new MecanumPowers(dirPowers.multiplyEachComp(ONE_AWAY_POWERS));
        }
    }
}
