package org.firstinspires.ftc.teamcode.kurio.mecanumpursuit;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.kurio.Robot;
import org.firstinspires.ftc.teamcode.kurio.math.MathUtil;
import org.firstinspires.ftc.teamcode.kurio.math.Point;
import org.firstinspires.ftc.teamcode.kurio.math.Pose;
import org.firstinspires.ftc.teamcode.kurio.mecanumpursuit.waypoints.PointTurnWayPoint;
import org.firstinspires.ftc.teamcode.kurio.mecanumpursuit.waypoints.StopWayPoint;
import org.firstinspires.ftc.teamcode.kurio.mecanumpursuit.waypoints.WayPoint;

import java.util.LinkedList;
import java.util.List;

@Config
public class PurePursuitPath {
    public static double DEAD_MAN_SWITCH = 2500;
    private final Robot robot;
    public List<WayPoint> waypoints;

    // currPoint in 0..n-2 means we're on the path from waypoints[currPoint] to
    // waypoints[currPoint + 1]. currPoint = n-1 means we're done.
    public int currPoint;
    boolean interrupting;
    public ElapsedTime timeUntilDeadman;

    public PurePursuitPath(Robot robot, List<WayPoint> waypoints) {
        // We need to deep copy our linked list so the same point doesn't get flipped multiple times
        this.waypoints = new LinkedList<>();
        for (WayPoint w : waypoints) {
            this.waypoints.add(w.clone());
        }

        this.currPoint = 0;
        this.robot = robot;
        this.interrupting = false;
        this.timeUntilDeadman = new ElapsedTime();

        if (!(waypoints.get(waypoints.size() - 1) instanceof StopWayPoint)) {
            throw new IllegalArgumentException("Final Pure Pursuit waypoint must be a StopWaypoint!");
        }
    }

//    public void reverse() {
//        for (WayPoint w : this.waypoints) {
//            w.y = -w.y;
//
//            // We also need to invert headings.
//            if (w instanceof HeadingControlledWaypoint) {
//                HeadingControlledWaypoint hCW = (HeadingControlledWaypoint) w;
//                hCW.targetHeading = MathUtil.angleWrap(-hCW.targetHeading);
//            }
//        }
//    }

    public void update() {
        Pose robotPosition = robot.getPose();
        Pose robotVelocity = robot.getVelocity();

        // Check whether we should advance to the next piece of the curve
        boolean jumpToNextSegment;
        do {
            jumpToNextSegment = false;
            WayPoint target = waypoints.get(currPoint + 1);

            // Stop waypoint deadman switch
            if (target instanceof StopWayPoint && timeUntilDeadman.milliseconds() > DEAD_MAN_SWITCH) {
                jumpToNextSegment = true;
                currPoint++;
            } else if (!(target instanceof StopWayPoint) || robot.getVelocity().distToOrigin() > 2.0) {
                timeUntilDeadman.reset();
            }
            if (target instanceof StopWayPoint) {
                if (robotPosition.distanceTo(target) < ((StopWayPoint) target).allowedPositionError) {
                    jumpToNextSegment = true;
                    currPoint++;
                }
            } else if (target instanceof PointTurnWayPoint) {
                PointTurnWayPoint ptTarget = (PointTurnWayPoint) target;
                if (Math.abs(robotPosition.heading - ptTarget.targetHeading) < ptTarget.allowedHeadingError) {
                    jumpToNextSegment = true;
                    currPoint++;
                }
            } else {
                if (robotPosition.distanceTo(target) < target.followDistance) {
                    jumpToNextSegment = true;
                    currPoint++;
                }
            }

            Log.v("PP", "Index: " + currPoint);

        } while (jumpToNextSegment && currPoint < waypoints.size() - 1);
//        if (finished()) {return;}

        WayPoint target = waypoints.get(currPoint + 1);
        // If we're making a stop and in the stop portion of the move
        if (target instanceof StopWayPoint && robotPosition.distanceTo(target) < target.followDistance) {
            robot.setPowers(MecanumPurePursuitController.goToPosition(
                    robotPosition, robotVelocity, target, (StopWayPoint) target));
            Log.v("PP", "Locking onto point " + target);
        } else if (target instanceof PointTurnWayPoint) {
            robot.setPowers(MecanumPurePursuitController.goToPosition(robotPosition, robotVelocity, target, null));
        } else {
            trackToLine(robotPosition, robotVelocity, waypoints.get(currPoint), waypoints.get(currPoint + 1));
        }
    }

    /**
     * @param robotPosition a pose representing the current position and orientation of the robot
     * @param start the starting point of the line segment we're following. Can be any subclass of
     *             waypoint.
     * @param end the end point of the line segment we're following. Must be a normal waypoint or
     *            a heading controlled waypoint. Passing a normal waypoint will cause the robot to
     *            turn itself in the direction of travel, while passing a heading controlled
     *            waypoint will cause the robot's heading to lock to the desired direction.
     */
    private void trackToLine(Pose robotPosition, Pose robotVelocity, WayPoint start, WayPoint end) {
        Point intersection = MathUtil.lineSegmentCircleIntersection(
                start, end, robotPosition, end.followDistance
        );

        // If our line intersects at all
        // We clone the midpoint to preserve metadata, if it exists
        WayPoint target = end.clone();
        if (intersection == null) {
            target.x = end.x;
            target.y = end.y;
        } else {
            target.x = intersection.x;
            target.y = intersection.y;
        }
        robot.setPowers(MecanumPurePursuitController.goToPosition(robotPosition, robotVelocity,
                target, end instanceof StopWayPoint ? (StopWayPoint) end : null));
    }

    public Canvas draw(Canvas canvas) {
        double[] xPoints = new double[waypoints.size()];
        double[] yPoints = new double[waypoints.size()];

        for (int i = 0; i < waypoints.size(); i++) {
            xPoints[i] = waypoints.get(i).toFTCSystem().x;
            yPoints[i] = waypoints.get(i).toFTCSystem().y;
        }

        return canvas.setStroke("red").setStrokeWidth(1).strokePolyline(xPoints, yPoints);
    }

    public boolean finished() {
        return currPoint >= waypoints.size() - 1;
    }
}
