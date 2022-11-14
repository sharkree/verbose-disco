package org.firstinspires.ftc.teamcode.kurio.purepursuit;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.kurio.Robot;
import org.firstinspires.ftc.teamcode.kurio.math.Line;
import org.firstinspires.ftc.teamcode.kurio.math.MathUtil;
import org.firstinspires.ftc.teamcode.kurio.math.Point;
import org.firstinspires.ftc.teamcode.kurio.math.Pose;
import org.firstinspires.ftc.teamcode.kurio.purepursuit.waypoints.StopWayPoint;
import org.firstinspires.ftc.teamcode.kurio.purepursuit.waypoints.WayPoint;

import java.util.LinkedList;
import java.util.List;

@Config
public class PurePursuitPath {
    public static double MIN_VELOCITY = 1.0;
    public static double STOP_SWITCH = 1000;
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

    public void update() {
        Pose robotPosition = robot.getPose().clone();
        Pose robotVelocity = robot.getVelocity().clone();

        // Check whether we should advance to the next piece of the curve
        boolean jumpToNextSegment;
        do {
            jumpToNextSegment = false;
            WayPoint target = waypoints.get(currPoint + 1);

            if (target instanceof StopWayPoint && timeUntilDeadman.milliseconds() > STOP_SWITCH) {
                jumpToNextSegment = true;
            } else if (!(target instanceof StopWayPoint) || robot.getVelocity().distToOrigin() > MIN_VELOCITY) {
                timeUntilDeadman.reset();
            }

            if (target instanceof StopWayPoint) {
                if (robotPosition.distanceTo(target) < ((StopWayPoint) target).allowedPositionError) {
                    jumpToNextSegment = true;
                }
            } else {
                if (robotPosition.distanceTo(target) < target.followDistance) {
                    jumpToNextSegment = true;
                }
            }

            if (jumpToNextSegment) currPoint++;
        } while (jumpToNextSegment && currPoint < waypoints.size() - 1);

        if (finished()) {return;}

//        WayPoint target = waypoints.get(min(currPoint + 1, waypoints.size() - 1));
        WayPoint target = waypoints.get(currPoint + 1);
        // If we're making a stop and in the stop portion of the move
        if (target instanceof StopWayPoint && robotPosition.distanceTo(target) < target.followDistance) {
            robot.setPowers(MecanumPurePursuitController.goToPosition(
                    robotPosition, robotVelocity, target, (StopWayPoint) target));
            Log.v("PP", "Locking onto point " + target);
        } else {
            trackToLine(robotPosition, robotVelocity, waypoints.get(currPoint), target);
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
        Line currSegment = new Line(start, end);
        Point center = currSegment.nearestLinePoint(robotPosition);

        Point intersection = MathUtil.lineSegmentCircleIntersection(
                start, end, center, end.followDistance
        );

        WayPoint target = end.clone();
//        if (intersection != null) { // if there wasn't an intersection, we go to the known target point
            target.x = intersection.x;
            target.y = intersection.y;
//        }

        if (end instanceof StopWayPoint) robot.setPowers(MecanumPurePursuitController.goToPosition(robotPosition, robotVelocity, target, (StopWayPoint) end));
        else robot.setPowers(MecanumPurePursuitController.goToPosition(robotPosition, robotVelocity, target, null));
    }

    public void draw(Canvas canvas) {
        double[] xPoints = new double[waypoints.size()];
        double[] yPoints = new double[waypoints.size()];

        for (int i = 0; i < waypoints.size(); i++) {
            xPoints[i] = waypoints.get(i).toFTCSystem().x;
            yPoints[i] = waypoints.get(i).toFTCSystem().y;
        }

        canvas.setStroke("red").setStrokeWidth(1).strokePolyline(xPoints, yPoints);
    }

    public boolean finished() {
        return currPoint >= waypoints.size() - 1;
    }
}
