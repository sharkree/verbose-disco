package org.firstinspires.ftc.teamcode.kurio.debug.telemetry;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.kurio.math.Pose;

import java.util.List;

/**
 * Set of helper functions for drawing Kuriosity robot and location history on dashboard canvases.
 */

public class DashboardUtil {
    public static final double ROBOT_RADIUS = 6.50;

    public static void drawPoseHistory(Canvas canvas, List<Pose> poseHistory) {
        canvas.setStrokeWidth(1);
        canvas.setStroke("#3F51B5");
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];
        for (int i = 0; i < poseHistory.size(); i++) {
            Pose pose = poseHistory.get(i);
            xPoints[i] = pose.x;
            yPoints[i] = pose.y;
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawRobot(Canvas canvas, Pose pose) {
        canvas.setStrokeWidth(1);
        canvas.setStroke("4CAF50");

        canvas.strokeCircle(pose.x, pose.y, ROBOT_RADIUS);
        Vector2d v = pose.getHeadingVector().times(ROBOT_RADIUS);
        double x1 = pose.x + v.getX() / 2, y1 = pose.y + v.getY() / 2;
        double x2 = pose.x + v.getX(), y2 = pose.y + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }
}