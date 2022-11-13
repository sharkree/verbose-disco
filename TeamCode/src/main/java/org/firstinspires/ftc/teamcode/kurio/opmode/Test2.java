package org.firstinspires.ftc.teamcode.kurio.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.kurio.Robot;
import org.firstinspires.ftc.teamcode.kurio.math.Pose;
import org.firstinspires.ftc.teamcode.kurio.purepursuit.PurePursuitPath;
import org.firstinspires.ftc.teamcode.kurio.purepursuit.waypoints.HeadingControlledWaypoint;
import org.firstinspires.ftc.teamcode.kurio.purepursuit.waypoints.StopWayPoint;
import org.firstinspires.ftc.teamcode.kurio.purepursuit.waypoints.WayPoint;

import java.util.List;

@Autonomous
public class Test2 extends LinearOpMode {
    List<WayPoint> points = WayPoint.collate(
            new WayPoint(Pose.ZERO, 8),
            new HeadingControlledWaypoint(-5, 20, 8, Math.PI / 2),
            new StopWayPoint(15, 48, 6, Math.PI / 2, 1.0)
    );

    @Override
    public void runOpMode() {
        Robot robit = new Robot(this, new Pose(0, 0, Math.PI / 2));

        PurePursuitPath followPath = new PurePursuitPath(robit, points);

        waitForStart();

        while (!followPath.finished()) {
            robit.getTelemetryDump().sendPath(followPath);

            followPath.update();
        }
    }
}
