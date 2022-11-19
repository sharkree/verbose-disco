package org.firstinspires.ftc.teamcode.kurio.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.kurio.Robot;
import org.firstinspires.ftc.teamcode.kurio.math.Pose;
import org.firstinspires.ftc.teamcode.kurio.purepursuit.PurePursuitPath;
import org.firstinspires.ftc.teamcode.kurio.purepursuit.waypoints.HeadingControlledWayPoint;
import org.firstinspires.ftc.teamcode.kurio.purepursuit.waypoints.StopWayPoint;
import org.firstinspires.ftc.teamcode.kurio.purepursuit.waypoints.WayPoint;

import java.util.List;

@Autonomous
public class Test2 extends LinearOpMode {
    List<WayPoint> points = WayPoint.collate(
            new WayPoint(Pose.ZERO, 8),
            new HeadingControlledWayPoint(0, 20, 8, 0),
            new StopWayPoint(0, 40, 8, 0, 1.0)
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
