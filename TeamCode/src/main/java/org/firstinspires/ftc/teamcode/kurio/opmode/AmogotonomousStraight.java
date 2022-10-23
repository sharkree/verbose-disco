package org.firstinspires.ftc.teamcode.kurio.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.kurio.Robot;
import org.firstinspires.ftc.teamcode.kurio.math.Pose;
import org.firstinspires.ftc.teamcode.kurio.mecanumpursuit.PurePursuitPath;
import org.firstinspires.ftc.teamcode.kurio.mecanumpursuit.waypoints.HeadingControlledWaypoint;
import org.firstinspires.ftc.teamcode.kurio.mecanumpursuit.waypoints.StopWayPoint;
import org.firstinspires.ftc.teamcode.kurio.mecanumpursuit.waypoints.WayPoint;

import java.util.List;

//@Config
@Autonomous
public class AmogotonomousStraight extends LinearOpMode {
    List<WayPoint> points = WayPoint.collate(
            new WayPoint(Pose.ZERO, 8),
            new WayPoint(Pose.ZERO.x, 10, 8),
            new HeadingControlledWaypoint(10, 20, 8, 0),
            new HeadingControlledWaypoint(0, 40, 6, 0),
            new StopWayPoint(Pose.ZERO.x, 40, 4, 0, 0.2)
    );

    @Override
    public void runOpMode() {
        Robot robit = new Robot(this);

        PurePursuitPath followPath = new PurePursuitPath(robit, points);

        waitForStart();

        while (!followPath.finished()) {
            robit.getTelemetryDump().sendPath(followPath);

            followPath.update();
        }
    }
}
