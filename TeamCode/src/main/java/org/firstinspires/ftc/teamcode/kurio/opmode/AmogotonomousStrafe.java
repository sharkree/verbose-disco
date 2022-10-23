package org.firstinspires.ftc.teamcode.kurio.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.kurio.Robot;
import org.firstinspires.ftc.teamcode.kurio.math.Pose;
import org.firstinspires.ftc.teamcode.kurio.mecanumpursuit.PurePursuitPath;
import org.firstinspires.ftc.teamcode.kurio.mecanumpursuit.waypoints.StopWayPoint;
import org.firstinspires.ftc.teamcode.kurio.mecanumpursuit.waypoints.WayPoint;

import java.util.List;

@Autonomous
public class AmogotonomousStrafe extends LinearOpMode {
    List<WayPoint> points = WayPoint.collate(
            new WayPoint(Pose.ZERO, 8),
            new WayPoint(20, 0, 8),
            new StopWayPoint(41, 0, 6, 0, 1.0)
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
