package org.firstinspires.ftc.teamcode.kurio.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.kurio.Robot;
import org.firstinspires.ftc.teamcode.kurio.math.Pose;
import org.firstinspires.ftc.teamcode.kurio.mecanumpursuit.PurePursuitPath;
import org.firstinspires.ftc.teamcode.kurio.mecanumpursuit.waypoints.StopWayPoint;
import org.firstinspires.ftc.teamcode.kurio.mecanumpursuit.waypoints.WayPoint;

import java.util.List;

//@Config
@Autonomous
public class Amogotonomous extends LinearOpMode {
    List<WayPoint> points = WayPoint.collate(
            new WayPoint(Pose.ZERO, 4),
            new WayPoint(1, 20, 10),
            new StopWayPoint(Pose.ZERO.x, 40, 4, 0, 1)
    );

    @Override
    public void runOpMode() {
        Robot robit = new Robot(this);

        PurePursuitPath followPath = new PurePursuitPath(robit, points);

        waitForStart();

        while (!followPath.finished()) {
            followPath.update();
        }
    }
}
