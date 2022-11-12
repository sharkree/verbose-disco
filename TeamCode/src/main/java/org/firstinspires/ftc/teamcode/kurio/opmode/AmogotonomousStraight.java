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

//@Config
@Autonomous
public class AmogotonomousStraight extends LinearOpMode {
    List<WayPoint> points = WayPoint.collate(
            new HeadingControlledWaypoint(Pose.ZERO, 4),
            new HeadingControlledWaypoint(-5, 20, 4, 0),
//            new HeadingControlledWaypoint(10, 20, 8, 0),
//            new HeadingControlledWaypoint(0, 40, 6, 0),
            new StopWayPoint(20, 40, 4, 0, 1.0)
    );

    @Override
    public void runOpMode() {
        Robot robit = new Robot(this);

        waitForStart();
        PurePursuitPath followPath = new PurePursuitPath(robit, points);

        while (!followPath.finished()) {
            robit.getTelemetryDump().sendPath(followPath);

            followPath.update();
        }
    }
}
