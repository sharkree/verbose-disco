package org.firstinspires.ftc.teamcode.kurio.purepursuit;

import static org.firstinspires.ftc.teamcode.kurio.purepursuit.MecanumPurePursuitController.rDistanceToTarget;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.firstinspires.ftc.teamcode.kurio.math.MathUtil;
import org.firstinspires.ftc.teamcode.kurio.math.Point;
import org.firstinspires.ftc.teamcode.kurio.math.Pose;
import org.firstinspires.ftc.teamcode.kurio.purepursuit.waypoints.HeadingControlledWayPoint;
import org.firstinspires.ftc.teamcode.kurio.purepursuit.waypoints.StopWayPoint;
import org.junit.jupiter.api.Test;

class MecanumPurePursuitControllerTest {

    @Test
    void rDistanceToTargetTestNormal() {
        Pose zero = Pose.ZERO;
        Point a = new Point(0, 5);

        assertEquals(new Pose(0, 5, 0), rDistanceToTarget(zero, a));
    }

    @Test
    void rDistanceToTargetTestHCW() {
        Pose zero = Pose.ZERO;
        HeadingControlledWayPoint a = new HeadingControlledWayPoint(new Pose(0, 5, 0), 5);

        assertEquals(new Pose(0, 5, 0), rDistanceToTarget(zero, a));
    }

    @Test
    void test_start() {
        Pose robotPose = new Pose(0, 0, 0);
        Pose robotVelocity = new Pose(0, 0, 0);
        StopWayPoint target = new StopWayPoint(0, 48, 8, 0, 1.0);
        MecanumPowers mecanumPowers = MecanumPurePursuitController.goToPosition(robotPose, robotVelocity, target, null);
        assertEquals(1.0, mecanumPowers.frontLeft);
        assertEquals(1.0, mecanumPowers.frontRight);
        assertEquals(1.0, mecanumPowers.backLeft);
        assertEquals(1.0, mecanumPowers.backRight);
    }

    @Test
    void test_end() {
        Pose robotPose = new Pose(0, 47.88, 0);
        Pose robotVelocity = new Pose(0, 4, 0);
        StopWayPoint target = new StopWayPoint(0, 48, 8, 0, 1.0);
        MecanumPowers mecanumPowers = MecanumPurePursuitController.goToPosition(robotPose, robotVelocity, target, target);
        assertEquals(0, mecanumPowers.frontLeft, MathUtil.HIGH_EPSILON);
        assertEquals(0, mecanumPowers.frontRight, MathUtil.HIGH_EPSILON);
        assertEquals(0, mecanumPowers.backLeft, MathUtil.HIGH_EPSILON);
        assertEquals(0, mecanumPowers.backRight, MathUtil.HIGH_EPSILON);
    }
}