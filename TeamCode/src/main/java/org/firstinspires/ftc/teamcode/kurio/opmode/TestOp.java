package org.firstinspires.ftc.teamcode.kurio.opmode;

import static org.firstinspires.ftc.teamcode.kurio.purepursuit.MecanumUtil.toPowers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kurio.Robot;
import org.firstinspires.ftc.teamcode.kurio.math.Pose;

@TeleOp
public class TestOp extends LinearOpMode {
    private Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this, new Pose(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            updateDrivetrainModule();
        }
    }

    private void updateDrivetrainModule() {
        double x = Math.signum(gamepad1.left_stick_x) * Math.pow(gamepad1.left_stick_x, 2);
        double y = Math.signum(gamepad1.left_stick_y) * Math.pow(gamepad1.left_stick_y, 2);
        double theta = Math.signum(gamepad1.right_stick_x) * Math.pow(gamepad1.right_stick_x, 2);

        robot.setPowers(toPowers(x, y, theta));
    }
}
