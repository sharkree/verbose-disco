package org.firstinspires.ftc.teamcode.kurio.modules;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.kurio.purepursuit.MecanumPowers;

public class DrivetrainModule implements Module {
    public static final double EPSILON = 0.03;
    private final boolean isOn = true;

    private MecanumPowers powers;

    private final DcMotor fLeft;
    private final DcMotor fRight;
    private final DcMotor bLeft;
    private final DcMotor bRight;

    public DrivetrainModule(HardwareMap hardwareMap) {
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        fLeft.setDirection(DcMotor.Direction.FORWARD);
        bLeft.setDirection(DcMotor.Direction.FORWARD);
        fRight.setDirection(DcMotor.Direction.REVERSE);
        bRight.setDirection(DcMotor.Direction.REVERSE);

        powers = MecanumPowers.REST;
    }

    @Override
    public void update() {
        Log.v("Motor Powers", powers.toString());
        setMotorPower(fLeft, powers.frontLeft);
        setMotorPower(fRight, powers.frontRight);
        setMotorPower(bLeft, powers.backLeft);
        setMotorPower(bRight, powers.backRight);
    }

    public void setMotorPowers(MecanumPowers powers) {
        this.powers = powers;
    }

    private void setMotorPower(DcMotor motor, double power) {
        if (Math.abs(power) < EPSILON) {
            motor.setPower(0);
        } else {
            motor.setPower(power);
        }
    }

    @Override
    public boolean isOn() {
        return isOn;
    }

    @Override
    public String getName() {
        return "DrivetrainModule";
    }
}