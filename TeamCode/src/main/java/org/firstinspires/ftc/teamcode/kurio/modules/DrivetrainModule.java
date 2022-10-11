package org.firstinspires.ftc.teamcode.kurio.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.kurio.mecanumpursuit.MecanumPowers;

public class DrivetrainModule implements Module {
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

        fLeft.setDirection(DcMotor.Direction.REVERSE);
        bLeft.setDirection(DcMotor.Direction.REVERSE);
        fRight.setDirection(DcMotor.Direction.FORWARD);
        bRight.setDirection(DcMotor.Direction.FORWARD);

        powers = MecanumPowers.REST;
    }

    @Override
    public void update() {
        setMotorPower(fLeft, powers.frontLeft);
        setMotorPower(fRight, powers.frontRight);
        setMotorPower(bLeft, powers.backLeft);
        setMotorPower(bRight, powers.backRight);
    }

    public void setMotorPowers(MecanumPowers powers) {
        this.powers = powers;
    }

    private void setMotorPower(DcMotor motor, double power) {
        if (Math.abs(power) < 0.06) {
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