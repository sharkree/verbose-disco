package org.firstinspires.ftc.teamcode.kurio;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.kurio.debug.DebugThread;
import org.firstinspires.ftc.teamcode.kurio.debug.telemetry.TelemetryDump;
import org.firstinspires.ftc.teamcode.kurio.math.Pose;
import org.firstinspires.ftc.teamcode.kurio.odometry.SensorThread;
import org.firstinspires.ftc.teamcode.kurio.mecanumpursuit.MecanumPowers;
import org.firstinspires.ftc.teamcode.kurio.modules.DrivetrainModule;
import org.firstinspires.ftc.teamcode.kurio.modules.Module;
import org.firstinspires.ftc.teamcode.kurio.modules.ModuleThread;

public class Robot {
    // threads
    private final SensorThread sensorThread;
    private final ModuleThread moduleThread;
    private final DebugThread debugThread;

    // modules
    private final DrivetrainModule drivetrainModule;

    // other members
    private final LinearOpMode linearOpMode;
    private final TelemetryDump telemetryDump;

    // electronics
    private final LynxModule controlHub;

    public Robot(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;

        this.telemetryDump = new TelemetryDump(linearOpMode.telemetry);

        try {
            controlHub = linearOpMode.hardwareMap.get(LynxModule.class, "Control Hub");
            controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        } catch (RuntimeException e) {
            throw new RuntimeException("One or more of the REV hubs could not be found. More info: " + e);
        }

        drivetrainModule = new DrivetrainModule(linearOpMode.hardwareMap);

        Module[] modules = new Module[]{
                drivetrainModule
        };

        // threads
        debugThread = new DebugThread(this);
        moduleThread = new ModuleThread(this, modules);
        sensorThread = new SensorThread(this);

        this.start();
    }

    public void start() {
        Thread[] threads;
        threads = new Thread[]{
                new Thread(debugThread, "DebugThread"),
                new Thread(sensorThread, "SensorThread"),
                new Thread(moduleThread, "ModuleThread")
        };

        for (Thread thread : threads) {
            thread.start();
        }
    }

    public boolean isOpModeActive() {
        return linearOpMode.opModeIsActive();
    }

    public boolean isTeleOp() {
        return !linearOpMode.getClass().isAnnotationPresent(Autonomous.class);
    }

    public boolean running() {
        return (!linearOpMode.isStopRequested() && !linearOpMode.isStarted()) || isOpModeActive();
    }

    public boolean started() {
        return linearOpMode.isStarted();
    }

    public LynxModule getControlHub() {
        return controlHub;
    }

    public DrivetrainModule getDrivetrainModule(){
        return drivetrainModule;
    }

    public HardwareMap getHardwareMap(){
        return this.linearOpMode.hardwareMap;
    }

    public LinearOpMode getOpMode() {
        return linearOpMode;
    }

    public TelemetryDump getTelemetryDump() {
        return this.telemetryDump;
    }

    public SensorThread getSensorThread() {
        return sensorThread;
    }

    public DebugThread getDebugThread() { return debugThread; }

    public Pose getPose() {
        return this.sensorThread.getOdometry().getPose();
    }

    public Pose getVelocity() {
        return this.sensorThread.getOdometry().getVelocity();
    }

    public void setPowers(MecanumPowers powers) {
        this.drivetrainModule.setMotorPowers(powers);
    }
}
