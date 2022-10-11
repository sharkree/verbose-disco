package org.firstinspires.ftc.teamcode.kurio.debug;

import android.os.SystemClock;

import org.firstinspires.ftc.teamcode.kurio.Robot;
import org.firstinspires.ftc.teamcode.kurio.debug.telemetry.Telemeter;

import java.util.ArrayList;
import java.util.List;

public class DebugThread implements Runnable, Telemeter {
    private final Robot robot;

    private long updateTime = 0;
    private long lastLoopTime = 0;

    public DebugThread(Robot robot) {
        this.robot = robot;
        robot.getTelemetryDump().registerTelemeter(this);
    }

    @Override
    public void run() {
        while (robot.running()) {
            robot.getTelemetryDump().update();
            long currentTime = SystemClock.elapsedRealtime();
            updateTime = currentTime - lastLoopTime;
            lastLoopTime = currentTime;
        }
    }

    @Override
    public List<String> getTelemetryData() {
        List<String> returnValue = new ArrayList<>();
        returnValue.add("Debug thread update time: " + updateTime);
        return returnValue;
    }

    @Override
    public String getName() {
        return "DebugThread";
    }

    @Override
    public boolean isOn() {
        return true;
    }
}
