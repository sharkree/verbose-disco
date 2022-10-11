package org.firstinspires.ftc.teamcode.kurio.modules;

import android.os.SystemClock;

import org.firstinspires.ftc.teamcode.kurio.Robot;
import org.firstinspires.ftc.teamcode.kurio.debug.telemetry.Telemeter;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class ModuleThread implements Runnable, Telemeter {
    private final Robot robot;
    private final Module[] modules;

    private boolean started = false;

    private long updateDuration = 0;
    private Map<String, Long> moduleUpdateTimes;

    public ModuleThread(Robot robot, Module[] modules) {
        this.robot = robot;
        this.modules = modules;

        moduleUpdateTimes = new HashMap<>(modules.length * 3);
        robot.getTelemetryDump().registerTelemeter(this);
        for (Module m : modules) robot.getTelemetryDump().registerTelemeter(m);
    }

    public void run() {
        while (robot.running()) {
            long overallStart = SystemClock.elapsedRealtime();

            Map<String, Long> aTime = new HashMap<>(3);
            for (Module module : modules) {
                if (module.isOn()) {
                    long start = SystemClock.elapsedRealtime();
                    module.update();
                    aTime.put(module.getName(), SystemClock.elapsedRealtime() - start);
                }
            }

            synchronized (this) {
                moduleUpdateTimes = aTime;
                updateDuration = SystemClock.elapsedRealtime() - overallStart;
            }
        }
    }

    @Override
    public List<String> getTelemetryData() {
        List<String> retVal = new ArrayList<>();
        retVal.add("Module thread total update time: " + updateDuration);
        for (Module m : modules) {
            retVal.add(m.getName() + " update time: " + moduleUpdateTimes.get(m.getName()));
        }

        return retVal;
    }

    @Override
    public String getName() {
        return "ModuleThread";
    }

    @Override
    public boolean isOn() {
        return true;
    }
}