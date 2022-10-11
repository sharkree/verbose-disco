package org.firstinspires.ftc.teamcode.kurio.modules;

import org.firstinspires.ftc.teamcode.kurio.debug.telemetry.Telemeter;

public interface Module extends Telemeter {
    void update();

    boolean isOn();

    String getName();
}