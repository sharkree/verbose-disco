package org.firstinspires.ftc.teamcode.kurio.debug.telemetry;

import java.util.ArrayList;
import java.util.List;

public interface Telemeter {
    default List<String> getTelemetryData() {
        return new ArrayList<>();
    }

    String getName();

    boolean isOn();
}