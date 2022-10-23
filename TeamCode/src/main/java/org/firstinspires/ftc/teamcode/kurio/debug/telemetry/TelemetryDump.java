package org.firstinspires.ftc.teamcode.kurio.debug.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.kurio.math.Pose;
import org.firstinspires.ftc.teamcode.kurio.mecanumpursuit.PurePursuitPath;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;

public class TelemetryDump implements PoseWatcher {
    private final Telemetry telemetry;
    private FtcDashboard dashboard;
    private final List<Pose> poseHistory = new ArrayList<>();

    private final ConcurrentLinkedQueue<Telemeter> telemeters = new ConcurrentLinkedQueue<>();

    public TelemetryDump(Telemetry telemetry) {
        this.telemetry = telemetry;
        this.dashboard = FtcDashboard.getInstance();
        while (dashboard == null) {
            this.dashboard = FtcDashboard.getInstance();
            telemetry.addData("dashboard not working: ", System.currentTimeMillis());
            telemetry.update();
        }
        this.dashboard.setTelemetryTransmissionInterval(25);
    }

    public void registerTelemeter(Telemeter telemeter) {
        telemeters.add(telemeter);
    }

    public void removeTelemeter(Telemeter telemeter) {
        telemeters.remove(telemeter);
    }

    public void update() {
        telemetry.addLine(getData());
        telemetry.update();
    }

    @Override
    public void sendPose(Pose pose) {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();

        pose = pose.toFTCSystem();

        poseHistory.add(pose);
        DashboardUtil.drawRobot(canvas, pose);
        DashboardUtil.drawPoseHistory(canvas, poseHistory);

        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void sendPath(PurePursuitPath path) {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();


        path.draw(canvas);

        dashboard.sendTelemetryPacket(packet);
    }

    private String getData() {
        StringBuilder allData = new StringBuilder("TELEMETRY DATA\n\n");

        for (Telemeter telemeter : telemeters) {
            if (!telemeter.isOn()) continue;

            allData.append("---");
            allData.append(telemeter.getName());
            allData.append("---\n");
            List<String> tmp = telemeter.getTelemetryData();
            for (String s : tmp) {
                allData.append(s);
                allData.append("\n");
            }
            allData.append("\n\n");
        }

        return allData.toString();
    }
}
