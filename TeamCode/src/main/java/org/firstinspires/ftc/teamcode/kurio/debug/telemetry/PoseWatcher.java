package org.firstinspires.ftc.teamcode.kurio.debug.telemetry;

import org.firstinspires.ftc.teamcode.kurio.math.Pose;
import org.firstinspires.ftc.teamcode.kurio.mecanumpursuit.PurePursuitPath;

interface PoseWatcher {
    void sendPose(Pose pose);

    void sendPath(PurePursuitPath path);
}
