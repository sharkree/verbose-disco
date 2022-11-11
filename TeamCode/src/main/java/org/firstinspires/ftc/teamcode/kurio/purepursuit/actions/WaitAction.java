package org.firstinspires.ftc.teamcode.kurio.purepursuit.actions;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.kurio.Robot;

public class WaitAction implements Actions.InterruptingActionOnArrival {
    double waitMS;
    ElapsedTime timer;

    public WaitAction(double waitMS) {
        this.waitMS = waitMS;
        this.timer = null;
    }

    @Override
    public boolean runCycle(Robot robot) {
        if (timer == null) {
            timer = new ElapsedTime();
        }

        return timer.milliseconds() > waitMS;
    }
}
