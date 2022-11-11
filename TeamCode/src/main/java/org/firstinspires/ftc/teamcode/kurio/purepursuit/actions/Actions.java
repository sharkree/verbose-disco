package org.firstinspires.ftc.teamcode.kurio.purepursuit.actions;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.kurio.Robot;
import org.firstinspires.ftc.teamcode.kurio.purepursuit.PurePursuitPath;

@Config
public class Actions {
    public interface Action {}

    public interface DoOnceAction extends Action {
        void runOnce(Robot robot);
    }

    public interface RepeatedAction extends Action {
        boolean runLoop(Robot robot, PurePursuitPath path); // Returns whether we should advance
    }

    // Once a position is reached, if that position has an ArrivalInterruptSubroutine, we advance to
    // the next waypoint (so the waypoint with the ArrivalInterruptSubroutine is our current waypoint)
    // and we call runCycle every tick until it eventually returns true
    public interface InterruptingActionOnArrival extends Action {
        boolean runCycle(Robot robot); // Returns whether it's complete
    }
}