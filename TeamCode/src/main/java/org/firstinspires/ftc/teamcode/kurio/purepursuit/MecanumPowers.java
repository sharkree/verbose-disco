package org.firstinspires.ftc.teamcode.kurio.purepursuit;

import org.firstinspires.ftc.teamcode.kurio.math.MathUtil;
import org.firstinspires.ftc.teamcode.kurio.math.Pose;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

public class MecanumPowers {
    public double frontLeft;
    public double frontRight;
    public double backLeft;
    public double backRight;

    public static final MecanumPowers REST = new MecanumPowers(0.0, 0.0, 0.0, 0.0);

    public MecanumPowers(double frontLeft, double frontRight, double backLeft, double backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }

    public MecanumPowers(double x, double y, double turnPower) {
        this.frontLeft = y - turnPower - x;
        this.backLeft = y - turnPower + x;
        this.frontRight = y + turnPower + x;
        this.backRight = y + turnPower - x;

        this.scale();
    }

    public MecanumPowers(Pose p) {
        this(p.x, p.y, p.heading);
    }

    public List<Double> asList() {
        return Arrays.asList(this.frontLeft, this.frontRight, this.backLeft, this.backRight);
    }

    // If we're somehow above one, scale back down
    private void scale() {
        List<Double> vals = asList();
        double absMax = Math.max(Collections.max(vals), -Collections.min(vals));
        if (absMax > 1) {
            this.frontLeft /= absMax;
            this.frontRight /= absMax;
            this.backLeft /= absMax;
            this.backRight /= absMax;
        }

//        if (this.frontLeft < 0.06) this.frontLeft = 0;
//        if (this.frontRight < 0.06) this.frontRight = 0;
//        if (this.backLeft < 0.06) this.backLeft = 0;
//        if (this.backRight < 0.06) this.backRight= 0;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        MecanumPowers mecanumPowers = (MecanumPowers) o;
        return MathUtil.approxEquals(mecanumPowers.frontLeft, this.frontLeft) &&
                MathUtil.approxEquals(mecanumPowers.frontRight, this.frontRight) &&
                MathUtil.approxEquals(mecanumPowers.backLeft, this.backLeft) &&
                MathUtil.approxEquals(mecanumPowers.backRight, this.backRight);
    }

    /*
    %.1f----%.1f
    | Front |
    |       |
    |       |
    %.1f----%.1f
     */
    @Override
    public String toString() {
        return String.format(Locale.US,
                "\n\n" +
                        "(%.3f)---(%.3f)\n" +
                        "|   Front   |\n" +
                        "|           |\n" +
                        "|           |\n" +
                        "(%.3f)---(%.3f)\n"
                , frontLeft, frontRight, backLeft, backRight);
    }
}
