package org.firstinspires.ftc.teamcode.kurio.math;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;

class MathUtilTest {

    @Test
    public void test_line_circle_intersect_simple() {
        Point center = new Point(5, 5);
        int radius = 2;

        // Horizontal line that intercepts
        Point interceptH1 = MathUtil.lineSegmentCircleIntersection(
                new Point(0, 3),
                new Point(6, 3),
                center,
                radius);
        assertNotNull(interceptH1);
        assertEquals(5.0, interceptH1.x, MathUtil.EPSILON);
        assertEquals(3.0, interceptH1.y,  MathUtil.EPSILON);

        // Horizontal line that does not intercept
        Point interceptH2 = MathUtil.lineSegmentCircleIntersection(
                new Point(0, 1),
                new Point(6, 1),
                center,
                radius);
        assertNull(interceptH2);

        // Horizontal line cross over
        Point interceptH3 = MathUtil.lineSegmentCircleIntersection(
                new Point(0, 4),
                new Point(6, 4),
                center,
                radius);
        assertNotNull(interceptH3);
        assertEquals(4.0, interceptH3.y,  MathUtil.EPSILON);

        // Vertical line that intercepts
        Point interceptV1 = MathUtil.lineSegmentCircleIntersection(
                new Point(3, 0),
                new Point(3, 6),
                center,
                radius);
        assertNotNull(interceptV1);
        assertEquals(3.0, interceptV1.x, MathUtil.EPSILON);
        assertEquals(5.0, interceptV1.y, MathUtil.EPSILON);

        // Vertical line that does not intercept
        Point interceptV2 = MathUtil.lineSegmentCircleIntersection(
                new Point(1, 0),
                new Point(1, 6),
                center,
                radius);
        assertNull(interceptV2);

        // Vertical line that does not intercept
        Point interceptV3 = MathUtil.lineSegmentCircleIntersection(
                new Point(10, 0),
                new Point(10, 6),
                center,
                radius);
        assertNull(interceptV3);

        // Vertical line that falls short
        Point interceptV4 = MathUtil.lineSegmentCircleIntersection(
                new Point(3, 0),
                new Point(3, 3),
                center,
                radius);
        assertNotNull(interceptV4);
        assertEquals(3.0, interceptV4.x, MathUtil.EPSILON);
        assertEquals(5.0, interceptV4.y, MathUtil.EPSILON);

        double len = Math.sqrt(46.0);
        double angle = Math.PI / 4.0 - Math.asin(2.0 / Math.sqrt(50));
        double xt = len * Math.cos(angle);
        double yt = len * Math.sin(angle);
        Point intercept = MathUtil.lineSegmentCircleIntersection(
                new Point(0, 0),
                new Point(xt, yt),
                center,
                radius);
        assertEquals(xt, intercept.x, MathUtil.EPSILON);
        assertEquals(yt, intercept.y, MathUtil.EPSILON);
    }
}