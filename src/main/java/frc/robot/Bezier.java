package frc.robot;
// I can't believe I have to write this

import org.opencv.core.Point;
/**
 * Quadratic Bézier class
 */
public class Bezier {
    // x, y, length
    public Point start = null;
    public Point control = null;
    public Point end = null;

    public Bezier(Point start, Point control, Point end) {
        this.start = start;
        this.control = control;
        this.end = end;
    }

    public Bezier() {}

    public Point sample(double t) {
        double mt = 1 - t;
        return new Point(
            start.x * mt * mt + 2 * control.x * mt * t + end.x * t * t,
            start.y * mt * mt + 2 * control.y * mt * t + end.y * t * t
        );
    }

    public String serialize(int resolution) {
        double sampleDelta = 1.0 / resolution;
        String result = "";

        for(double t = 0.0; t < 1.0; t += sampleDelta) {
            Point sample = sample(t);
            result += "," + sample.x + "," + sample.y;
        }
        result = result.substring(1); // trim leading comma
        return result;
    }
}
