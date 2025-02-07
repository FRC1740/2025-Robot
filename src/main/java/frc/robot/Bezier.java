package frc.robot;
// I can't believe I have to write this

import org.opencv.core.Point;

import frc.robot.constants.ElevatorCommandConstants.ElevatorPoseConstraint;
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

    public Boolean isColliding(int resolution, ElevatorPoseConstraint constraint) {
        double sampleDelta = 1.0 / resolution;
        double xMin = Math.min(constraint.lowerPoint.elevatorPosition, constraint.upperPoint.elevatorPosition);
        double xMax = Math.max(constraint.lowerPoint.elevatorPosition, constraint.upperPoint.elevatorPosition);
        double yMin = constraint.lowerPoint.handPosition;
        double yMax = constraint.upperPoint.handPosition;

        for(double t = 0.0; t < 1.0; t += sampleDelta) {
            Point sample = sample(t);
            if (sample.y > yMin &&
                sample.y < yMax &&
                sample.x > xMin &&
                sample.x < xMax) {
                return true;
            }
        }

        return false;
    }

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
