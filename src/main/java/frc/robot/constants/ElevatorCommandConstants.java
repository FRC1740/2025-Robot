package frc.robot.constants;
/** 
 * Holds data for specifically controling the elevator and hand superstructure via commands
*/

import java.lang.reflect.Array;
import java.util.List;

public class ElevatorCommandConstants {

    public ElevatorPoseConstaint[] elevatorConstraints = new ElevatorPoseConstaint[] {
        // foo constraint for now
        new ElevatorPoseConstaint( 
            new ElevatorPose(.2, 0), 
            new ElevatorPose(.3, 0.5 * Math.PI))
    };

    /**
     * A class for defining an elevator position
     * allowing for smooth interpolation between poses
     */
    public class ElevatorPose {
        public double handPosition; // hand pos in radians
        public double elevatorPosition;

        /**
         * @param elevatorPosition elevator pos in 0-1 range
         * @param handPosition hand pos in radians (0 is straight down)
         */
        public ElevatorPose(double elevatorPosition, double handPosition) {
            this.elevatorPosition = elevatorPosition;
            this.handPosition = handPosition;
        }
    }

    /**
     * A class for defining a box where an elevator pose is invalid
     * used for pathfinding
     */
    public class ElevatorPoseConstaint {

        // points that define x1y1 and x2y2 of the contraint box
        public ElevatorPose lowerPoint; // lowest point (based on elevator)
        public ElevatorPose upperPoint; // highest point

        /**
         * @param elevatorPosition elevator pos in 0-1 range
         * @param handPosition hand pos in radians (0 is straight down)
         */
        public ElevatorPoseConstaint(ElevatorPose point1, ElevatorPose point2) {
            if (point1.elevatorPosition > point2.elevatorPosition) {
                upperPoint = point1;
                lowerPoint = point2;
            }else {
                lowerPoint = point1;
                upperPoint = point2;
            }
        }
    }
}
