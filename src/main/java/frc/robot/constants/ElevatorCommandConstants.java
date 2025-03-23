package frc.robot.constants;
/** 
 * Holds data for specifically controling the elevator and hand superstructure via commands
*/

public class ElevatorCommandConstants {
    public static double topDistance = -29.6; // foo encoder things

    // public static ElevatorPose L1Score = new ElevatorPose(-7.0, 0.35);
    // public static ElevatorPose L2Score = new ElevatorPose(-5, 0.25);
    // public static ElevatorPose L3Score = new ElevatorPose(-13, 0.25);
    // public static ElevatorPose L4Score = new ElevatorPose(-29, 0.35);
    // public static ElevatorPose L2Algae = new ElevatorPose(-3.2, 0.2);
    // public static ElevatorPose L3Algae = new ElevatorPose(-10, 0.2);
    // public static ElevatorPose Stow = new ElevatorPose(0.0, 0.1);
    // public static ElevatorPose Intake = new ElevatorPose(-4, .11);
    public static ElevatorPose L1Score = new ElevatorPose(-7.7, 0.39);
    public static ElevatorPose L2Score = new ElevatorPose(-4.19, 0.2);
    public static ElevatorPose L3Score = new ElevatorPose(-12.2, 0.2);
    public static ElevatorPose L4Score = new ElevatorPose(-28.7, 0.38);
    public static ElevatorPose L2Algae = new ElevatorPose(-2.7, 0.28);
    public static ElevatorPose L3Algae = new ElevatorPose(-10, 0.27);
    public static ElevatorPose Stow = new ElevatorPose(0.2, 0.1);
    public static ElevatorPose Intake = new ElevatorPose(-4.25, .09);

    /**
     * A class for defining an elevator position
     * allowing for smooth interpolation between poses
     */
    public static class ElevatorPose {
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
    public static class ElevatorPoseConstraint {

        // points that define x1y1 and x2y2 of the contraint box
        public ElevatorPose lowerPoint; // lowest point (based on elevator)
        public ElevatorPose upperPoint; // highest point

        /**
         * @param elevatorPosition elevator pos in 0-1 range
         * @param handPosition hand pos in radians (0 is straight down)
         */
        public ElevatorPoseConstraint(ElevatorPose point1, ElevatorPose point2) {
            if (point1.handPosition > point2.handPosition) {
                upperPoint = point1;
                lowerPoint = point2;
            }else {
                lowerPoint = point1;
                upperPoint = point2;
            }
        }
    }

    // public static ElevatorPoseConstraint[] elevatorConstraints = null;

    // static {
    //     elevatorConstraints = new ElevatorPoseConstraint[] {
    //         // foo constraint for now
    //         new ElevatorPoseConstraint( 
    //             new ElevatorPose(.2, 0), 
    //             new ElevatorPose(.3, 1.0 * Math.PI)),
    //         new ElevatorPoseConstraint( 
    //             new ElevatorPose(.5, 0), 
    //             new ElevatorPose(.6, 2.0 * Math.PI)),
    //         // new ElevatorPoseConstraint( 
    //         //     new ElevatorPose(0, 2.5), 
    //         //     new ElevatorPose(.1, Math.PI))
    //     };
    // }
}
