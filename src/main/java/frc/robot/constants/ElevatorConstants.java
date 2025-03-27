package frc.robot.constants;

public class ElevatorConstants {
    public static final int elevatorCurrentLimit = 40; // amps

    public static final double elevatorGearRatio = 1.0 / 9.0; // 5:1 (5 mot rot = 1 rot)
    public static final double elevatorGearDiameterInches = 1.7453703703703705; // 
    public static final double elevatorGearCircumferenceInches = elevatorGearDiameterInches * Math.PI; // 2PIr = PId
    public static final double elevatorConversionFactor = elevatorGearRatio * elevatorGearCircumferenceInches;

    public static final double kP = 0.23; // .05
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.0;
    public static final double kFFDeadband = 0.0; // zone which error is acceptable, .1in both ways
    public static final double kFFGroundOffset = 0.0; // height the elevator must pass (dont care in stow)
    public static final double kGroundHalvingOffset = 0.0; // height the elevator must pass (dont care in stow)
    public static final double k4HalvingOffset = 1000.0; // height the elevator must pass (dont care in stow)

    public static final double bumperWidth = 3.0 + 5.0/8.0; // chassis-bumper edge
}
