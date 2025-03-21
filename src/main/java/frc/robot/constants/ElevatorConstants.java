package frc.robot.constants;

public class ElevatorConstants {
    public static final int elevatorCurrentLimit = 40; // amps

    public static final double elevatorGearRatio = 1.0 / 5.0; // 5:1 (5 mot rot = 1 rot)
    public static final double elevatorGearDiameterInches = 1.7453703703703705; // 
    public static final double elevatorGearCircumferenceInches = elevatorGearDiameterInches * Math.PI; // 2PIr = PId
    public static final double elevatorConversionFactor = elevatorGearRatio * elevatorGearCircumferenceInches;

    public static final double kP = 0.06; // .05
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.06;
    public static final double kFFDeadband = 0.07; // zone which error is acceptable, .1in both ways
    public static final double kFFGroundOffset = .3; // height the elevator must pass (dont care in stow)
    public static final double kGroundHalvingOffset = 1.5; // height the elevator must pass (dont care in stow)

    public static final double bumperWidth = 3.0 + 5.0/8.0; // chassis-bumper edge
}
