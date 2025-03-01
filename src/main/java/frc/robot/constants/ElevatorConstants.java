package frc.robot.constants;

public class ElevatorConstants {
    public static final int elevatorCurrentLimit = 20; // amps

    public static final double elevatorGearRatio = 1.0 / 12.0; // 12:1 (12 mot rot = 1 rot)
    public static final double elevatorGearDiameterInches = 1.7453703703703705; // 
    public static final double elevatorGearCircumferenceInches = elevatorGearDiameterInches * Math.PI; // 2PIr = PId
    public static final double elevatorConversionFactor = elevatorGearRatio * elevatorGearCircumferenceInches;

    public static final double kP = 0.2;
    public static final double kI = 0.001;
    public static final double kD = 0.0;

    public static final double bumperWidth = 3.0 + 5.0/8.0; // chassis-bumper edge
}
