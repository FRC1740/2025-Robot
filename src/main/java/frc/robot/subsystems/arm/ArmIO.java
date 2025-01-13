package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public boolean driveClosedLoop = false;
    public double elevatorPositionMeters = 0.0;
    public double elevatorAppliedVolts = 0.0;
    public double elevatorCurrentAmps = 0.0;
  }
  /** Run the motor at the specified velocity. */
  public default void setTarget(double target) {}

  /** Run the motor at the specified velocity. */
  public default void runVelocity(double velocity) {}

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}
}
