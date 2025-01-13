package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  ArmIO driveIO;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public Arm(ArmIO driveIO) {
    this.driveIO = driveIO;
  }

  public void periodic() {
    driveIO.updateInputs(inputs);
    Logger.processInputs("Arm/Drive", inputs);
  }

  /**
   * Runs the arm to the desired target.
   *
   * @param speed Speed in meters/sec
   */
  public void setTarget(double target) {

    driveIO.setTarget(target);

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("Arm/target", target);
  }

  /**
   * Runs the arm at the desired velocity.
   *
   * @param speed Speed in meters/sec
   */
  public void runVelocity(double speed) {

    driveIO.runVelocity(speed);

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("Arm/speed", speed);
  }
}
