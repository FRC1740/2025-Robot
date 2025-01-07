package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  ElevatorDriveIO driveIO;

  public Elevator(ElevatorDriveIO driveIO) {
    this.driveIO = driveIO;
  }

  /**
   * Runs the elevator at the desired velocity.
   *
   * @param speed Speed in meters/sec
   */
  public void runVelocity(double speed) {

    driveIO.runVelocity(speed);

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("ElevatorState/speed", speed);
  }
}
