package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElevatorDriveIOSim implements ElevatorDriveIO {
  // TunerConstants doesn't support separate sim constants, so they are declared locally
  private static final double DRIVE_KP = 0.05;
  private static final double DRIVE_KD = 0.0;
  private static final double DRIVE_KS = 0.0;

  private final DCMotorSim elevatorSim;
  private static final DCMotor DRIVE_GEARBOX = DCMotor.getNEO(1);
  private boolean driveClosedLoop = false;
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private PIDController driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
  private static final double DRIVE_KV_ROT =
      0.91035; // Same units as TunerConstants: (volt * secs) / rotation
  private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);

  public ElevatorDriveIOSim() {

    elevatorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DRIVE_GEARBOX, 0.025, 1.0 / 2.0), DRIVE_GEARBOX);
  }

  @Override
  public void updateInputs(ElevatorDriveIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts + driveController.calculate(elevatorSim.getAngularVelocityRadPerSec());
    } else {
      driveController.reset();
    }

    // Update simulation state
    elevatorSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    elevatorSim.update(0.02);

    // Update drive inputs
    inputs.elevatorAppliedVolts = driveAppliedVolts;
    // inputs.driveConnected = true;
    // inputs.drivePositionRad = driveSim.getAngularPositionRad();
    // inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    // inputs.driveAppliedVolts = driveAppliedVolts;
    // inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    // // Update turn inputs
    // inputs.turnConnected = true;
    // inputs.turnEncoderConnected = true;
    // inputs.turnAbsolutePosition = new Rotation2d(turnSim.getAngularPositionRad());
    // inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    // inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    // inputs.turnAppliedVolts = turnAppliedVolts;
    // inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

    // // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't
    // // matter)
    // inputs.odometryTimestamps = new double[] { Timer.getFPGATimestamp() };
    // inputs.odometryDrivePositionsRad = new double[] { inputs.drivePositionRad };
    // inputs.odometryTurnPositions = new Rotation2d[] { inputs.turnPosition };
  }

  @Override
  public void runVelocity(double velocity) {
    driveClosedLoop = true;
    driveFFVolts = DRIVE_KS * Math.signum(velocity) + DRIVE_KV * velocity;
    driveController.setSetpoint(velocity);
  }
}
