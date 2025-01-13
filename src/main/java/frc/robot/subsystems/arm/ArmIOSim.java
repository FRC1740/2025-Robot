package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
  // TunerConstants doesn't support separate sim constants, so they are declared locally
  private static final double DRIVE_KP = 0.3;
  private static final double DRIVE_KD = 0.0;

  private final SingleJointedArmSim elevatorSim;
  private static final DCMotor DRIVE_GEARBOX = DCMotor.getNEO(1);
  private boolean driveClosedLoop = false;
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private PIDController driveController = new PIDController(DRIVE_KP, 0.0, DRIVE_KD);

  public ArmIOSim() {
    elevatorSim =
        new SingleJointedArmSim(DRIVE_GEARBOX, 12.0 / 1.0, 1.0, .4, 0, 2 * Math.PI, true, Math.PI);
    // , 12.0 / 1.0, 9.0, .1, 0.0, 1.01, true, 0.1);
  }

  public static double normalizeRads(double rads) {
    // Normalize rads to the range [-PI, PI]
    rads = rads % (2 * Math.PI); // Ensure it's within [-2PI, 2PI]

    // Adjust if the angle is outside the desired range
    if (rads > Math.PI) {
      rads -= 2 * Math.PI; // Wrap around if above PI
    } else if (rads < -Math.PI) {
      rads += 2 * Math.PI; // Wrap around if below -PI
    }

    return rads;
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts
              + driveController.calculate(normalizeRads(elevatorSim.getAngleRads())) * 12.0;
    } else {
      driveController.reset();
    }

    // Update simulation state
    elevatorSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    elevatorSim.update(0.02);

    // Update drive inputs
    inputs.elevatorAppliedVolts = driveAppliedVolts;
    inputs.elevatorPositionMeters = normalizeRads(elevatorSim.getAngleRads());
    inputs.driveClosedLoop = driveClosedLoop;
    inputs.elevatorCurrentAmps = elevatorSim.getCurrentDrawAmps();
  }

  @Override
  public void setTarget(double position) {
    driveClosedLoop = true;
    driveFFVolts = 6.5; // DRIVE_KS * Math.signum(velocity) + DRIVE_KV * velocity;
    driveController.setSetpoint(position);
  }

  @Override
  public void runVelocity(double velocity) {
    assert (1 == 3);
    // driveClosedLoop = true;
    // // driveFFVolts = velocity * 5.0; // DRIVE_KS * Math.signum(velocity) + DRIVE_KV * velocity;
    // driveController.setSetpoint(velocity);
  }
}
