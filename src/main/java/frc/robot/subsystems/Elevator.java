package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;
import frc.robot.constants.CanIds;
import frc.robot.constants.ElevatorCommandConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.generated.TunerConstants;

public class Elevator extends SubsystemBase {
    private static double kDt = 0.02;
    SparkBase elevator = null;
    RelativeEncoder elevatorEncoder = null;
    PIDController elevatorController = null;
    private final TrapezoidProfile m_profile =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(10000.0, 150.0));
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
    // ShuffleboardTab elevatorTab = null;
    public double targetPosition = 0.0;
    
    Telemetry m_telemetry = null;

    private static Elevator instance;

    public static Elevator getInstance() {
      if(instance == null) {
        instance = new Elevator();
      }
      return instance;
    }

    public Elevator() {
        m_telemetry = Telemetry.getInstance();
        elevator = new SparkMax(CanIds.elevatorCanId, MotorType.kBrushless);
        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        elevatorConfig.encoder.positionConversionFactor(ElevatorConstants.elevatorConversionFactor);
        elevatorConfig.softLimit.forwardSoftLimitEnabled(true);
        elevatorConfig.softLimit.forwardSoftLimit(0.0);
        elevatorConfig.softLimit.reverseSoftLimitEnabled(true);
        elevatorConfig.softLimit.reverseSoftLimit(ElevatorCommandConstants.topDistance);
        elevatorConfig.smartCurrentLimit(ElevatorConstants.elevatorCurrentLimit);

        elevatorConfig.idleMode(IdleMode.kBrake);

        elevatorController = new PIDController(
            ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
        
        
        elevator.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        elevatorEncoder = elevator.getEncoder();
    }

    @Override
    public void periodic() {
        m_telemetry.telemeterizeElevator(getElevatorPosition(), getElevatorSetpoint(), elevator.getOutputCurrent(), elevatorController.calculate(elevatorEncoder.getPosition()), atPose());
    }

    /**
     * Runs one step to optimize the PID and get new outputs for the inputs
     */
    public void seekPosition() {
        // double output = elevatorController.calculate(elevatorEncoder.getPosition());
        // if (output > 0.0) {
        //     output /= 6.0;
        //     output -= 0.1;
        //     output = Math.max(output, 0.0);
        // }
        // elevator.set(
        //     output);
        m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);
        elevatorController.setSetpoint(m_setpoint.position);
        double output = elevatorController.calculate(elevatorEncoder.getPosition());
        elevator.set(output);

        // System.out.println(elevatorController.calculate(elevatorEncoder.getPosition()) * ElevatorConstants.outputFactor);
    }

    public void stop() {
        elevator.set(0.0);
    }

    /**
     * Set the elevator setpoint up to the specified position in inches 
     * @param inches distance from bottom
     */
    public void setElevatorToPosition(double inches) {
        elevatorController.setSetpoint(inches);
        m_goal = new TrapezoidProfile.State(inches, 0);
        targetPosition = inches;
    }
    /**
     * Gets the current elevator position
     * @return the elevator position
     */
    public double getElevatorPosition() {
        return elevatorEncoder.getPosition();
    }
    /**
     * Gets the current elevator position
     * @return the elevator position
     */
    public boolean atPose() {
        return Math.abs(getElevatorPosition() - getElevatorSetpoint()) < 0.3; // TODO! conversion factor
    }
    /**
     * Gets the current elevator position
     * @return the elevator position
     */
    public double getElevatorSetpoint() {
        return elevatorController.getSetpoint(); // TODO! conversion factor
    }

    /**
     * Sets the current position as the "bottom"
     */
    public void resetElevator() {
        elevatorEncoder.setPosition(0.0);
    }
}
