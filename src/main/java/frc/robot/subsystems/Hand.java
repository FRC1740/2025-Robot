package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;
import frc.robot.constants.CanIds;
import frc.robot.constants.HandConstants;

public class Hand extends SubsystemBase {
    SparkBase wrist = null; // rotates to pos
    PIDController wristController = null;
    AbsoluteEncoder wristEncoder = null;

    SparkBase linearActuator = null; // intake / outtakes, no pid just run or no run
    ShuffleboardTab handTab = null;
    DigitalInput hasCoral = new DigitalInput(0);

    Telemetry m_telemetry = null;
    
    public Hand(Telemetry telemetry) {
        m_telemetry = telemetry;
        wrist = new SparkMax(CanIds.wristCanId, MotorType.kBrushed);
        SparkMaxConfig wristConfig = new SparkMaxConfig();
        
        wristConfig.smartCurrentLimit(HandConstants.wristCurrentLimit);
        SoftLimitConfig wristSoftlimit = new SoftLimitConfig();
        wristSoftlimit.reverseSoftLimit(HandConstants.minimumWristAngle);
        wristSoftlimit.reverseSoftLimitEnabled(false);
        wristSoftlimit.forwardSoftLimit(HandConstants.maximumWristAngle);
        wristSoftlimit.forwardSoftLimitEnabled(false);

        wristConfig.softLimit.apply(wristSoftlimit);

        wristConfig.idleMode(IdleMode.kBrake);

        AbsoluteEncoderConfig absoluteEncoderConfig = new AbsoluteEncoderConfig();
        absoluteEncoderConfig.inverted(true);
        wristConfig.absoluteEncoder.apply(absoluteEncoderConfig);

        wristController = new PIDController(
            HandConstants.kWristP, HandConstants.kWristI, HandConstants.kWristD);
        wristController.setSetpoint(0.1);
        wristController.setIZone(0.2);
        wristController.setTolerance(0.01);
        
        wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        wristEncoder = wrist.getAbsoluteEncoder();


        linearActuator = new SparkMax(CanIds.linearActuatorCanId, MotorType.kBrushed);

        SparkMaxConfig linearActuatorConfig = new SparkMaxConfig();

        linearActuatorConfig.smartCurrentLimit(HandConstants.linearActuatorCurrentLimit);

        linearActuator.configure(linearActuatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // handTab = Shuffleboard.getTab("hand");
        // handTab.addFloat("hand position", () -> (float)getWristAngle());
        // handTab.addFloat("hand setpoint", () -> (float)getWristSetpoint());
        // handTab.addFloat("hand current", () -> (float)wrist.getOutputCurrent());


        // handTab.addFloat("linear actuator current", () -> (float)linearActuator.getOutputCurrent());
        // handTab.addBoolean("hasCoral", () -> hasCoral());
    }

    @Override
    public void periodic() {
        m_telemetry.telemeterizeWrist(getWristAngle(), wrist.getOutputCurrent());
    }

    /**
     * Runs one step to optimize the PID and get new outputs for the inputs
     */
    public void seekPosition() {
        double output = wristController.calculate(getWristAngle());
        // if (output < 0.0) { // gravity ff
        //     output -= 0.2;
        // }
        if ((getWristAngle() < HandConstants.minimumWristAngle && output < 0.0) ||
            (getWristAngle() > HandConstants.maximumWristAngle && output > 0.0)) {
            wrist.set(0.0);
            // System.out.println("out");
        }else {
            wrist.set(output);
                
            // System.out.println(output);
        }
    }

    /**
     * Sets setpoint for wrist to go to specified radian angle
     * @param angle
     */
    public void setWristSetpoint(double angle) {
        wristController.setSetpoint(angle);
    }

    public double getWristAngle() {
        double angle = wristEncoder.getPosition();
        if (angle > .9) { // wrap protection
            angle = angle - 1.0;
        }
        return angle;
    }

    public double getWristSetpoint() {
        return wristController.getSetpoint();
    }

    public boolean atPose() {
        // return true;
        return Math.abs(wristController.getSetpoint() - wristEncoder.getPosition()) < 0.1;
    }

    /**
     * Runs linear actuator in
     */
    public void intake() {
        linearActuator.set(-0.5);
    }
    /**
     * Runs linear actuator in
     */
    public double getLinearActuatorCurrent() {
        return linearActuator.getOutputCurrent();
    }
    /**
     * Runs linear actuator in
     */
    public void stop() {
        linearActuator.set(0.0);
    }
    /**
     * Runs linear actuator out
     */
    public void score() {
        linearActuator.set(1.0);
    }
    /**
     * checks sensor
     * @return 
     */
    public boolean hasCoral() {
        return !hasCoral.get(); // it's flipped
    }
}
