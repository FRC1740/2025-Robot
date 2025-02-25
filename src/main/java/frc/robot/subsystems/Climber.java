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
import frc.robot.constants.CanIds;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.HandConstants;

public class Climber extends SubsystemBase {
    SparkBase climber = null; // rotates to pos
    
    public Climber() {
        climber = new SparkMax(CanIds.climberCanId, MotorType.kBrushless);
        SparkMaxConfig wristConfig = new SparkMaxConfig();
        
        wristConfig.smartCurrentLimit(ClimberConstants.climberCurrentLimit);

        wristConfig.idleMode(IdleMode.kBrake);
        
        climber.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Runs climber out
     */
    public void unclimb() {
        climber.set(1.0);
    }
    /**
     * Stops climbing
     */
    public void stop() {
        climber.set(0.0);
    }
    /**
     * Runs climber in
     */
    public void climb() {
        climber.set(-1.0);
    }
}
