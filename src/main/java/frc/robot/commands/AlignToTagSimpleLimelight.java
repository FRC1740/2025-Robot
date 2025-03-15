// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToTagSimpleLimelight extends Command {
    CommandSwerveDrivetrain m_drive;

    double x_error;
    double y_error;
    double theta_error;
    boolean isLeftReef; // A C E etc
    SwerveRequest.RobotCentricFacingAngle m_driveRequest;
    double MaxSpeed;
    double MaxAngularRate;
    boolean foundTag = false;
    Rotation2d targetRot;

    NetworkTable DriveTrainTable = NetworkTableInstance.getDefault().getTable("DriveTrain");

    // Pose data Publisher
    StructArrayPublisher<Pose2d> PosePublisher = DriveTrainTable
            .getStructArrayTopic("Target Pose", Pose2d.struct).publish();

    public static double normalizeAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    /**
     * Creates a new Command that aligns the robot angle to an apriltag using the
     * Limelight.
     * <br>
     * </br>
     * This command <b>DOES DRIVE</b>
     */
    public AlignToTagSimpleLimelight(boolean leftReef, SwerveRequest.RobotCentricFacingAngle driveRequest, 
            Double DriveMaxSpeed, Double DriveMaxAngularRate, CommandXboxController joystick) {
                
        m_drive = CommandSwerveDrivetrain.getInstance();
        MaxSpeed = DriveMaxSpeed;
        MaxAngularRate = DriveMaxAngularRate;
        m_driveRequest = driveRequest;

        this.isLeftReef = leftReef;

        addRequirements(m_drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // timeRunning.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double[] botPoseTarget = LimelightHelpers.getBotPose_TargetSpace("");
        if (!foundTag) {
            // 5 is yaw
            m_driveRequest.TargetDirection = new Rotation2d(m_drive.getState().Pose.getRotation().getRadians() + (botPoseTarget[5]));
            foundTag = true;
        }else {
            if (isLeftReef) {
                x_error += 1.0;
            }else {
                x_error -= 1.0;
            }

            m_drive.setControl(
                m_driveRequest
                    .withVelocityX(x_error * 0.03)
                    .withTargetDirection(targetRot)
            );
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return XFinished && YFinished && ThetaFinished;
        return false; // strafe
    }
}
