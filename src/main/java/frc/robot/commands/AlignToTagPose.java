// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveCommandConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonVision;

public class AlignToTagPose extends Command {
    CommandSwerveDrivetrain m_drive;
    private PhotonVision m_photonvision;

    double x_error;
    double y_error;
    double theta_error;
    double angleToTag;
    Transform3d distanceToTag;
    boolean XFinished;
    boolean YFinished;
    boolean ThetaFinished;
    boolean isLeftReef; // A C E etc
    Pose2d targetPose = null;
    Double MaxSpeed = null;
    Double MaxAngularRate = null;
    SwerveRequest.FieldCentric m_driveRequest;

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
    public AlignToTagPose(boolean leftReef, CommandSwerveDrivetrain drive, SwerveRequest.FieldCentric driveRequest,
            PhotonVision photonvision, Double DriveMaxSpeed, Double DriveMaxAngularRate) {
        m_drive = drive;
        m_photonvision = photonvision;
        MaxSpeed = DriveMaxSpeed;
        MaxAngularRate = DriveMaxAngularRate;
        m_driveRequest = driveRequest;

        this.isLeftReef = leftReef;

        addRequirements(m_drive);
        addRequirements(m_photonvision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.err.println("ex");
        PhotonTrackedTarget target = m_photonvision.getBestTarget();
        if (target != null) {
            Optional<Pose3d> tagPose = VisionConstants.aprilTagFieldLayout.getTagPose(target.fiducialId);
            // TODO! check if reef tag
            if (tagPose.isPresent()) {
                targetPose = tagPose.get().toPose2d();
            }
        }

        if (targetPose != null) {
            distanceToTag = target.bestCameraToTarget; // getTranslationToAprilTag may be incorrect

            double leftToRightOffset = VisionConstants.reefLeftRightOffset;
            if (!isLeftReef) {
                leftToRightOffset *= -1;
            }

            Pose2d rotatedGoal = new Pose2d(DriveCommandConstants.xGoal, DriveCommandConstants.yGoal + leftToRightOffset, new Rotation2d());
            rotatedGoal = rotatedGoal.rotateBy(targetPose.getRotation()); // Rotate the goal to account for rotated tags

            rotatedGoal = new Pose2d(
                    (targetPose.getX() + rotatedGoal.getX()), // apply target offsets
                    (targetPose.getY() + rotatedGoal.getY()),
                    targetPose.getRotation().plus(new Rotation2d(Math.PI))); // normal of the tag is flipped from robot
                                                                             // target
            PosePublisher.set(new Pose2d[] { rotatedGoal });

            angleToTag = -normalizeAngle(
                    rotatedGoal.getRotation().getRadians() - m_drive.getState().Pose.getRotation().getRadians());

            x_error = -(m_drive.getState().Pose.getX() - rotatedGoal.getX()); // f - b error
            y_error = -(m_drive.getState().Pose.getY() - rotatedGoal.getY()); // l - r error
            // flip because mechs on "back"
            theta_error = normalizeAngle(angleToTag);
            System.out.println("angle: " + angleToTag);
            System.out.println("x: " + x_error);
            System.out.println("y: " + y_error);
            // x_error = 0;

            // control flip on red ds, so invert PID outputs
            if (m_drive.m_hasAppliedOperatorPerspective) {
                x_error *= -1;
                y_error *= -1;
            }

            if (Math.abs(x_error) < 0.05) {
                x_error = 0.0;
            }
            if (Math.abs(y_error) < 0.05) {
                y_error = 0.0;
            }

            m_drive.setControl(
                    m_driveRequest.withVelocityX(DriveCommandConstants.kXP * x_error * MaxSpeed) // Drive forward with
                                                                                                 // negative Y (forward)
                            .withVelocityY(DriveCommandConstants.kYP * y_error * MaxSpeed) // Drive left with negative X
                                                                                           // (left)
                            .withRotationalRate(-DriveCommandConstants.kThetaP * theta_error * MaxAngularRate) // Drive
                                                                                                               // counterclockwise
                                                                                                               // with
                                                                                                               // negative
                                                                                                               // X
                                                                                                               // (left)
            );

            XFinished = Math.abs(x_error) < DriveCommandConstants.kXToleranceMeters;
            YFinished = Math.abs(y_error) < DriveCommandConstants.kYToleranceMeters;
            ThetaFinished = Math.abs(theta_error) < DriveCommandConstants.kThetaToleranceRadians;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return XFinished && YFinished && ThetaFinished;
    }
}
