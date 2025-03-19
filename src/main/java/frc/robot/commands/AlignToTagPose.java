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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CoDriverControl;
import frc.robot.CoDriverControl.CoDriverInput;
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
    Pose2d targetPose = null;
    Pose2d rotatedGoal = null;
    Double MaxSpeed = null;
    Double MaxAngularRate = null;
    boolean hitFirstPose = false;
    SwerveRequest.FieldCentric m_driveRequest;
    Timer timeAligning = new Timer();

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
    public AlignToTagPose(SwerveRequest.FieldCentric driveRequest, 
            Double DriveMaxSpeed, Double DriveMaxAngularRate) {
                
        m_drive = CommandSwerveDrivetrain.getInstance();
        m_photonvision = PhotonVision.getInstance();
        MaxSpeed = DriveMaxSpeed;
        MaxAngularRate = DriveMaxAngularRate;
        m_driveRequest = driveRequest;

        addRequirements(m_drive);
        addRequirements(m_photonvision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timeAligning.restart();
        hitFirstPose = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // System.err.println("ex");
        PhotonTrackedTarget target = m_photonvision.getBestTarget();
        if (target != null) {
            Optional<Pose3d> tagPose = VisionConstants.aprilTagFieldLayout.getTagPose(target.fiducialId);
            // TODO! check if reef tag
            if (tagPose.isPresent()) {
                targetPose = tagPose.get().toPose2d();
            }
        }

        if (targetPose != null) {
            double leftToRightOffset = VisionConstants.reefLeftRightOffset;
            System.out.println(m_photonvision.targetingLeftReef);
            if (!m_photonvision.targetingLeftReef) {
                leftToRightOffset *= -1;
            }

            leftToRightOffset += VisionConstants.reefAlignmentFudge; // foo fac to recenter

            double L4Offset = 0.0;

            if (CoDriverControl.getInstance().atL4()) {
                L4Offset = VisionConstants.reefL4Offset;
            }

            rotatedGoal = new Pose2d(DriveCommandConstants.x2Goal + L4Offset, DriveCommandConstants.yGoal + leftToRightOffset, new Rotation2d());
            if (hitFirstPose) {
                rotatedGoal = new Pose2d(DriveCommandConstants.xGoal + L4Offset, DriveCommandConstants.yGoal + leftToRightOffset, new Rotation2d());
            }
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

            // control flip on red ds, so invert PID outputs
            if (m_drive.m_operatorPerspectiveFlipped) {
                x_error *= -1;
                y_error *= -1;
            }

            if (Math.abs(x_error) < 0.0005) {
                x_error = 0.0;
            }
            if (Math.abs(y_error) < 0.0005) {
                y_error = 0.0;
            }

            double pidX = DriveCommandConstants.kXP + DriveCommandConstants.kXI * timeAligning.get();
            double pidY = DriveCommandConstants.kYP + DriveCommandConstants.kYI * timeAligning.get();

            m_drive.setControl(
                    m_driveRequest.withVelocityX(pidX * x_error * MaxSpeed) // Drive forward with
                                                                                                 // negative Y (forward)
                            .withVelocityY(pidY * y_error * MaxSpeed) // Drive left with negative X
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
            if (XFinished && YFinished && ThetaFinished) {
                hitFirstPose = true;
            }
            
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
        return false;
    }
}
