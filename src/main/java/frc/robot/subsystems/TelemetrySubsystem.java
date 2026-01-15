// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.libs.LimelightHelpers;

public class TelemetrySubsystem extends SubsystemBase {
  private LimelightSubsystem m_limelightTwo;
  private final SwerveDrivePoseEstimator m_poseEstimator;
  private Pigeon m_pigeon;
  private SwerveDriveSubsystem m_swerveDriveSubsystem;

  public TelemetrySubsystem(SwerveDriveSubsystem swerveDriveSubsystem, Pigeon pigeon, LimelightSubsystem limelightTwo) {
    m_swerveDriveSubsystem = swerveDriveSubsystem;
    m_pigeon = pigeon;
    m_limelightTwo = limelightTwo;
    m_poseEstimator = new SwerveDrivePoseEstimator(
      SwerveDriveConstants.KINEMATICS,
      Rotation2d.fromDegrees(m_pigeon.getYaw()),
      m_swerveDriveSubsystem.getModulePositions(),
      new Pose2d(0.0,0.0, Rotation2d.fromDegrees(0.0))
    );
  }

  public double[] getPoseEstimateWithTag(){
    double[] pose = new double[3];
    pose[0] = -(m_poseEstimator.getEstimatedPosition().getY() - AprilTagConstants.TAG_Y);
    pose[1] = m_poseEstimator.getEstimatedPosition().getX() - AprilTagConstants.TAG_X;
    pose[2] = m_poseEstimator.getEstimatedPosition().getRotation().getDegrees();
    return pose;
  }

  public double getPigeonYaw(){
    return m_pigeon.getYaw();
  }

  public boolean getLimelightTV(){
    return m_limelightTwo.getTV();
  } 

  public double[] getBotPose_TargetSpace(){
    return LimelightHelpers.getBotPose_TargetSpace(m_limelightTwo.getLimelightName());
  }

  @Override
  public void periodic() {
    m_poseEstimator.update(
      Rotation2d.fromDegrees(m_pigeon.getYaw()),
      m_swerveDriveSubsystem.getModulePositions()
    );
    boolean doRejectUpdate = false;
    LimelightHelpers.SetRobotOrientation(m_limelightTwo.getLimelightName(), 
      m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 
      0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = 
      LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_limelightTwo.getLimelightName());

    if (Math.abs(m_pigeon.getRate()) > 360) {
      doRejectUpdate = true;
    }
    if (mt2 != null && mt2.tagCount == 0) {
      doRejectUpdate = true;
    }

    if (!doRejectUpdate && mt2 != null) {
      // Trust vision for X/Y (0.7m error), ignore vision for rotation (999999 error)
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
      m_poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
    }

    double poseEstimate[] = getPoseEstimateWithTag();
    SmartDashboard.putNumber("Estimated X", poseEstimate[0]);
    SmartDashboard.putNumber("Estimated Y", poseEstimate[1]);

    SmartDashboard.putNumber("Estimated X in", poseEstimate[0]*39.37);
    SmartDashboard.putNumber("Estimated Y in", poseEstimate[1]*39.37);

    // double botPoseInTargetSpace[] = getBotPose_TargetSpace();
    // if (botPoseInTargetSpace != null || botPoseInTargetSpace.length != 0) {
    //   SmartDashboard.putNumber("Target Space X (m)", botPoseInTargetSpace[0]);
    //   SmartDashboard.putNumber("Target Space Y (m)", botPoseInTargetSpace[1]);
    //   SmartDashboard.putNumber("Target Space Z (m)", botPoseInTargetSpace[2]);
    //   SmartDashboard.putNumber("Target Space Yaw (deg)", botPoseInTargetSpace[4]);
    // }
  }
}
