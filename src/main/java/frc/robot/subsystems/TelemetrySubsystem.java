// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.TelemetryConstants;
import frc.robot.libs.LimelightHelpers;

public class TelemetrySubsystem extends SubsystemBase {
  private LimelightSubsystem m_limelight;
  private final SwerveDrivePoseEstimator m_poseEstimator;
  private Pigeon m_pigeon;
  private SwerveDriveSubsystem m_swerveDriveSubsystem;
  private InterpolatingDoubleTreeMap interpolatingDoubleTreeMap = new InterpolatingDoubleTreeMap();

  final Field2d field = new Field2d();
private final PIDController pidController = new PIDController(0.07, 0, 0);
  public TelemetrySubsystem(SwerveDriveSubsystem swerveDriveSubsystem, Pigeon pigeon, LimelightSubsystem limelight) {
    m_swerveDriveSubsystem = swerveDriveSubsystem;
    m_pigeon = pigeon;
    m_limelight = limelight;
    m_poseEstimator = new SwerveDrivePoseEstimator(
      SwerveDriveConstants.KINEMATICS,
      Rotation2d.fromDegrees(m_pigeon.getYaw()),
      m_swerveDriveSubsystem.getModulePositions(),
      new Pose2d(0.0,0.0, Rotation2d.fromDegrees(0.0))
    );
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      // Mirror the starting pose to the red side
      // Field is 16.54m long, so red origin X ≈ 16.54 - 0.0
      resetPose(new Pose2d(
          16.54,  // red side X (field width in meters)
          8.02,   // mid-field Y, adjust to your actual starting position
          Rotation2d.fromDegrees(180.0) // facing blue hub from red side
      ));
    }

    for(double[] pair : TelemetryConstants.dataPoints){
      interpolatingDoubleTreeMap.put(pair[0], pair[1]);
    }

    SmartDashboard.putData(field);
    // RobotConfig config;
    // try {
    //   config = RobotConfig.fromGUISettings();
    // } catch (Exception e) {
    //   e.printStackTrace();
    //   throw new RuntimeException("Failed to load PathPlanner RobotConfig");
    // }

    // AutoBuilder.configure(
    //   this::getPose,
    //   this::resetPose,
    //   m_swerveDriveSubsystem::getRobotRelativeSpeeds,
    //   (speeds, feedforwards) -> m_swerveDriveSubsystem.drive(speeds, false), 
    //   new PPHolonomicDriveController(
    //     new PIDConstants(5.0, 0.0, 0.0), 
    //     new PIDConstants(5.0, 0.0, 0.0) 
    //   ),
    //   config,
    //   () -> {
    //     var alliance = DriverStation.getAlliance();
    //     if (alliance.isPresent()) {
    //       return alliance.get() == DriverStation.Alliance.Red;
    //     }
    //     return false;
    //   },
    //   m_swerveDriveSubsystem 
    // );

  }

  public Pose2d getPose(){
    Pose2d pose = new Pose2d(
      m_poseEstimator.getEstimatedPosition().getX(),
      m_poseEstimator.getEstimatedPosition().getY(),
      m_poseEstimator.getEstimatedPosition().getRotation()
    );
    return pose;
  }

  public double getPigeonYaw(){
    return m_pigeon.getYaw();
  }

  public boolean getLimelightTV(){
    return m_limelight.getTV();
  } 

  public double[] getBotPose_TargetSpace(){
    return LimelightHelpers.getBotPose_TargetSpace(m_limelight.getLimelightName());
  }

  public void resetPose(Pose2d newPose){
    m_poseEstimator.resetPosition(
      Rotation2d.fromDegrees(m_pigeon.getYaw()),
      m_swerveDriveSubsystem.getModulePositions(),
      newPose
    );
  }

  public double getRPSForPosition(){
    Translation2d currentPosition = getPose().getTranslation();
    double distance;
    if(getAlliance() == DriverStation.Alliance.Blue){
      distance = currentPosition.getDistance(TelemetryConstants.BLUE_HUB);
    }
    else{
      distance = currentPosition.getDistance(TelemetryConstants.RED_HUB);
    }
    return interpolatingDoubleTreeMap.get(distance);
  }

  public Rotation2d targetRotationToHub(){
    Pose2d robotPos = getPose();
    double deltaX, deltaY;
    if(getAlliance() == DriverStation.Alliance.Blue){
      deltaX = TelemetryConstants.BLUE_HUB.getX() - robotPos.getX();
      deltaY = TelemetryConstants.BLUE_HUB.getY() - robotPos.getY();
    }
    else{
      deltaX = TelemetryConstants.RED_HUB.getX() - robotPos.getX();
      deltaY = TelemetryConstants.RED_HUB.getY() - robotPos.getY();
    }
    

    double angleRadians = Math.atan2(deltaY, deltaX);
    Rotation2d rot = Rotation2d.fromRadians(angleRadians);

    var alliance = DriverStation.getAlliance();
    if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red && angleRadians < 0) {
      rot = Rotation2d.fromRadians(angleRadians);
    }
    else if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red && angleRadians > 0){
      rot = Rotation2d.fromRadians(angleRadians);
    }

    return rot;
  }

  public double getDistance(){
    Pose2d robotPos = getPose();
    double deltaX, deltaY;
    if(getAlliance() == DriverStation.Alliance.Blue){
      deltaX = TelemetryConstants.BLUE_HUB.getX() - robotPos.getX();
      deltaY = TelemetryConstants.BLUE_HUB.getY() - robotPos.getY();
    }
    else{   
      deltaX = TelemetryConstants.RED_HUB.getX() - robotPos.getX();
      deltaY = TelemetryConstants.RED_HUB.getY() - robotPos.getY();
    }

    return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
  }

  public DriverStation.Alliance getAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get();
    }
    return DriverStation.Alliance.Blue;
  }

  @Override
  public void periodic() {
    field.setRobotPose(getPose());

    m_poseEstimator.update(
      Rotation2d.fromDegrees(m_pigeon.getYaw()),
      m_swerveDriveSubsystem.getModulePositions()
    );
    boolean doRejectUpdate = false;
    LimelightHelpers.SetRobotOrientation(m_limelight.getLimelightName(), 
      m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 
      0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = 
      LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_limelight.getLimelightName());

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

    Pose2d poseEstimate = getPose();
    SmartDashboard.putNumber("Estimated X", poseEstimate.getX());
    SmartDashboard.putNumber("Estimated Y", poseEstimate.getY());
    SmartDashboard.putNumber("Estimated Rotation", poseEstimate.getRotation().getDegrees());

    SmartDashboard.putNumber("Estimated X in", poseEstimate.getX()*39.37);
    SmartDashboard.putNumber("Estimated Y in", poseEstimate.getY()*39.37);

    SmartDashboard.putNumber("rot to hub", targetRotationToHub().getDegrees());
    SmartDashboard.putNumber("distance to hub", getDistance());
    SmartDashboard.putNumber("Pigean rot", m_pigeon.getYaw());
    double currentDeg = getPose().getRotation().getDegrees();
      double targetDeg = targetRotationToHub().getDegrees();
      
      double error = targetDeg - currentDeg;
      error = Math.IEEEremainder(error, 360.0);
      
    SmartDashboard.putNumber("Testing rot", -pidController.calculate(0, error));
  }
}
