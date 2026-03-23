// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import frc.robot.util.HolonomicWaypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.TelemetrySubsystem;
import frc.robot.util.HolonomicWaypoint;

public final class AutosContainer {
  private final TelemetrySubsystem m_telemetrySubsystem;
  private final SwerveDriveSubsystem m_swerveDriveSubsystem;
  private final ShooterSubsystem m_ShooterSubsystem;
  private final IntakeSubsystem m_IntakeSubsystem;
  private final ClimberSubsystem m_ClimberSubsystem;
  public final Command m_moveF, m_moveB, m_wayPointAuto;
  public final Command m_BlueLeftAuto, m_BlueCenterAuto, m_BlueRightAuto, m_RedLeftAuto, m_RedCenterAuto, m_RedRightAuto;
  //SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();


  public AutosContainer(SwerveDriveSubsystem swerveDrive, TelemetrySubsystem telemetrySubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, ClimberSubsystem climberSubsystem) {
    m_swerveDriveSubsystem = swerveDrive;
    m_telemetrySubsystem = telemetrySubsystem;
    m_ShooterSubsystem = shooterSubsystem;
    m_IntakeSubsystem = intakeSubsystem;
    m_ClimberSubsystem = climberSubsystem;

    m_moveB = new MoveToTargetAuto(
      swerveDrive, 
      telemetrySubsystem, 
      new Pose2d(1.2, 3.73, Rotation2d.fromDegrees(180))
    );
    m_moveF = new MoveToTargetAuto(
      swerveDrive, 
      telemetrySubsystem, 
      new Pose2d(2.5, 3.73, Rotation2d.fromDegrees(0))
    );
    
    List<HolonomicWaypoint> points = List.of(
      new HolonomicWaypoint(new Pose2d(1, 3.7, Rotation2d.fromDegrees(-45)), Rotation2d.fromDegrees(0), 0.5, new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(5.0))),
      new HolonomicWaypoint(new Pose2d(2.5, 3, Rotation2d.fromDegrees(90)), Rotation2d.fromDegrees(0), 0.5, new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(5.0))),
      new HolonomicWaypoint(new Pose2d(2.5, 4.4, Rotation2d.fromDegrees(-135)), Rotation2d.fromDegrees(0), 0.5, new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(5.0))),
      new HolonomicWaypoint(new Pose2d(1, 3.7, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0), 0.0, new Pose2d(0.02, 0.02, Rotation2d.fromDegrees(2.0)))
    );
    m_wayPointAuto = new FollowWaypointsAuto(
      m_swerveDriveSubsystem, m_telemetrySubsystem, points
    );
    // m_moveToTargetChain = new SequentialCommandGroup(
    //   new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(2,2, Rotation2d.fromDegrees(0))),

    //   new ParallelDeadlineGroup(
    //       new SequentialCommandGroup(
    //           new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(2,2, Rotation2d.fromDegrees(0))),
    //           new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(2,2, Rotation2d.fromDegrees(0)))
    //       ),
    //       m_IntakeSubsystem.deployAndRun() 
    //   ),

    //   new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(2,2, Rotation2d.fromDegrees(0))),

    //   // 4. Shoot
    //   Commands.deferredProxy(() -> 
    //       m_ShooterSubsystem.shootSequence(
    //           ShooterConstants.FEEDER_SPEED, 
    //           m_telemetrySubsystem.getRPSForPosition()
    //       )
    //   )
    //);
    double BlueAllianceXPose = 3.647;
    double RedAllianceXPose = 12.893;
    double centerYPose = 4.034, leftYPose = 1.43, rightYPose =6.638;
    double shootTime = 3.5, driveTime = 1.2, driveSpeed = -1;

    //m_telemetrySubsystem.resetPose(new Pose2d(RedAllianceXPose, centerYPose, Rotation2d.fromDegrees(180)));
    m_BlueLeftAuto   = shooterAutoFactory(new Pose2d(BlueAllianceXPose, leftYPose, Rotation2d.fromDegrees(0)), driveSpeed, driveTime, shootTime);
    m_BlueCenterAuto = shooterAutoFactory(new Pose2d(BlueAllianceXPose, centerYPose, Rotation2d.fromDegrees(0)), driveSpeed, driveTime, shootTime);
    m_BlueRightAuto  = shooterAutoFactory(new Pose2d(BlueAllianceXPose, rightYPose, Rotation2d.fromDegrees(0)), driveSpeed, driveTime, shootTime);

    m_RedLeftAuto    = shooterAutoFactory(new Pose2d(RedAllianceXPose, leftYPose, Rotation2d.fromDegrees(180)), driveSpeed, driveTime, shootTime);
    m_RedCenterAuto  = shooterAutoFactory(new Pose2d(RedAllianceXPose, centerYPose, Rotation2d.fromDegrees(180)), driveSpeed, driveTime, shootTime);
    m_RedRightAuto   = shooterAutoFactory(new Pose2d(RedAllianceXPose, rightYPose, Rotation2d.fromDegrees(180)), driveSpeed, driveTime, shootTime);


    m_chooser.setDefaultOption("move to target", m_moveF);
    // Default Option

    // Blue Alliance Options
    m_chooser.addOption("Blue Center Auto", m_BlueCenterAuto);
    m_chooser.addOption("Blue Left Auto", m_BlueLeftAuto);
    m_chooser.addOption("Blue Right Auto", m_BlueRightAuto);

    // Red Alliance Options
    m_chooser.addOption("Red Left Auto", m_RedLeftAuto);
    m_chooser.addOption("Red Center Auto", m_RedCenterAuto);
    m_chooser.addOption("Red Right Auto", m_RedRightAuto);

    // Waypoint Option (if still using it)
    //m_chooser.addOption("Waypoint Auto", m_wayPointAuto);

    SmartDashboard.putData("Auto Chooser", m_chooser);
    //m_chooser.addOption("Align Auto", m_AlignToTag);
    SmartDashboard.putData("Auto Chooser",m_chooser);
  }

  private Command shooterAutoFactory(Pose2d startPose, double speed, double driveTime, double shootTime) {
    return Commands.sequence(
      // 1. Reset Pose
      Commands.runOnce(() -> m_telemetrySubsystem.resetPose(startPose)),

      // 2. Drive AND Align
      new AlignToHubAuto(m_swerveDriveSubsystem, m_telemetrySubsystem)
          .deadlineWith(
              Commands.run(() -> m_swerveDriveSubsystem.drive(new ChassisSpeeds(0.0, speed, 0), true))
          )
          .withTimeout(driveTime),

      // 3. Stop
      Commands.runOnce(() -> m_swerveDriveSubsystem.drive(new ChassisSpeeds(0, 0, 0), true), m_swerveDriveSubsystem),

      // 4. NEW: Stay Aligned for 0.5 seconds to settle the robot
      new AlignToHubAuto(m_swerveDriveSubsystem, m_telemetrySubsystem)
          .withTimeout(0.5),

      // 5. Align AND Shoot
      new AlignToHubAuto(m_swerveDriveSubsystem, m_telemetrySubsystem)
          .deadlineWith(
              Commands.deferredProxy(() -> m_ShooterSubsystem.shootSequence(
                  ShooterConstants.FEEDER_SPEED, 
                  m_telemetrySubsystem.getRPSForPosition()
              ))
          ).withTimeout(shootTime)      
    );
}


  public Command getSelectedAuto(){
    return m_chooser.getSelected();
  }

  //   private final Command m_complexAuto = new SequentialCommandGroup(
  //   new SimpleAuto(m_swerveDriveSubsystem, m_telemetrySubsystem),
  //   new WaitCommand(1.0), // Optional: wait 1 second between tasks
  //   new FindAprilTagAuto(m_swerveDriveSubsystem, m_telemetrySubsystem)
  // );
}
