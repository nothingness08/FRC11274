// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public final class AutosContainer {
  private final TelemetrySubsystem m_telemetrySubsystem;
  private final SwerveDriveSubsystem m_swerveDriveSubsystem;
  private final ShooterSubsystem m_ShooterSubsystem;
  //public final Command m_simpleAuto, m_findAprilTagAuto, m_moveToTargetF, m_moveToTargetB, m_moveToTargetL, m_moveToTargetR, m_AlignToTag, m_simpleAutoTest;
  public final Command m_simpleAutoTest;
  public final Command m_test, m_climb, m_shoot;
  //SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();


  public AutosContainer(SwerveDriveSubsystem swerveDrive, TelemetrySubsystem telemetrySubsystem, ShooterSubsystem shooterSubsystem) {
    m_swerveDriveSubsystem = swerveDrive;
    m_telemetrySubsystem = telemetrySubsystem;
    m_ShooterSubsystem = shooterSubsystem;

    m_simpleAutoTest = new SimpleAutoTest(swerveDrive, shooterSubsystem);
    //m_simpleAuto = new SimpleAuto(m_swerveDriveSubsystem, m_telemetrySubsystem);
    // m_findAprilTagAuto = new FindAprilTagAuto(m_swerveDriveSubsystem, m_telemetrySubsystem);
    // m_moveToTargetF = new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(4, 11, Rotation2d.fromDegrees(0)));
    // m_moveToTargetB = new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(4, 11, Rotation2d.fromDegrees(0)));
    // m_moveToTargetL = new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(4, 11, Rotation2d.fromDegrees(0)));
    // m_moveToTargetR = new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(4, 11, Rotation2d.fromDegrees(0)));

    // m_AlignToTag = new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(4.2, 11, Rotation2d.fromDegrees(0)));
    m_test = Commands.sequence(
    Commands.sequence(
        Commands.run(() -> m_swerveDriveSubsystem.drive(new ChassisSpeeds(0, 0.4, 0), true), m_swerveDriveSubsystem).withTimeout(2),
        Commands.run(() -> m_swerveDriveSubsystem.drive(new ChassisSpeeds(0, 0, 0.5), false), m_swerveDriveSubsystem).withTimeout(0.4),
        Commands.run(() -> m_swerveDriveSubsystem.drive(new ChassisSpeeds(0, 0, 0), false), m_swerveDriveSubsystem).withTimeout(0.3),
        m_ShooterSubsystem.setDutyCycle(0.93).withTimeout(5)
    ));
    m_climb = Commands.sequence(Commands.sequence());
    m_shoot = Commands.sequence(Commands.sequence(
      Commands.run(() -> new MoveToTargetAuto(swerveDrive, telemetrySubsystem, new Pose2d(2.0,4.034536, Rotation2d.fromDegrees(0))))
    ));
    m_chooser.setDefaultOption("Simple Auto", m_test);
    m_chooser.addOption("FindAprilTagAuto", m_shoot);
    //m_chooser.addOption("FindAprilTagAuto", m_findAprilTagAuto);
    //m_chooser.addOption("Align Auto", m_AlignToTag);
    SmartDashboard.putData("Auto Chooser",m_chooser);
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
