// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  public final Command m_simpleAuto, m_findAprilTagAuto, m_moveToTargetF, m_moveToTargetB, m_moveToTargetL, m_moveToTargetR, m_AlignToTag;
  
  //SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final SendableChooser<Command> autoChooser;


  public AutosContainer(SwerveDriveSubsystem swerveDrive, TelemetrySubsystem telemetrySubsystem) {
    m_swerveDriveSubsystem = swerveDrive;
    m_telemetrySubsystem = telemetrySubsystem;

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);


    m_simpleAuto = new SimpleAuto(m_swerveDriveSubsystem, m_telemetrySubsystem);
    m_findAprilTagAuto = new FindAprilTagAuto(m_swerveDriveSubsystem, m_telemetrySubsystem);
    m_moveToTargetF = new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(4, 11, Rotation2d.fromDegrees(0)));
    m_moveToTargetB = new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(4, 11, Rotation2d.fromDegrees(0)));
    m_moveToTargetL = new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(4, 11, Rotation2d.fromDegrees(0)));
    m_moveToTargetR = new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(4, 11, Rotation2d.fromDegrees(0)));

    m_AlignToTag = new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new Pose2d(4.2, 11, Rotation2d.fromDegrees(0)));

    //m_chooser.setDefaultOption("Simple Auto", m_simpleAuto);
    //m_chooser.addOption("FindAprilTagAuto", m_findAprilTagAuto);
    //m_chooser.addOption("Align Auto", m_AlignToTag);
    //SmartDashboard.putData("Auto Mode",m_chooser);
  }

  public Command getSelectedAuto(){
    return autoChooser.getSelected();

    //return m_chooser.getSelected();
  }
}
