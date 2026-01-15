// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
  
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public AutosContainer(SwerveDriveSubsystem swerveDrive, TelemetrySubsystem telemetrySubsystem) {
    m_swerveDriveSubsystem = swerveDrive;
    m_telemetrySubsystem = telemetrySubsystem;

    m_simpleAuto = new SimpleAuto(m_swerveDriveSubsystem, m_telemetrySubsystem);
    m_findAprilTagAuto = new FindAprilTagAuto(m_swerveDriveSubsystem, m_telemetrySubsystem);
    m_moveToTargetF = new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new double[]{0, -1, 0});
    m_moveToTargetB = new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new double[]{0, -1.5, 0});
    m_moveToTargetL = new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new double[]{-0.25, -1.1, 0});
    m_moveToTargetR = new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new double[]{0.25, -1.1, 0});

    m_AlignToTag = new MoveToTargetAuto(m_swerveDriveSubsystem, m_telemetrySubsystem, new double[]{0, -1.2, 0});

    m_chooser.setDefaultOption("Simple Auto", m_simpleAuto);
    m_chooser.addOption("FindAprilTagAuto", m_findAprilTagAuto);
    m_chooser.addOption("Align Auto", m_AlignToTag);
    SmartDashboard.putData("Auto Mode",m_chooser);
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
