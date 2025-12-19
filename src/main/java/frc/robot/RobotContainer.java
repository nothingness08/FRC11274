// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandXboxController  m_driverController = new CommandXboxController (OIConstants.DRIVER_CONTROLLER_PORT);

  private final Pigeon m_pigeon = new Pigeon();
  private final LimelightSubsystem m_limelightTwo = new LimelightSubsystem("limelight-two");

  private final SwerveDriveSubsystem m_swerveDriveSubsystem = new SwerveDriveSubsystem(m_pigeon);
  private final TelemetrySubsystem m_telemetrySubsystem = new TelemetrySubsystem(m_swerveDriveSubsystem, m_pigeon, m_limelightTwo);
  
  private final AutosContainer m_autosContainer = new AutosContainer(m_swerveDriveSubsystem, m_telemetrySubsystem);
  
  Trigger xButton = m_driverController.x();
  Trigger yButton = m_driverController.y();
  Trigger aButton = m_driverController.a();
  Trigger bButton = m_driverController.b();


  public RobotContainer() {
    configureButtonBindings();

    m_swerveDriveSubsystem.setDefaultCommand(
      new DriveWithJoystick(m_swerveDriveSubsystem, m_driverController)
    );
  }

  private void configureButtonBindings() {
    xButton.onTrue(m_autosContainer.m_moveToTargetL);
    yButton.onTrue(m_autosContainer.m_moveToTargetF);
    aButton.onTrue(m_autosContainer.m_moveToTargetB);
    bButton.onTrue(m_autosContainer.m_moveToTargetR);
  }

  public Command getAutonomousCommand() {
    return m_autosContainer.getSelectedAuto();
  }
}