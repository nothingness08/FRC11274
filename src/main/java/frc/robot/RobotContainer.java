// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.libs.LimelightHelpers;
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
  private final LimelightSubsystem m_limelightThree = new LimelightSubsystem("limelight");

  private final SwerveDriveSubsystem m_swerveDriveSubsystem = new SwerveDriveSubsystem(m_pigeon);
  private final TelemetrySubsystem m_telemetrySubsystem = new TelemetrySubsystem(m_swerveDriveSubsystem, m_pigeon, m_limelightThree);
  
  private ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

  private final AutosContainer m_autosContainer = new AutosContainer(m_swerveDriveSubsystem, m_telemetrySubsystem, m_shooterSubsystem);

  Trigger xButton = m_driverController.x();
  Trigger yButton = m_driverController.y();
  Trigger aButton = m_driverController.a();
  Trigger bButton = m_driverController.b();


  public RobotContainer() {
    configureButtonBindings();

    m_shooterSubsystem.setDefaultCommand(m_shooterSubsystem.setDutyCycle(0));

    m_swerveDriveSubsystem.setDefaultCommand(
      new DriveWithJoystick(m_swerveDriveSubsystem, m_driverController)
    );

    LimelightHelpers.setupPortForwardingUSB(0);
  }

  private void configureButtonBindings() {
    // xButton.whileTrue(m_shooterSubsystem.setDutyCycle(0.5));
    // yButton.whileTrue(m_shooterSubsystem.setDutyCycle(0.95));

    // m_driverController.povUp()
    //     .whileTrue(m_climberSubsystem.setDutyCycle(0.3));

    // m_driverController.povDown()
    //     .whileTrue(m_climberSubsystem.setDutyCycle(-0.3));

    // m_driverController.leftBumper()
    // .onTrue(m_climberSubsystem.setPosition(0, false));

    // m_driverController.rightBumper()
    //     .onTrue(m_climberSubsystem.setPosition(45, false));

    // m_driverController.leftTrigger()
    // .onTrue(m_climberSubsystem.setPosition(10, true));

    //m_driverController.a().onTrue(m_climberSubsystem.switchLimitsCommand());
    //m_driverController.b().onTrue(m_climberSubsystem.setCurrentPosToZeroCommand());

    // m_driverController.x().whileTrue(m_shooterSubsystem.shootSequence(-0.8, 35));
    // m_driverController.y().whileTrue(m_shooterSubsystem.shootSequence(-0.8, 40)); //8 m/s
    // m_driverController.b().whileTrue(m_shooterSubsystem.shootSequence(-0.8, 50));
    // m_driverController.rightTrigger().whileTrue(m_shooterSubsystem.setDutyCycleFeeder(-0.8));
    m_driverController.rightTrigger().onTrue(m_shooterSubsystem.setVelocity(5));
    m_driverController.a().onTrue(m_shooterSubsystem.setVelocity(60));
    m_driverController.b().onTrue(m_shooterSubsystem.setVelocity(40));
    m_driverController.y().onTrue(m_shooterSubsystem.setVelocity(30));
    m_driverController.x().onTrue(m_shooterSubsystem.setVelocity(0));


    //xButton.onTrue(m_autosContainer.m_moveToTargetL);
    //yButton.onTrue(m_autosContainer.m_moveToTargetF);
    //aButton.onTrue(m_autosContainer.m_moveToTargetB);
    //bButton.onTrue(m_autosContainer.m_moveToTargetR);
  }

  public Command getAutonomousCommand() {
    return m_autosContainer.getSelectedAuto();
  }
}