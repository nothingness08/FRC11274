// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutosContainer;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.libs.LimelightHelpers;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.TelemetrySubsystem;


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
  private IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  private final AutosContainer m_autosContainer = new AutosContainer(m_swerveDriveSubsystem, m_telemetrySubsystem, m_shooterSubsystem);

  Trigger xButton = m_driverController.x();
  Trigger yButton = m_driverController.y();
  Trigger aButton = m_driverController.a();
  Trigger bButton = m_driverController.b();


  public RobotContainer() {
    configureButtonBindings();

    m_shooterSubsystem.setDefaultCommand(m_shooterSubsystem.setDutyCycle(0));

    m_swerveDriveSubsystem.setDefaultCommand(
  new DriveWithJoystick( //
    m_swerveDriveSubsystem, 
    m_driverController, 
    m_telemetrySubsystem,
    () -> m_driverController.getHID().getXButton(), //Hub Align, this overrides
    () -> m_driverController.getHID().getYButton() //Joystick Align
  ));
    // 

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
    // .onTrue(m_climberSubsystem.setPosition(0, true));

    // m_driverController.a().onTrue(m_climberSubsystem.switchLimitsCommand());
    // m_driverController.b().onTrue(m_climberSubsystem.setCurrentPosToZeroCommand());

    // m_driverController.x().whileTrue(m_shooterSubsystem.shootSequence(-0.8, 35));
    // m_driverController.y().whileTrue(m_shooterSubsystem.shootSequence(-0.8, 40)); //8 m/s
    // m_driverController.b().whileTrue(m_shooterSubsystem.shootSequence(-0.8, 50));
    // m_driverController.rightTrigger().whileTrue(m_shooterSubsystem.setDutyCycleFeeder(-0.8));


    // m_driverController.a().onTrue(m_shooterSubsystem.shootSequence(-0.6, 60));
    //m_driverController.b().onTrue(m_shooterSubsystem.shootSequence(-0.6, 40));
    m_driverController.y().onTrue(Commands.deferredProxy(() -> m_shooterSubsystem.shootSequence(-0.9, m_telemetrySubsystem.getRPSForPosition())));
    m_driverController.b().onTrue(m_shooterSubsystem.setDutyCycle(0));


    //xButton.onTrue(m_autosContainer.m_moveToTargetL);
    //yButton.onTrue(m_autosContainer.m_moveF);
    //aButton.onTrue(m_autosContainer.m_moveB);
    //bButton.onTrue(m_autosContainer.m_moveToTargetR);

    // m_driverController.y().onTrue(m_intakeSubsystem.setPosition(0.2));
    // m_driverController.a().onTrue(m_intakeSubsystem.setPosition(-0.07));
    // m_driverController.b().onTrue(m_intakeSubsystem.setCurrentPosToZeroCommand());

    // m_driverController.povUp()
    //     .whileTrue(m_intakeSubsystem.setDutyCycle(0.08));

    // m_driverController.povDown()
    //     .whileTrue(m_intakeSubsystem.setDutyCycle(-0.08));
    // }

    // m_driverController.povUp()
    //     .onTrue(m_shooterSubsystem.runOnce(() -> m_shooterSubsystem.increaseSpeed()));

    // m_driverController.povDown()
    //     .onTrue(m_shooterSubsystem.runOnce(() -> m_shooterSubsystem.decreaseSpeed()));
  
    
  }

  public Command getAutonomousCommand() {
    return m_autosContainer.getSelectedAuto();
  }
}