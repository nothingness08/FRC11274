// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandXboxController  m_driverController = new CommandXboxController (OIConstants.DRIVER_CONTROLLER_PORT);

  private final Pigeon m_pigeon = new Pigeon();
  private final LimelightSubsystem m_limelightTwo = new LimelightSubsystem(); //add name arguement when pull changes
  private final SwerveDriveSubsystem m_swerveDriveSubsystem = new SwerveDriveSubsystem(m_pigeon, m_limelightTwo);
  
  private final Command m_simpleAuto = new SimpleAuto(m_swerveDriveSubsystem, m_pigeon);
  private final Command m_findAprilTagAuto = new FindAprilTagAuto(m_swerveDriveSubsystem, m_pigeon, m_limelightTwo);

  Trigger xButton = m_driverController.x();

  //private final Command m_complexAuto = new ComplexAuto(m_robotDrive, m_hatchSubsystem);

  SendableChooser<Command> m_chooser = new SendableChooser<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_chooser.setDefaultOption("Simple Auto", m_simpleAuto);
    m_chooser.addOption("FindAprilTagAuto", m_findAprilTagAuto);
    SmartDashboard.putData("Auto Mode",m_chooser);

    // Set the default command for the swerve drive to be joystick control
    m_swerveDriveSubsystem.setDefaultCommand(
      new DriveWithJoystick(m_swerveDriveSubsystem, m_driverController)
    );
  }

  private void configureButtonBindings() { //gemini
    // Example of mapping a button to a command:
    xButton.onTrue(new SimpleAuto(m_swerveDriveSubsystem, m_pigeon));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_chooser.getSelected();
    //return null;
  }
}