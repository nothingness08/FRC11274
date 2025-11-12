// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.CIMSwerveDriveSubsystem;

/** An example command that uses an example subsystem. */
public class DriveWithJoystick extends Command {
  private final CIMSwerveDriveSubsystem m_swerveDrive;
  private final XboxController m_controller;

  public DriveWithJoystick(CIMSwerveDriveSubsystem swerveDrive, XboxController controller) {
    m_swerveDrive = swerveDrive;
    m_controller = controller;
    // Use addRequirements() to tell the scheduler that this command requires the subsystem.
    addRequirements(m_swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get driver inputs from the sticks
    // Invert Y axis because WPILib treats positive Y as down by default
    double xSpeed = m_controller.getLeftX(); 
    double ySpeed = -m_controller.getLeftY();
    double rot = m_controller.getRightX();

    // Apply a deadband
    //xSpeed = Math.abs(xSpeed) > OIConstants.CONTROLLER_DEADBAND ? xSpeed : 0.0;
    //ySpeed = Math.abs(ySpeed) > OIConstants.CONTROLLER_DEADBAND ? ySpeed : 0.0;

    double mag = Math.sqrt(Math.pow(ySpeed, 2) + Math.pow(xSpeed, 2));
    if(mag < OIConstants.CONTROLLER_DEADBAND) {
      xSpeed = 0.0;
      ySpeed = 0.0;
    }
    rot = Math.abs(rot) > OIConstants.CONTROLLER_DEADBAND ? rot : 0.0;

    // Use field-centric control (robot relative would use ChassisSpeeds.fromFieldRelativeSpeeds)
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);

    // Pass the speeds to the subsystem's drive method
    m_swerveDrive.drive(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the robot when the command stops
    m_swerveDrive.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
