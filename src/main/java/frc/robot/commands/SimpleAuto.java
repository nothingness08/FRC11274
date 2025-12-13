// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

/** An example command that uses an example subsystem. */
public class SimpleAuto extends Command {
  private final SwerveDriveSubsystem m_swerveDrive;
  private final Pigeon m_pigeon;
  private final PIDController m_turnController;
  private double startAngle;
  private static final double TARGET_ANGLE_DEGREES = 360.0;
  private static final double TURN_SPEED = 0.1; // Max rotational speed (adjust as needed, ideally a small number < 1)
  private static final double ANGLE_TOLERANCE_DEGREES = 2.0; // How close we need to be to stop

  /*
  *
   * Creates a new ExampleCommand.
   *
   * @param m_swerveDrive The subsystem used by this command.
   */
  public SimpleAuto(SwerveDriveSubsystem swerveDrive, Pigeon pigeon) {
    m_swerveDrive = swerveDrive;
    m_pigeon = pigeon;

    m_turnController = new PIDController(0.02, 0.0, 0.002);
    
    m_turnController.setTolerance(ANGLE_TOLERANCE_DEGREES);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startAngle = m_pigeon.getYaw();
    m_turnController.setSetpoint(TARGET_ANGLE_DEGREES + startAngle);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = m_pigeon.getYaw();
    double turnSpeed = m_turnController.calculate(currentAngle);

    m_swerveDrive.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0, -turnSpeed*TURN_SPEED), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.drive(new ChassisSpeeds(0.0, 0.0, 0.0), false);
    m_turnController.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_turnController.atSetpoint();
  }
}
