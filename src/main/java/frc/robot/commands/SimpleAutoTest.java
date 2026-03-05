// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

/** An example command that uses an example subsystem. */
public class SimpleAutoTest extends Command {
  private final SwerveDriveSubsystem m_swerveDrive;
  private final ShooterSubsystem m_ShooterSubsystem;
  private static final Timer m_timer = new Timer();


  /*
  *
   * Creates a new ExampleCommand.
   *
   * @param m_swerveDrive The subsystem used by this command.
   */
  public SimpleAutoTest(SwerveDriveSubsystem swerveDrive, ShooterSubsystem shooterSubsystem) {
    m_swerveDrive = swerveDrive;
    m_ShooterSubsystem = shooterSubsystem;
    m_ShooterSubsystem.setDutyCycle(0.93).schedule();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveDrive, m_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("auto running: " + m_timer.get());
    // if (m_timer.get() < 1.5) {
    //     // Drive motors forward at 50% power
    //     m_swerveDrive.drive(new ChassisSpeeds(0, -0.4, 0), true);
    // }
    // else if(m_timer.get() < 2){
    //   m_swerveDrive.drive(new ChassisSpeeds(0, 0, 0.5), true);

    // } else if(m_timer.get() < 5){
    //   m_ShooterSubsystem.set(0.93);
    // }else{
    //     // Stop motors after 2 seconds
    //     m_swerveDrive.drive(new ChassisSpeeds(0, 0, 0), true);
    // }    
    //m_ShooterSubsystem.set(0.93);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.drive(new ChassisSpeeds(0.0, 0.0, 0.0), false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
