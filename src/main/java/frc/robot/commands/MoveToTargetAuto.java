// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

/** An example command that uses an example subsystem. */
public class MoveToTargetAuto extends Command {
  private final SwerveDriveSubsystem m_swerveDrive;
  private final PIDController xController, yController, rotController;
  private final TelemetrySubsystem m_telemetrySubsystem;

  
  
  private double targetPos[];
  /*
  *
   * Creates a new ExampleCommand.
   *
   * @param m_swerveDrive The subsystem used by this command.
   */
  public MoveToTargetAuto(SwerveDriveSubsystem swerveDrive, TelemetrySubsystem telemetrySubsystem, double[] targetPos) {
    m_swerveDrive = swerveDrive;
    this.targetPos = targetPos;
    m_telemetrySubsystem = telemetrySubsystem;
    xController = new PIDController(0.02, 0, 0.001); 
    yController = new PIDController(0.02, 0, 0.001);
    rotController = new PIDController(1, 0, 0.01);
    addRequirements(m_swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPos[] = m_telemetrySubsystem.getPoseEstimateWithTag();
    double currentYawInTagSpace = m_telemetrySubsystem.getBotPose_TargetSpace()[4];
    if(m_telemetrySubsystem.getLimelightTV()){
      double xSpeed = xController.calculate(currentPos[0], targetPos[0]);
      double ySpeed = yController.calculate(currentPos[1], targetPos[1]);
      double rotSpeed = rotController.calculate(currentYawInTagSpace, targetPos[2]);
      double mag = Math.sqrt(Math.pow(ySpeed, 2) + Math.pow(xSpeed, 2));
      if(mag > 1.0){
        xSpeed *= (1/(mag));
        ySpeed *= (1/(mag));
      }
      m_swerveDrive.drive(new ChassisSpeeds(xSpeed, ySpeed, 0), true);
    }
    else{
      m_swerveDrive.drive(new ChassisSpeeds(0, 0, 0), true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.drive(new ChassisSpeeds(0.0, 0.0, 0.0), false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!m_telemetrySubsystem.getLimelightTV()){
      return true;
    }
    double currentPos[] = m_telemetrySubsystem.getPoseEstimateWithTag();
    double distance = Math.hypot(targetPos[0] - currentPos[0], targetPos[1] - currentPos[1]);
    return distance < 0.1;
  }
}
