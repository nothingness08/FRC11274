// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

/** An example command that uses an example subsystem. */
public class MoveToTargetAuto extends Command {
  private final SwerveDriveSubsystem m_swerveDrive;
  private final TelemetrySubsystem m_telemetrySubsystem;

  private final HolonomicDriveController holonomicController = 
    new HolonomicDriveController(
      new PIDController(1.1, 0, 0.005),
      new PIDController(1.1, 0, 0.005),
      new ProfiledPIDController(1.5, 0.01, 0.01, 
        new TrapezoidProfile.Constraints(6.28, 3.14)
      )
    );

  private Pose2d targetPos;
  /*
  *
   * Creates a new ExampleCommand.
   *
   * @param m_swerveDrive The subsystem used by this command.
   */
  public MoveToTargetAuto(SwerveDriveSubsystem swerveDrive, TelemetrySubsystem telemetrySubsystem, Pose2d targetPos) {
    m_swerveDrive = swerveDrive;
    this.targetPos = targetPos;
    m_telemetrySubsystem = telemetrySubsystem;
    holonomicController.setTolerance(new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(4)));
    addRequirements(m_swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = m_telemetrySubsystem.getPose();
    if(m_telemetrySubsystem.getLimelightTV()){
      ChassisSpeeds newSpeeds = holonomicController.calculate(currentPose, targetPos, 0, targetPos.getRotation());
      ChassisSpeeds newSpeedsFixed = new ChassisSpeeds(-newSpeeds.vxMetersPerSecond, newSpeeds.vyMetersPerSecond, -newSpeeds.omegaRadiansPerSecond);
      m_swerveDrive.drive(newSpeedsFixed, false);
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
    return holonomicController.atReference();
  }
}
