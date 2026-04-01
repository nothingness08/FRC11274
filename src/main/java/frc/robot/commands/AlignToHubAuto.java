
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.*;

public class AlignToHubAuto extends Command {
  private final SwerveDriveSubsystem m_swerveDrive;
  private final TelemetrySubsystem m_telemetry;
  private final PIDController pid = new PIDController(0.03, 0, 0);

  public AlignToHubAuto(SwerveDriveSubsystem swerve, TelemetrySubsystem telemetry) {
    m_swerveDrive = swerve;
    m_telemetry = telemetry;
    addRequirements(swerve);
  }

  @Override
  public void execute() {
    double currentDeg = m_telemetry.getPose().getRotation().getDegrees();
    double targetDeg = m_telemetry.targetRotationToHub().getDegrees();
    double error = Math.IEEEremainder(targetDeg - currentDeg, 360.0);
    
    double rot = -pid.calculate(0, error);
    // Move at 0 speed but apply rotation
    m_swerveDrive.drive(new ChassisSpeeds(0, 0, rot * SwerveDriveConstants.MAX_ROTATE_SPEED), true);
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.drive(new ChassisSpeeds(0,0,0), true);
  }
}
