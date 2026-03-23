package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.TelemetrySubsystem;
import frc.robot.util.HolonomicWaypoint;
import java.util.List;

public class FollowWaypointsAuto extends Command {
  private final SwerveDriveSubsystem m_swerveDrive;
  private final TelemetrySubsystem m_telemetrySubsystem;;
  private final List<HolonomicWaypoint> m_waypoints;
  private final HolonomicDriveController m_controller;
  private int m_index = 0;

  public FollowWaypointsAuto(SwerveDriveSubsystem swerveDrive, TelemetrySubsystem telemetrySubsystem, List<HolonomicWaypoint> waypoints) {
    m_swerveDrive = swerveDrive;
    m_telemetrySubsystem = telemetrySubsystem;
    m_waypoints = waypoints;

    PIDController xController = new PIDController(5, 0, 0);
    PIDController yController = new PIDController(5, 0, 0);
    
    ProfiledPIDController thetaController = new ProfiledPIDController(
      3, 0, 0, 
      new TrapezoidProfile.Constraints(
          6.28, 
          3.14)
    );
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    this.m_controller = new HolonomicDriveController(xController, yController, thetaController);
    
    addRequirements(m_swerveDrive);
  }

  @Override
  public void initialize() {
    m_index = 0;
    //m_telemetrySubsystem.resetPose(null);
  }

  @Override
  public void execute() {
    if (m_index >= m_waypoints.size()) return;

    HolonomicWaypoint goal = m_waypoints.get(m_index);

    // Update tolerance for this specific waypoint
    m_controller.setTolerance(goal.tolerance());

    // Calculate output
    ChassisSpeeds speeds = m_controller.calculate(
      m_telemetrySubsystem.getPose(),
      goal.pose(),
      goal.velocity(),
      goal.heading()
    );

    if(m_telemetrySubsystem.getAlliance() == DriverStation.Alliance.Blue){
      m_swerveDrive.drive(
        new ChassisSpeeds(-speeds.vyMetersPerSecond, speeds.vxMetersPerSecond, -speeds.omegaRadiansPerSecond), 
        true
      );
    }
    else{
      m_swerveDrive.drive(
        new ChassisSpeeds(speeds.vyMetersPerSecond, -speeds.vxMetersPerSecond, -speeds.omegaRadiansPerSecond), 
        true
      );
    }

    // Advance to next waypoint if we are close enough
    if (m_controller.atReference()) {
      m_index++;
    }
  }

  @Override
  public boolean isFinished() {
    return m_index >= m_waypoints.size();
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.drive(new ChassisSpeeds(0.0, 0.0, 0.0), false);
  }
}
