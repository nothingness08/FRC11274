// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  double POWER_CONSTANT = 0.5;


  private final RobotContainer m_robotContainer;

  private static XboxController controller = new XboxController(0);

  private static WPI_TalonSRX motor0 = new WPI_TalonSRX(0);


  private final Timer periodicTimer = new Timer();
  
  Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
  );

  private static double currentAngle;
  private static double currentTick;

  private static double lastAngle = 0;
  private static double offset = 0;

  public static double thetaToTick(double theta) {
    return ((theta - 90) / 360.0) * 4096.0;
  }

  public static double getCurrentTick(double currentAngle, double lastAngle) {
    // double distance = Math.abs(currentAngle - lastAngle);
    
    // if(distance > 180){
    //   if (currentAngle < 0 && lastAngle > 0) {
    //     offset += 360;
    //   } else if (currentAngle > 0 && lastAngle < 0) {
    //     offset -= 360.0;
    //   }
    // }

    double delta = currentAngle - lastAngle;
    
    if (delta > 180) {
      offset -= 360;
    } else if (delta < -180) {
      offset += 360;
    }
    
    double continousAngle = currentAngle + offset;
    double currentTick = thetaToTick(continousAngle);
    return currentTick;
  }


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    periodicTimer.start();
    motor0.configFactoryDefault();

    motor0.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);

    motor0.setSensorPhase(true);
    motor0.setInverted(true);

    motor0.configNominalOutputForward(0, 10); 
    motor0.configNominalOutputReverse(0, 10);
    motor0.configPeakOutputForward(1, 10);
    motor0.configPeakOutputReverse(-1, 10);
    motor0.configAllowableClosedloopError(0, 0, 10);

    motor0.config_kF(0, 0, 10);
    motor0.config_kP(0, 0.4, 10);
    motor0.config_kI(0, 0, 10);
    motor0.config_kD(0, 0.01, 10);

    motor0.setSelectedSensorPosition(0, 0, 10);

  


  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    

    //System.out.println(motor0.getSelectedSensorPosition(0));
    
    ChassisSpeeds speeds = new ChassisSpeeds(controller.getLeftX(), controller.getLeftY()*-1, 0);
    // Convert to module states
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);
    // Front left module state
    SwerveModuleState frontLeft = moduleStates[0];
    // Front right module state
    SwerveModuleState frontRight = moduleStates[1];
    // Back left module state
    SwerveModuleState backLeft = moduleStates[2];
    // Back right module state
    SwerveModuleState backRight = moduleStates[3];
    
   
    double mag = Math.sqrt(Math.pow(controller.getLeftY(), 2) + Math.pow(controller.getLeftX(), 2));
    
    if(mag > 0.9){
      
      currentAngle = frontLeft.angle.getDegrees();
  
      currentTick = getCurrentTick(currentAngle, lastAngle);
      lastAngle = currentAngle;

      motor0.set(TalonSRXControlMode.Position, currentTick);
      System.out.println("Degree: " + frontLeft.angle.getDegrees());
      System.out.println("Tick: " + currentTick);
      System.out.println("Last: " + lastAngle);
      System.out.println("Current: " + currentAngle);
      System.out.println();
    }

    CommandScheduler.getInstance().run();
  }

  





  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}




// if(controller.getAButton()){
//   motor0.set(TalonSRXControlMode.Position, 0);
// } 
// if(controller.getBButton()){
//   motor0.set(TalonSRXControlMode.Position, 1000);
// }
// if(controller.getXButton()){
//   motor0.set(TalonSRXControlMode.Position, -1000);
// }