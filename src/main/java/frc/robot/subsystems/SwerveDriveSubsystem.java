// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;;


public class SwerveDriveSubsystem extends SubsystemBase {
  //put this into array later for all 4 modules
  private final WPI_TalonSRX m_frontLeftAngleMotor = new WPI_TalonSRX(SwerveDriveConstants.FRONT_LEFT_ANGLE_MOTOR_ID);
  private double lastAngle = 0;
  private double offset = 0;

  
  public SwerveDriveSubsystem() {
    //make this for loop to initialize all 4 modules
    m_frontLeftAngleMotor.configFactoryDefault();

    m_frontLeftAngleMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);

    m_frontLeftAngleMotor.setSensorPhase(true);
    m_frontLeftAngleMotor.setInverted(false);

    m_frontLeftAngleMotor.configNominalOutputForward(0, 10); 
    m_frontLeftAngleMotor.configNominalOutputReverse(0, 10);
    m_frontLeftAngleMotor.configPeakOutputForward(1, 10);
    m_frontLeftAngleMotor.configPeakOutputReverse(-1, 10);
    m_frontLeftAngleMotor.configAllowableClosedloopError(0, 0, 10);

    m_frontLeftAngleMotor.config_kF(0, SwerveDriveConstants.kF, 10);
    m_frontLeftAngleMotor.config_kP(0, SwerveDriveConstants.kP, 10);
    m_frontLeftAngleMotor.config_kI(0, SwerveDriveConstants.kI, 10);
    m_frontLeftAngleMotor.config_kD(0, SwerveDriveConstants.kD, 10);

    m_frontLeftAngleMotor.setSelectedSensorPosition(0, 0, 10);
  }

  
  public double thetaToTick(double theta) {
    return ((theta - 90) / 360.0) * SwerveDriveConstants.TICKS_PER_REVOLUTION;
  }

  public double getCurrentTick(double currentAngle) {
    double delta = currentAngle - lastAngle;
    
    if (delta > 180) {
      offset -= 360;
    } else if (delta < -180) {
      offset += 360;
    }
    
    double currentTick = thetaToTick(currentAngle + offset);
    return currentTick;
  }

  public void drive(ChassisSpeeds speeds){
    SwerveModuleState[] moduleStates = SwerveDriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
    //SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveDriveConstants.kMaxSpeedMetersPerSecond);
    //SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, SwerveDrive.MAX_SPEED_METERS_PER_SECOND);

    SwerveModuleState frontLeftAngleState = moduleStates[0];

    double currentAngle = frontLeftAngleState.angle.getDegrees();
  
    double currentTick = getCurrentTick(currentAngle);
    lastAngle = currentAngle;
    m_frontLeftAngleMotor.set(TalonSRXControlMode.Position, currentTick);
  }

  @Override
  public void periodic() {

  }

  
}
