// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.PrintStream;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;


public class CIMSwerveDriveSubsystem extends SubsystemBase {
  //put this into array later for all 4 modules
  private final WPI_TalonSRX m_frontLeftAngleMotor = new WPI_TalonSRX(SwerveDriveConstants.FRONT_LEFT_ANGLE_MOTOR_ID);
  private final WPI_TalonSRX m_frontRightAngleMotor = new WPI_TalonSRX(SwerveDriveConstants.FRONT_RIGHT_ANGLE_MOTOR_ID);
  private final WPI_TalonSRX m_backLeftAngleMotor = new WPI_TalonSRX(SwerveDriveConstants.BACK_LEFT_ANGLE_MOTOR_ID);
  private final WPI_TalonSRX m_backRightAngleMotor = new WPI_TalonSRX(SwerveDriveConstants.BACK_RIGHT_ANGLE_MOTOR_ID);

  private final WPI_TalonSRX[]  m_AngleMotor= {
    m_frontLeftAngleMotor,
    m_frontRightAngleMotor,
    m_backLeftAngleMotor,
    m_backRightAngleMotor
  };

  float rpm[];
  private final WPI_TalonSRX m_frontLeftDriveMotor = new WPI_TalonSRX(SwerveDriveConstants.FRONT_LEFT_DRIVE_MOTOR_ID);
  private final WPI_TalonSRX m_frontRightDriveMotor = new WPI_TalonSRX(SwerveDriveConstants.FRONT_RIGHT_DRIVE_MOTOR_ID);
  private final WPI_TalonSRX m_backLeftDriveMotor = new WPI_TalonSRX(SwerveDriveConstants.BACK_LEFT_DRIVE_MOTOR_ID);
  private final WPI_TalonSRX m_backRightDriveMotor = new WPI_TalonSRX(SwerveDriveConstants.BACK_RIGHT_DRIVE_MOTOR_ID);

  private final WPI_TalonSRX[]  m_DriveMotor= {
    m_frontLeftDriveMotor,
    m_frontRightDriveMotor,
    m_backLeftDriveMotor,
    m_backRightDriveMotor
  };

  private double[] lastAngle = {0, 0, 0, 0};
  private double[] offset = {0, 0, 0, 0};
  private double[] rotAngles = {45, -45, 135, -135};
  
  // private double lastAngle = 0;
  // private double offset = 0;
  // private double currentTick = 0;
  // private double currentAngle = 0;

  public CIMSwerveDriveSubsystem() {
    //make this for loop to initialize all 4 modules
    for(WPI_TalonSRX angleMotor : m_AngleMotor) {
      angleMotor.configFactoryDefault();

      angleMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);

      angleMotor.setSensorPhase(true);
      angleMotor.setInverted(true);

      angleMotor.configNominalOutputForward(0, 10); 
      angleMotor.configNominalOutputReverse(0, 10);
      angleMotor.configPeakOutputForward(1, 10);
      angleMotor.configPeakOutputReverse(-1, 10);
      angleMotor.configAllowableClosedloopError(0, 0, 10);

      angleMotor.config_kF(0, SwerveDriveConstants.kF, 10);
      angleMotor.config_kP(0, SwerveDriveConstants.kP, 10);
      angleMotor.config_kI(0, SwerveDriveConstants.kI, 10);
      angleMotor.config_kD(0, SwerveDriveConstants.kD, 10);

      angleMotor.setSelectedSensorPosition(0, 0, 10);
    }

    for(WPI_TalonSRX driveMotor : m_DriveMotor) {
      driveMotor.configFactoryDefault();

      driveMotor.setInverted(false);

      driveMotor.configNominalOutputForward(0, 10); 
      driveMotor.configNominalOutputReverse(0, 10);
      driveMotor.configPeakOutputForward(1, 10);
      driveMotor.configPeakOutputReverse(-1, 10);

      driveMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }
  }

  
  public double thetaToTick(double theta) {
    return ((theta - 90) / 360.0) * SwerveDriveConstants.TICKS_PER_REVOLUTION;
  }

  public double getCurrentTick(double currentAngle, int moduleIndex) {
    double delta = currentAngle - lastAngle[moduleIndex];
    
    if (delta > 180) {
      offset[moduleIndex] -= 360;
    } else if (delta < -180) {
      offset[moduleIndex] += 360;
    }
    
    double currentTick = thetaToTick(currentAngle + offset[moduleIndex]);
    return currentTick;
  }

  public void drive(ChassisSpeeds speeds){
    SwerveModuleState[] moduleStates = SwerveDriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
    //SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveDriveConstants.kMaxSpeedMetersPerSecond);
    //SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, SwerveDrive.MAX_SPEED_METERS_PER_SECOND);
    System.out.println(speeds.omegaRadiansPerSecond);

    for(int i = 0; i < moduleStates.length; i++) {
      SwerveModuleState state = moduleStates[i];
      double currentAngle;
      if(speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0 && speeds.omegaRadiansPerSecond != 0){
        currentAngle = rotAngles[i];
        m_DriveMotor[i].set(TalonSRXControlMode.PercentOutput, speeds.omegaRadiansPerSecond*SwerveDriveConstants.MAXPERCENTOUTPUT);
        m_backLeftDriveMotor.set(TalonSRXControlMode.PercentOutput, speeds.omegaRadiansPerSecond*SwerveDriveConstants.MAXPERCENTOUTPUT);
      } else if(speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0 && speeds.omegaRadiansPerSecond == 0){
        currentAngle = lastAngle[i];
        m_DriveMotor[i].set(TalonSRXControlMode.PercentOutput, 0);
      } 
      else{
        currentAngle = state.angle.getDegrees();
        m_DriveMotor[i].set(TalonSRXControlMode.PercentOutput, SwerveDriveConstants.MAXPERCENTOUTPUT);
        m_backLeftDriveMotor.set(TalonSRXControlMode.PercentOutput, SwerveDriveConstants.MAXPERCENTOUTPUT);
      } 

      double currentTick = getCurrentTick(currentAngle, i);
      lastAngle[i] = currentAngle;
      m_AngleMotor[i].set(TalonSRXControlMode.Position, currentTick);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Current Tick FL: ", m_AngleMotor[0].getSelectedSensorPosition());
    SmartDashboard.putNumber("Current Tick FR: ", m_AngleMotor[1].getSelectedSensorPosition());
    SmartDashboard.putNumber("Current Tick BL: ", m_AngleMotor[2].getSelectedSensorPosition());
    SmartDashboard.putNumber("Current Tick BR: ", m_AngleMotor[3].getSelectedSensorPosition());

    SmartDashboard.putNumber("Target Angle FL: ", lastAngle[0]);
    SmartDashboard.putNumber("Target Angle FR: ", lastAngle[1]);
    SmartDashboard.putNumber("Target Angle BL: ", lastAngle[2]);
    SmartDashboard.putNumber("Target Angle BR: ", lastAngle[3]);
  }
}