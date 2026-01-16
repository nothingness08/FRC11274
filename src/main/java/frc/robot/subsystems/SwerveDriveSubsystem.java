// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.PrintStream;
import java.util.ConcurrentModificationException;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.libs.LimelightHelpers;


import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.LimelightSubsystem;

public class SwerveDriveSubsystem extends SubsystemBase {

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

  private final TalonFX m_frontLeftDriveMotor = new TalonFX(SwerveDriveConstants.FRONT_LEFT_DRIVE_MOTOR_ID, SwerveDriveConstants.CANbus);
  private final TalonFX m_frontRightDriveMotor = new TalonFX(SwerveDriveConstants.FRONT_RIGHT_DRIVE_MOTOR_ID, SwerveDriveConstants.CANbus);
  private final TalonFX m_backLeftDriveMotor = new TalonFX(SwerveDriveConstants.BACK_LEFT_DRIVE_MOTOR_ID, SwerveDriveConstants.CANbus);
  private final TalonFX m_backRightDriveMotor = new TalonFX(SwerveDriveConstants.BACK_RIGHT_DRIVE_MOTOR_ID, SwerveDriveConstants.CANbus);

  private final TalonFX[]  m_DriveMotor= {
    m_frontLeftDriveMotor,
    m_frontRightDriveMotor,
    m_backLeftDriveMotor,
    m_backRightDriveMotor
  };

  private PIDController[] driveControllers = {
    new PIDController(0.02, 0.0, 0.002),
    new PIDController(0.02, 0.0, 0.002),
    new PIDController(0.02, 0.0, 0.002),
    new PIDController(0.02, 0.0, 0.002),
  };

  private double[] lastAngle = {0, 0, 0, 0};
  private double[] offset = {0, 0, 0, 0};
  private double[] targetTick = {0, 0, 0, 0};

  private double[] percentOutputSign = {-1, -1, -1, -1};
  
  private double[] rotAngles = {45, -45, 135, -135};


  private double[][] rotAnglesComponents = {{ Math.sqrt(2)/2, Math.sqrt(2)/2}, {Math.sqrt(2)/2, -Math.sqrt(2)/2}, {-Math.sqrt(2)/2, Math.sqrt(2)/2}, {-Math.sqrt(2)/2, -Math.sqrt(2)/2}};
  private Pigeon m_pigeon;

  public SwerveDriveSubsystem(Pigeon pigeon) {
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

    for(TalonFX driveMotor : m_DriveMotor) {
      TalonFXConfiguration configs = new TalonFXConfiguration();

      configs.Slot0.kP = SwerveDriveConstants.kP;
      configs.Slot0.kI = SwerveDriveConstants.kI;
      configs.Slot0.kD = SwerveDriveConstants.kD;
      configs.Slot0.kV = 2;
      
      configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

      driveMotor.getConfigurator().apply(configs);

    }

    m_pigeon = pigeon;
  }

  private double findAngles(double[] velocities){
    if(velocities[0] < 0){
      if(velocities[1] < 0){
        return Math.toDegrees(Math.atan(velocities[1]/velocities[0])) - 180;
      } else{
        return Math.toDegrees(Math.atan(velocities[1]/velocities[0])) + 180;
      }
    }
    else{
      return Math.toDegrees(Math.atan(velocities[1]/velocities[0]));
    } 
  }

  private double[] offsetByAngle(double[] velocities, double angle){
    if(velocities[0] == 0 && velocities[1]==0){
      return new double[]{0,0};
    }
    double translationAngle = findAngles(velocities);
    double offsetTranslationAngle = translationAngle - angle;
    double vxOffset = Math.cos(Math.toRadians(offsetTranslationAngle)) * Math.sqrt(velocities[0] * velocities[0] + velocities[1] * velocities[1]);
    double vyOffset = Math.sin(Math.toRadians(offsetTranslationAngle)) * Math.sqrt(velocities[0] * velocities[0] + velocities[1] * velocities[1]);
    return new double[]{vxOffset, vyOffset};
  }
  
  private double[][] getVelocitiesAngles(ChassisSpeeds speeds){
    double [][] velocitiesAngles = new double[4][2];
    double[] velocitiesOffset = offsetByAngle(new double[]{speeds.vxMetersPerSecond, speeds.vyMetersPerSecond}, m_pigeon.getYaw());


    for(int i = 0; i < 4; i++) {
      double vx = speeds.omegaRadiansPerSecond * rotAnglesComponents[i][0] + velocitiesOffset[0];
      double vy = speeds.omegaRadiansPerSecond * rotAnglesComponents[i][1] + velocitiesOffset[1];
      velocitiesAngles[i][0] = Math.sqrt(vx * vx + vy * vy);
      velocitiesAngles[i][1] = findAngles(new double[] {vx, vy});
    }
    return velocitiesAngles;
  }

  private double thetaToTick(double theta) {
    return ((theta - 90) / 360.0) * SwerveDriveConstants.TICKS_PER_REVOLUTION;
  }

  private double getCurrentTick(double currentAngle, int moduleIndex) {
    double delta = currentAngle - lastAngle[moduleIndex];
    
    if (delta > 180) {
      offset[moduleIndex] -= 360;
    } else if (delta < -180) {
      offset[moduleIndex] += 360;
    }
    
    double currentTick = thetaToTick(currentAngle + offset[moduleIndex]);
    return currentTick;
  }

  public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
    double [][] velocitiesAndAngles = getVelocitiesAngles(speeds);
    
    for(int i = 0; i < 4; i++) {
      double desiredPercentOutput = SwerveDriveConstants.MAXPERCENTOUTPUT * velocitiesAndAngles[i][0];
      double currentAngle, modifiedCurrentAngle;
      // if(i==0){
      //   System.out.println("last: " + lastAngle[i]);
      //   System.out.println("current: " + currentAngle);
      // }
      if(speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0 && speeds.omegaRadiansPerSecond == 0){
        currentAngle = lastAngle[i];
      }
      else{
        currentAngle = velocitiesAndAngles[i][1];
      }
      modifiedCurrentAngle = currentAngle;
      if((Math.abs(lastAngle[i] - currentAngle)) > 90 && !Double.isNaN(currentAngle)){
        percentOutputSign[i] *= -1;
        if(currentAngle > 0){
          modifiedCurrentAngle -=180;
        }
        else{
          modifiedCurrentAngle +=180;
        }
      }
      
      desiredPercentOutput *= percentOutputSign[i];
      if(i==0){
        System.out.println(desiredPercentOutput);
      }
      m_DriveMotor[i].set(desiredPercentOutput);
      double currentTick = getCurrentTick(modifiedCurrentAngle, i);
      targetTick[i] = currentTick;
      lastAngle[i] = currentAngle;
      m_AngleMotor[i].set(TalonSRXControlMode.Position, currentTick);
    }
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(int i = 0; i < 4; i++){
      double distanceMeters = (m_DriveMotor[i].getPosition().getValueAsDouble() / SwerveDriveConstants.DRIVER_GEAR_RATIO) * (Math.PI * 0.1016); //0.1016 is wheel diameter in meters
      double angleRotations = m_AngleMotor[i].getSelectedSensorPosition() / 4096.0;
      Rotation2d angle = Rotation2d.fromRotations(angleRotations);  
      positions[i] = new SwerveModulePosition(distanceMeters, angle);
    }
    return positions;
  }

  private double[] getDeltaTarget(){
    double[] deltas = new double[4];
    for(int i = 0; i < 4; i++){
      deltas[i] = Math.abs(targetTick[i] - m_AngleMotor[i].getSelectedSensorPosition());
    }
    return deltas;
  }

  @Override
  public void periodic() {

    // SmartDashboard.putNumber("Actual Tick FL: ", m_AngleMotor[0].getSelectedSensorPosition());
    // SmartDashboard.putNumber("Actual Tick FR: ", m_AngleMotor[1].getSelectedSensorPosition());
    // SmartDashboard.putNumber("Actual Tick BL: ", m_AngleMotor[2].getSelectedSensorPosition());
    // SmartDashboard.putNumber("Actual Tick BR: ", m_AngleMotor[3].getSelectedSensorPosition());

    // double[] deltas = getDeltaTarget();
    // SmartDashboard.putNumber("Delta Tick FL: ", deltas[0]);
    // SmartDashboard.putNumber("Delta Tick FR: ", deltas[1]);
    // SmartDashboard.putNumber("Delta Tick BL: ", deltas[2]);
    // SmartDashboard.putNumber("Delta Tick BR: ", deltas[3]);

    // SmartDashboard.putNumber("Target Angle FL: ", lastAngle[0]);
    // SmartDashboard.putNumber("Target Angle FR: ", lastAngle[1]);
    // SmartDashboard.putNumber("Target Angle BL: ", lastAngle[2]);
    // SmartDashboard.putNumber("Target Angle BR: ", lastAngle[3]);
  }
}