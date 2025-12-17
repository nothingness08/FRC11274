// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.PrintStream;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.VecBuilder;
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

  private double[] lastAngle = {0, 0, 0, 0};
  private double[] offset = {0, 0, 0, 0};
  private double[] targetTick = {0, 0, 0, 0};

  private double[] rotAngles = {45, -45, 135, -135};

  private final SwerveDrivePoseEstimator m_poseEstimator;
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
    m_poseEstimator = new SwerveDrivePoseEstimator(
      SwerveDriveConstants.KINEMATICS,
      Rotation2d.fromDegrees(m_pigeon.getYaw()),
      getModulePositions(),
      new Pose2d(0.0,0.0, Rotation2d.fromDegrees(0.0))
    );
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
    SwerveModuleState[] moduleStates = SwerveDriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
    //SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveDriveConstants.kMaxSpeedMetersPerSecond);
    //SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, SwerveDrive.MAX_SPEED_METERS_PER_SECOND);
    double[] deltas = getDeltaTarget();

    for(int i = 0; i < moduleStates.length; i++) {
      SwerveModuleState state = moduleStates[i];
      double currentAngle;
      double desiredPercentOutput = 0;

      if(speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0 && speeds.omegaRadiansPerSecond != 0){
        currentAngle = rotAngles[i];
        desiredPercentOutput = SwerveDriveConstants.MAXPERCENTOUTPUT * Math.signum(speeds.omegaRadiansPerSecond);
      } else if(speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0 && speeds.omegaRadiansPerSecond == 0){
        currentAngle = lastAngle[i];
      }
      else{
        currentAngle = state.angle.getDegrees();
        if(fieldRelative){
          currentAngle -= m_pigeon.getYaw();
        }
        desiredPercentOutput = SwerveDriveConstants.MAXPERCENTOUTPUT * state.speedMetersPerSecond;
      }
      desiredPercentOutput *=-1;
      
      // if(deltas[i] > 200){ //set band for driving
      //   desiredPercentOutput = 0;
      // }
      m_DriveMotor[i].set(desiredPercentOutput);
      double currentTick = getCurrentTick(currentAngle, i);
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

  public double[] getPoseEstimateWithTag(){
    double[] pose = new double[3];
    pose[0] = -(m_poseEstimator.getEstimatedPosition().getY() - AprilTagConstants.TAG_Y);
    pose[1] = m_poseEstimator.getEstimatedPosition().getX() - AprilTagConstants.TAG_X;
    pose[2] = m_poseEstimator.getEstimatedPosition().getRotation().getDegrees();
    return pose;
  }
  @Override
  public void periodic() {
    m_poseEstimator.update(
      Rotation2d.fromDegrees(m_pigeon.getYaw()),
      getModulePositions()
    );
    boolean doRejectUpdate = false;
    LimelightHelpers.SetRobotOrientation("limelight-two", 
      m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 
      0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = 
      LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-two");

    if (Math.abs(m_pigeon.getRate()) > 360) {
      doRejectUpdate = true;
    }
    if (mt2 != null && mt2.tagCount == 0) {
      doRejectUpdate = true;
    }

    if (!doRejectUpdate && mt2 != null) {
      // Trust vision for X/Y (0.7m error), ignore vision for rotation (999999 error)
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
      m_poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
    }

    double poseEstimate[] = getPoseEstimateWithTag();
    // SmartDashboard.putNumber("Estimated X", poseEstimate[0]);
    // SmartDashboard.putNumber("Estimated Y", poseEstimate[1]);

    // SmartDashboard.putNumber("Estimated X in", poseEstimate[0]*39.37);
    // SmartDashboard.putNumber("Estimated Y in", poseEstimate[1]*39.37);

    SmartDashboard.putNumber("Actual Tick FL: ", m_AngleMotor[0].getSelectedSensorPosition());
    SmartDashboard.putNumber("Actual Tick FR: ", m_AngleMotor[1].getSelectedSensorPosition());
    SmartDashboard.putNumber("Actual Tick BL: ", m_AngleMotor[2].getSelectedSensorPosition());
    SmartDashboard.putNumber("Actual Tick BR: ", m_AngleMotor[3].getSelectedSensorPosition());

    double[] deltas = getDeltaTarget();
    SmartDashboard.putNumber("Delta Tick FL: ", deltas[0]);
    SmartDashboard.putNumber("Delta Tick FR: ", deltas[1]);
    SmartDashboard.putNumber("Delta Tick BL: ", deltas[2]);
    SmartDashboard.putNumber("Delta Tick BR: ", deltas[3]);

    SmartDashboard.putNumber("Target Angle FL: ", lastAngle[0]);
    SmartDashboard.putNumber("Target Angle FR: ", lastAngle[1]);
    SmartDashboard.putNumber("Target Angle BL: ", lastAngle[2]);
    SmartDashboard.putNumber("Target Angle BR: ", lastAngle[3]);
  }
}