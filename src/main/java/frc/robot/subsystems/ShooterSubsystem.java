// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ShooterConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX m_shooter = new TalonFX(ShooterConstants.shooter_ID, ShooterConstants.CANbus);
  private final TalonFX m_follower = new TalonFX(ShooterConstants.shooter2_ID, ShooterConstants.CANbus);
  
  private final TalonFX m_feeder = new TalonFX(ShooterConstants.feeder_ID, ShooterConstants.CANbus);
  Follower toFollowLeader = new Follower(m_shooter.getDeviceID(), MotorAlignmentValue.Opposed);

  public ShooterSubsystem() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.Slot0.kV = ShooterConstants.kV;
    configs.Slot0.kP = ShooterConstants.kP;
    configs.Slot0.kS = ShooterConstants.kS;
    configs.Slot0.kI = ShooterConstants.kI;

    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    configs.Voltage.PeakForwardVoltage = 11.0;
    configs.Voltage.PeakReverseVoltage = -11.0;
    
    configs.CurrentLimits.SupplyCurrentLimit = 60;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    
    configs.CurrentLimits.StatorCurrentLimit = 60;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;

    m_shooter.getConfigurator().apply(configs);

    m_follower.setControl(toFollowLeader);
  }

  /**
   * Move the elevator up and down.
   * @param dutycycle [-1, 1] speed to set the elevator too.
   */
  public Command setDutyCycle(double dutycycle) { 
    return run(() -> m_shooter.setControl(new DutyCycleOut(dutycycle)))
      .finallyDo(() -> stop()); 
  }

  public void stop() {
    m_shooter.setControl(new DutyCycleOut(0));
  }

  public Command setDutyCycleFeeder(double dutycycle) { 
    return run(() -> m_feeder.setControl(new DutyCycleOut(dutycycle)))
      .finallyDo(() -> stopFeeder()); 
  }

  public void stopFeeder() {
    m_feeder.setControl(new DutyCycleOut(0));
  }

  public Command setVelocity(double velocityRPS) {
    return run(() -> m_shooter.setControl(new VelocityVoltage(velocityRPS).withSlot(0)))
      .finallyDo((interrupted) -> stop());
  }

  public double getVelocity() {
    // refresh() is called to get the most up-to-date data from the CAN bus
    return m_shooter.getVelocity().refresh().getValueAsDouble();
  }

  public boolean atSetpoint(double targetRPS) {
    return Math.abs(getVelocity() - targetRPS) < 3.0; 
  }

  public Command shootSequence(double feederPercent, double shooterRPS) {
    return run(() -> {
       m_shooter.setControl(new VelocityVoltage(shooterRPS).withSlot(0));

      // 2. Only run the feeder if the shooter is at speed
      if (atSetpoint(shooterRPS)) {
        m_feeder.setControl(new DutyCycleOut(feederPercent));
      } else {
        m_feeder.setControl(new DutyCycleOut(0));
      }
    })
    .finallyDo((interrupted) -> {
      stop();
      stopFeeder();
    });
  }

  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("Flywheel RPS", getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
