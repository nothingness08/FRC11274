// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final TalonFX m_climber = new TalonFX(ClimberConstants.CLIMBER_ID, ClimberConstants.CANbus);

  public ClimberSubsystem() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    
    configs.Slot0.kP = ClimberConstants.kP_Align;
    configs.Slot0.kG = ClimberConstants.kG_Align;

    configs.Slot1.kP = ClimberConstants.kP_Climb;
    configs.Slot1.kG = ClimberConstants.kG_Climb;

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    configs.CurrentLimits.SupplyCurrentLimit = 80;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    
    configs.CurrentLimits.StatorCurrentLimit = 80;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;

    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants.MAX_HEIGHT_ROTATIONS;
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    m_climber.getConfigurator().apply(configs);

    m_climber.setPosition(0);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  /**
   * Move the elevator up and down.
   * @param dutycycle [-1, 1] speed to set the elevator too.
   */
  public Command setDutyCycle(double dutycycle) { 
      return run(() -> m_climber.setControl(new DutyCycleOut(dutycycle)))
        .finallyDo(() -> stop()); 
    }

  public void stop() {
    m_climber.setControl(new DutyCycleOut(0));
  }

  public Command setPosition(double rotations, boolean climbMode) {
    int slot = climbMode ? 1 : 0;
    return run(() -> m_climber.setControl(new PositionVoltage(rotations).withSlot(slot)))
        .until(() -> {
            double currentPosition = m_climber.getPosition().refresh().getValueAsDouble();
            return Math.abs(currentPosition - rotations) < 1.5;
        })
        .finallyDo(() -> {
            // Optional: Stop the motor or switch to a neutral mode when finished
            m_climber.setControl(new DutyCycleOut(0));
        });

  }

  public double getPosition() {
    return m_climber.getPosition().refresh().getValueAsDouble();
  }

  public Command switchLimitsCommand(){
    return runOnce(() -> switchLimits());
  }

  public Command setCurrentPosToZeroCommand(){
    return runOnce(() -> setCurrentPosToZero());
  }

  private void switchLimits(){
    var swLimits = new SoftwareLimitSwitchConfigs();

    // 2. Fetch the current settings from the motor (prevents overwriting thresholds)
    m_climber.getConfigurator().refresh(swLimits);

    boolean newState = !swLimits.ForwardSoftLimitEnable;
    
    swLimits.ForwardSoftLimitEnable = newState;
    swLimits.ReverseSoftLimitEnable = newState;

    // 4. Apply only this specific group back to the motor
    m_climber.getConfigurator().apply(swLimits);
  }

  private void setCurrentPosToZero(){
    m_climber.setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Pos Rotations", getPosition());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}