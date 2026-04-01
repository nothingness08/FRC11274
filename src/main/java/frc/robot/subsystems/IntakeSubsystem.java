// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  //private final TalonFX m_roller = new TalonFX(IntakeConstants.RollerConstants.ROLLER_ID, IntakeConstants.CANbus);
  private final TalonFX m_pivot = new TalonFX(IntakeConstants.PivotConstants.PIVOT_ID, IntakeConstants.CANbus);
  private final TalonFX m_roller = new TalonFX(IntakeConstants.RollerConstants.ROLLER_ID, IntakeConstants.CANbus);
  private double targetRotation;
  public IntakeSubsystem() {
    TalonFXConfiguration rollerConfigs = new TalonFXConfiguration();
    TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
    
    pivotConfigs.Slot0.kP = IntakeConstants.PivotConstants.kP_Down;
    //pivotConfigs.Slot0.kG = IntakeConstants.PivotConstants.kG;
    //pivotConfigs.Slot0.kV = IntakeConstants.PivotConstants.kV;
    pivotConfigs.Slot1.kP = IntakeConstants.PivotConstants.kP_Up;

    pivotConfigs.Feedback.SensorToMechanismRatio = IntakeConstants.PivotConstants.GEAR_RATIO; 
    pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    //pivotConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    pivotConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    pivotConfigs.CurrentLimits.SupplyCurrentLimit = IntakeConstants.PivotConstants.SUPPLY_CURRENT_LIMIT;
    pivotConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    
    pivotConfigs.CurrentLimits.StatorCurrentLimit = IntakeConstants.PivotConstants.STATOR_CURRENT_LIMIT;
    pivotConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

    pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.PivotConstants.MAX_ROTATIONS;
    pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.PivotConstants.MIN_ROTATIONS;
    pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    this.setDefaultCommand(moveToPosition());
    m_pivot.setPosition(IntakeConstants.PivotConstants.INITIALIZE_ROTATIONS);
    targetRotation = getPivotPosition();
    m_pivot.getConfigurator().apply(pivotConfigs);


    rollerConfigs.Slot0.kV = IntakeConstants.RollerConstants.kV;
    rollerConfigs.Slot0.kP = IntakeConstants.RollerConstants.kP;
    rollerConfigs.Slot0.kS = IntakeConstants.RollerConstants.kS;

    rollerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    rollerConfigs.Voltage.PeakForwardVoltage = 11.0;
    rollerConfigs.Voltage.PeakReverseVoltage = -11.0;
    
    rollerConfigs.CurrentLimits.SupplyCurrentLimit = IntakeConstants.RollerConstants.SUPPLY_CURRENT_LIMIT;
    rollerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    
    rollerConfigs.CurrentLimits.StatorCurrentLimit = IntakeConstants.RollerConstants.STATOR_CURRENT_LIMIT;
    rollerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

    m_roller.getConfigurator().apply(rollerConfigs);
  }

  


  /**
   * Move the elevator up and down.
   * @param dutycycle [-1, 1] speed to set the elevator too.
   */
  public Command setPivotDutyCycle(double dutycycle) {

      return run(() -> m_pivot.setControl(new DutyCycleOut(dutycycle)))
        .finallyDo(() -> stopPivot()); 
  }

  public void stopPivot() {
    m_pivot.setControl(new DutyCycleOut(0));
    setTargetPosition(getPivotPosition());
  }

    private Command moveToPosition() {
    return run(() -> {
      double error = targetRotation - getPivotPosition();
      int slot;
      if (Math.abs(error) < 0.01) {
        slot = 0; 
      } else {
        slot = (error > 0) ? 1 : 0;
      }
      m_pivot.setControl(new PositionVoltage(targetRotation).withSlot(slot));
    });
  }

  private void setTargetPosition(double rotations){
    targetRotation = rotations;
  }

 public Command setPosition(double rotations) {
    return run(() -> {
        setTargetPosition(rotations);
    }).until(() -> Math.abs(getPivotPosition() - rotations) < 0.05);
  }

  public double getPivotPosition() {
    return m_pivot.getPosition().getValueAsDouble();
  }

  private void setCurrentPosToZero(){
    m_pivot.setPosition(0);
  }

  public Command setCurrentPosToZeroCommand(){
    return runOnce(() -> setCurrentPosToZero());
  }

  public Command setRollerVelocity(double velocityRPS) {
    return run(() -> m_roller.setControl(new VelocityVoltage(velocityRPS).withSlot(0)))
        .finallyDo(() -> stopRoller());
  }

  public void stopRoller() {
    m_roller.setControl(new VelocityVoltage(0));
  }
  
  // public Command deployAndRun() {
  //   return setPosition(IntakeConstants.PivotConstants.DEPLOY_ROTATIONS)
  //       .andThen(run(() -> m_roller.setControl(new DutyCycleOut(IntakeConstants.RollerConstants.INTAKE_SPEED))))
  //       .finallyDo(() -> {
  //           stopRoller();
  //       });
  // }

  private Command waitWithSubsystem(double seconds) {
    return run(() -> {}).withTimeout(seconds);
}

public Command oscillate() {
    return run(() -> {
        double target = IntakeConstants.PivotConstants.DEPLOY_ROTATIONS;
        double error = target - getPivotPosition();
        int slot = (error > 0) ? 1 : 0;
        m_pivot.setControl(new PositionVoltage(target).withSlot(slot));
    })
    .until(() -> Math.abs(getPivotPosition() - IntakeConstants.PivotConstants.DEPLOY_ROTATIONS_SHOOTING) < 0.05)
    .andThen(run(() -> {}).withTimeout(0.5))
    .andThen(run(() -> {
        double target = IntakeConstants.PivotConstants.RETRACT_ROTATIONS;
        double error = target - getPivotPosition();
        int slot = (error > 0) ? 1 : 0;
        m_pivot.setControl(new PositionVoltage(target).withSlot(slot));
    })
    .until(() -> Math.abs(getPivotPosition() - IntakeConstants.PivotConstants.RETRACT_ROTATIONS_SHOOTING) < 0.05))
    .andThen(run(() -> {}).withTimeout(0.5))
    .repeatedly();
}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Position", getPivotPosition());
    SmartDashboard.putNumber("Intake Target Position", targetRotation);
    //System.out.println(targetRotation);
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}