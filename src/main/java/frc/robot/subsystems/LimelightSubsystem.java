// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.libs.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
  //private NetworkTable limelightTable;
  double tx, ty, ta;
  boolean hasTarget;

  public LimelightSubsystem() {
    //limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    LimelightHelpers.setPipelineIndex("", 0);
  }

  @Override
  public void periodic() {
    tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
    ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees
    ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)
    hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?
  }
}
