// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.libs.LimelightHelpers;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase {
  //private NetworkTable limelightTable;
  private static final double TAG_HEIGHT = 0.591, CAMERA_HEIGHT = 0.325; //meters, CHANGE HEIGHT OF CAMERA
  private static final String LIMELIGHT_NAME = "limelight"; 

  private static NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME);

  public LimelightSubsystem() {
    LimelightHelpers.setPipelineIndex(LIMELIGHT_NAME, 0);
  }
  /**
   * Calculates the estimated X (sideways) and Y (forward) distance to the target.
   * @return A double array where index 0 is DX and index 1 is DY.
   */
  // public double[] getDxDy(){
  //   double tx = LimelightHelpers.getTX(LIMELIGHT_NAME);
  //   double ty = LimelightHelpers.getTY(LIMELIGHT_NAME);

  //   if (!LimelightHelpers.getTV(LIMELIGHT_NAME)) {
  //     return new double[]{0.0, 0.0}; // Return 0s if no target is found
  //   }
  //   double heightDifference = TAG_HEIGHT - CAMERA_HEIGHT;
  //   double dy = heightDifference / Math.tan(Math.toRadians(ty));
  //   double dx = dy * Math.tan(Math.toRadians(tx));
  //   return new double[]{dx, dy};
  // }

  public double getDoubleEntry(String key) {
    return limelightTable.getEntry(key).getDouble(0);
  }

  public boolean hasTarget() {
    return LimelightHelpers.getTV(LIMELIGHT_NAME);
  }

  @Override
  public void periodic() {
  }
}