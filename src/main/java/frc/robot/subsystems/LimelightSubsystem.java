// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.libs.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
  private static final double TAG_HEIGHT = 0.591, CAMERA_HEIGHT = 0.35; //meters
  private static final String LIMELIGHT_NAME = "limelight"; 

  public LimelightSubsystem() {
    LimelightHelpers.setPipelineIndex(LIMELIGHT_NAME, 0);
  }
  /**
   * Calculates the estimated X (sideways) and Y (forward) distance to the target.
   * @return A double array where index 0 is DX and index 1 is DY.
   */
  public double[] getDxDy(){
    double tx = LimelightHelpers.getTX(LIMELIGHT_NAME);
    double ty = LimelightHelpers.getTY(LIMELIGHT_NAME);

    if (!LimelightHelpers.getTV(LIMELIGHT_NAME)) {
      return new double[]{0.0, 0.0}; // Return 0s if no target is found
    }
    double heightDifference = TAG_HEIGHT - CAMERA_HEIGHT;
    double dy = heightDifference / Math.tan(Math.toRadians(ty));
    double dx = dy * Math.tan(Math.toRadians(tx));
    return new double[]{dx, dy};
  }

  public boolean getTV(){
    return LimelightHelpers.getTV(LIMELIGHT_NAME);
  }

  public double[] getBotPose_TargetSpace(){
    double[] botPose_TargetSpace = LimelightHelpers.getBotPose_TargetSpace(LIMELIGHT_NAME);
    return botPose_TargetSpace;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("TV", LimelightHelpers.getTV(LIMELIGHT_NAME));
    SmartDashboard.putNumber("TX", LimelightHelpers.getTX(LIMELIGHT_NAME));
    SmartDashboard.putNumber("TY", LimelightHelpers.getTY(LIMELIGHT_NAME));

    double[] dxdy = getDxDy();
    SmartDashboard.putNumber("DX", dxdy[0]);
    SmartDashboard.putNumber("DY", dxdy[1]);
    //System.out.println("Limelight TX: " + LimelightHelpers.getTX(LIMELIGHT_NAME));
  }
}