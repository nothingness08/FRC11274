// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.libs.LimelightHelpers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class LimelightSubsystem extends SubsystemBase {
  //private NetworkTable limelightTable;
  double x, y, area;
  NetworkTableEntry tx, ty, ta;
  boolean hasTarget;
  double heightOfTag = 0.591, heightOfCamera = 0.325; //meters, CHANGE HEIGHT OF CAMERA

  public LimelightSubsystem() {
    LimelightHelpers.setPipelineIndex("", 0);
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
  }

  double[] getDxDy(){
    double heightDifference = heightOfTag - heightOfCamera;
    double dy = heightDifference / Math.tan(Math.toRadians(y));
    double dx = dy * Math.tan(Math.toRadians(x));
    return new double[]{dx, dy};
  }

  @Override
  public void periodic() {
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);


    SmartDashboard.putNumber("Limelight TX", x);
    SmartDashboard.putNumber("Limelight TY", y);
    SmartDashboard.putNumber("Limelight DX", getDxDy()[0]);
    SmartDashboard.putNumber("Limelight DY", getDxDy()[1]);
  }
}