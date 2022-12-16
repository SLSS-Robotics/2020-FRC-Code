/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem extends SubsystemBase {
  /**
   * Creates a new VisionSubsystem.
   */
  public Joystick joystick;
 
  
  double[] defaultValue = new double[3];
  private NetworkTableInstance table;
  private NetworkTable cameraTable;
  public NetworkTableEntry isDriverMode;
  private String cameraName = "Logitech";
  double curr_X_Distance;
  double curr_angle_offset;
  double distance;
  double area_at_1_meter;
  double currArea;
  Solenoid solenoid = new Solenoid(0);

  double camera_distance = 0.5; //m
  double hypot_offset = 0.0;
  double latency;
  public VisionSubsystem() {
    joystick = new Joystick(1);
    table = NetworkTableInstance.getDefault();
    cameraTable = table.getTable("chameleon-vision").getSubTable(cameraName);
    isDriverMode = cameraTable.getEntry("driver_mode");
    area_at_1_meter = 400;
  }

  @Override
  public void periodic() {
    latency = cameraTable.getEntry("latency").getDouble(0.0);
    curr_angle_offset = cameraTable.getEntry("targetYaw").getDouble(0.0);
    currArea = cameraTable.getEntry("targetArea").getDouble(0.0); 
    distance = 1 / (currArea * ((double)1 / (double)area_at_1_meter));
    //System.out.println(curr_X_Distance);
    SmartDashboard.putNumber("X offset", distance);
    SmartDashboard.putNumber("Angle offset", curr_angle_offset);
    SmartDashboard.putNumber("Latency ", latency);

  }

  public double x_offset(){
    return curr_X_Distance;
  }

  public double angle_offset(){
    return curr_angle_offset;
  }

  public void setDriverMode(boolean toggle){
    isDriverMode.setBoolean(toggle);
  }

  public void toggleSolenoid(boolean toggle){
    solenoid.set(toggle);
  }
  
}