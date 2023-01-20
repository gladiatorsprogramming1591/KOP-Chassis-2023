// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  private final DifferentialDrive m_differentialDrive;
  private final Joystick m_driverJoystick;

  private double m_AxisForward = 0;
  private double m_AxisTurning = 0;

  public DriveTrain(DifferentialDrive differentialDrive, Joystick driverJoystick) {
    m_differentialDrive = differentialDrive;
    m_driverJoystick = driverJoystick;
    setDefaultCommand(new FastDrive(this));

    public void drive(double xSpeed, double zRotation, boolean squareInputs) {
      m_differentialDrive.arcadeDrive(xSpeed, zRotation, squareInputs);
    }

    public double getAxisForward() {
      return -m_driverJoystick.getY();
  }

  public double getAxisTurning() {
      return m_driverJoystick.getZ();
  }

  public abstract void setBrakeMode();

  public abstract void setCoastMode();

  @Override
  public void periodic() {
   //  drive(m_AxisForward, m_AxisTurning, Constants.kSlowSquaredInputs);
   m_differentialDrive.feedWatchdog();
 
   tx = table.getEntry("tx");
   ty = table.getEntry("ty");
   ta = table.getEntry("ta");
   tv = table.getEntry("tv");
 
   x = tx.getDouble(0.0);
   y = ty.getDouble(0.0);
   area = ta.getDouble(0.0);
   v = tv.getDouble(0.0);
  }
  }
}
