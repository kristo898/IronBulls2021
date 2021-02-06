// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
    WPI_TalonSRX rightMaster;
    WPI_TalonSRX leftMaster;
    WPI_VictorSPX rightSlave;
    WPI_VictorSPX leftSlave;
    SpeedControllerGroup leftMotors;
    SpeedControllerGroup rightMotors;
    DifferentialDrive drive;
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    // Master Controllers
    leftMaster = new WPI_TalonSRX(Constants.leftMaster);
    rightMaster = new WPI_TalonSRX(Constants.rightMaster);
    // Slave Controllers
    leftSlave = new WPI_VictorSPX(Constants.leftSlave);
    rightSlave = new WPI_VictorSPX(Constants.rightSlave);
    rightMaster.configFactoryDefault();
    leftMaster.configFactoryDefault();
    leftSlave.configFactoryDefault();
    rightSlave.configFactoryDefault();
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    rightMaster.setInverted(false);
    leftMaster.setInverted(true);
    leftSlave.setInverted(true);
    rightSlave.setInverted(false);
    rightMotors = new SpeedControllerGroup(rightMaster, rightSlave);
    leftMotors = new SpeedControllerGroup(leftMaster, leftSlave);
    drive = new DifferentialDrive(leftMotors, rightMotors);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void driveWithJoystick(Joystick Joystick, double speed)
  {
    drive.arcadeDrive(Joystick.getRawAxis(Constants.left_X_Axis) * speed,Joystick.getRawAxis(Constants.left_Y_AXIS) * speed);
  }
  public void driveForward(double speed)
  {
    drive.tankDrive(speed, speed);
  }
  public void stop()
  {
    drive.stopMotor();
  }
}
