// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pneumatics extends SubsystemBase {
  Compressor compressor;
  DoubleSolenoid Intake12;
  /** Creates a new Pnmatics. */
  public Pneumatics() {
    Compressor compressor = new Compressor();
    DoubleSolenoid Intake12 = new DoubleSolenoid(Constants.INTAKEFORWARD12, Constants.INTAKEBACKWARD12);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    compressor.start();
  }
  public void BackIntake12Pneumatics(Value kforward)
  {
    
  }
}
