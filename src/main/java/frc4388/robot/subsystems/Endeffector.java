// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.ElevatorConstants;
import frc4388.robot.Constants.EndeffectorConstants;

public class Endeffector extends SubsystemBase {
  /** Creates a new Endefector. */
  private TalonFX endeffectorMotor;
  public Endeffector(TalonFX endffectorTalonFX) {
    endeffectorMotor = endffectorTalonFX;

    endeffectorMotor.setNeutralMode(NeutralModeValue.Brake);
    endeffectorMotor.getConfigurator().apply(EndeffectorConstants.ENDEFECTOR_PID);

  }

  public void PIDPosition(double position) {
    var request = new PositionVoltage(position);
    endeffectorMotor.setControl(request);
  }

  public void PIDTop() {
    PIDPosition(EndeffectorConstants.TOP);
  }

  public void PIDMiddle() {
    PIDPosition(EndeffectorConstants.MIDDLE);
  }

  public void PIDBottom() {
    PIDPosition(EndeffectorConstants.BOTTOM);
  }

  public void endEffectorStop() {
    endeffectorMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
