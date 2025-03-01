// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  public TalonFX climberMotor;

  public Climber(TalonFX climberTalonFX) {
    climberMotor = climberTalonFX;
    climberMotor.setNeutralMode(NeutralModeValue.Brake);

  }

  public void stopClimber(){
    climberMotor.set(0);
  }


  public void climberOut(){
    climberMotor.set(ClimberConstants.CLIMBER_SPEED);

  }

  public void climberIn(){
    climberMotor.set(-ClimberConstants.CLIMBER_SPEED);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
