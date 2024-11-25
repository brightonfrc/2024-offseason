// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkAbsoluteEncoder.Type;

public class Lift extends SubsystemBase {
  /** Creates a new Lift. */
  private final CANSparkMax motor;
  private final AbsoluteEncoder encoder;
  
  
  public Lift(CANSparkMax inMotor) {
    this.motor = inMotor;
    this.encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    this.motor.setIdleMode(IdleMode.kBrake);
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}

