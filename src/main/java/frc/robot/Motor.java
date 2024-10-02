// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix.motorcontrol.can.VictorSPX; 
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

/** Add your docs here. */
public class Motor {
    private double power; 
    private VictorSPX motor; 
    public Motor(double power, VictorSPX motor){
        this.power = power;
        this.motor = motor;
    }
    public double GetPower(){
        return power 
    }
    public void SetPower(double power){
        motor.set(VictorSPXControlMode.PercentOutput, power)
        this.power = power;
    }
}
