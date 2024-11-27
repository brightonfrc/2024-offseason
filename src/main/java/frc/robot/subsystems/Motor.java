package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Motor extends SubsystemBase {

    private VictorSPX victorSPX;

    public double GetPower(){
        //return victorSPX.get();
        return victorSPX.getMotorOutputPercent();
    }

    public void SetPower(double speed){
        victorSPX.set(VictorSPXControlMode.PercentOutput, speed);
        //victorSPX.set(speed);
    }

    public Motor(VictorSPX VictorSPXMotor){
        victorSPX = VictorSPXMotor;
    }
}