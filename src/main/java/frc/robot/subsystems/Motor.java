package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Motor extends SubsystemBase {

    private VictorSP victorSP;

    public double GetPower(){
        return victorSP.get();
    }

    public void SetPower(double speed){
        victorSP.set(speed);
    }

    public Motor(VictorSP VictorSPMotor){
        victorSP = VictorSPMotor;
    }
}