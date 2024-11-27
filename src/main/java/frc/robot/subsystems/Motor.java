package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Motor extends SubsystemBase {
    private VictorSP _visctorSP;

    public Motor(VictorSP victorSP)
    {
        _visctorSP = victorSP;
    }

    public void SetMotorPower(double Speed)
    {
        _visctorSP.set(Speed);
    }
    public double GetMotorPower()
    {
        return _visctorSP.get();
    }

}
