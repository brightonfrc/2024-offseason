import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public class Motor 
{
    private VictorSP _victorSP;

    public Motor(VictorSP victorSP)
    {
        _victorSP = victorSP;
    }
    
    public void SetMotorPower(double Speed)
    {
        _victorSP.set(Speed);
    }
    public double GetMotorPower()
    {
        return _victorSP.get();
    }
}