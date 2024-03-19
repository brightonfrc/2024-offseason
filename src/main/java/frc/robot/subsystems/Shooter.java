package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;




public class Shooter extends SubsystemBase {
  public VictorSPX leftShooter;
  public VictorSPX rightShooter;
  public Shooter(VictorSPX leftShooter, VictorSPX rightShooter){
    this.leftShooter=leftShooter;
    this.rightShooter=rightShooter;
  }
  public void shootAmp(){
    //setting it to 50% power for now
    leftShooter.set(ControlMode.PercentOutput, 0.5);
    rightShooter.set(ControlMode.PercentOutput, 0.5);
  }
  public void stopRunning(){
    leftShooter.set(ControlMode.PercentOutput,0);
    rightShooter.set(ControlMode.PercentOutput,0);
  }
}
