package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GyroscopeSubsystem extends SubsystemBase {

    private AHRS navx;

    public GyroscopeSubsystem() {
        navx = new AHRS(SPI.Port.kMXP);

    }

    public double getBearing() {
        return navx.getFusedHeading();
    }
    
}

