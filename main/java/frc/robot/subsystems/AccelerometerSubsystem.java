package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.ADXL362;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;

public class AccelerometerSubsystem extends SubsystemBase{
    ADXL362 accels = new ADXL362(SPI.Port.kMXP, Accelerometer.Range.k8G);
    double xAccel;
    double yAccel;
    double zAccel;
    double lastXAccel;
    double lastYAccel;
    double lastZAccel;
    public AccelerometerSubsystem(){

    }

    @Override
    public void periodic(){
        lastXAccel = xAccel;
        lastYAccel = yAccel;
        lastZAccel = zAccel;
        xAccel = accels.getX();
        yAccel = accels.getY();
        zAccel = accels.getZ();
//Updates the acceleration values every 20ms
    }

    public double returnCurrentAcceleration(String a){
        if (a.equals("X")){
            return xAccel;
        }

        else if (a.equals("Y")){
            return yAccel;
        }

        return zAccel;
    }

    public double returnPreviousAcceleration(String a){
        if (a.equals("X")){
            return lastXAccel;
        }

        else if (a.equals("Y")){
            return lastYAccel;
        }

        return lastZAccel;
    }
}