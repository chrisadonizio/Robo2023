package frc.robot.subsystems;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.Talon;


public class Claw extends SubsystemBase {
    private Talon claw_motor;
    //channel - The PWM channel that the Talon is attached to. 0-9 are on-board, 10-19 are on the MXP port
    public Claw(int channel){
        claw_motor = new Talon (channel);

    }


    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      System.out.println(claw_motor.get());
    }


    public void closeClaw(){
      claw_motor.setInverted(false);
      claw_motor.set(0.4);
    }


    public void openClaw(){
      claw_motor.setInverted(true);
      claw_motor.set(0.4);
    }


    public void stopClaw(){
      claw_motor.set(0);
    }
 
}