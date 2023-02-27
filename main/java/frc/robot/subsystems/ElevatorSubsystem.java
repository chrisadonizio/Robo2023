package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;


public class ElevatorSubsystem extends SubsystemBase {
private Spark elevator_motor;
  /** Creates a new ExampleSubsystem. */
  public ElevatorSubsystem(int channel)
  {      elevator_motor = new Spark(channel);
  }

  public void ascend(){
        elevator_motor.setInverted(false);
        elevator_motor.set(.4);
  }

  public void descend(){
        elevator_motor.setInverted(true);
        elevator_motor.set(.4);
  }

  public void condescend(){
      elevator_motor.stopMotor();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}