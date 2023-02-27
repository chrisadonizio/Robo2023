package frc.robot.commands;


import frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj2.command.CommandBase;


/** An example command that uses an example subsystem. */
public class MoveClaw extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Claw m_subsystem;
  public int state;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveClaw(Claw subsystem, int state) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(state == 1){
        m_subsystem.openClaw();
      }
      else if (state == 2) {
          m_subsystem.closeClaw(); 
        }
        else{
            m_subsystem.stopClaw();
        }
  }
}
