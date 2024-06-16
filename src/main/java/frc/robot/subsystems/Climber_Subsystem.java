package frc.robot.subsystems;
import frc.robot.Constants.Climber_Constants;
import frc.robot.Constants.Shooter_Intake_constants;


import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber_Subsystem  extends SubsystemBase{
    Spark climber1 = new Spark(Climber_Constants.Climber1_port);
    Spark climber2 = new Spark(Climber_Constants.Climber2_port);
  /** Creates a new ExampleSubsystem. */
  public Climber_Subsystem() {
    climber1.setInverted(true);
  }
   
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void ClimberCommand(double ClimbDirect) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    climber1.set(ClimbDirect);
    climber2.set(ClimbDirect);
  }

  public void RightClimberCommand(double ClimbDirect) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    climber1.set(ClimbDirect);
  }

  public void LeftClimberCommand(double ClimbDirect) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    climber2.set(ClimbDirect);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public Command exampleMethodCommand() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'exampleMethodCommand'");
  }
}
