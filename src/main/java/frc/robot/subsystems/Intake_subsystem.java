package frc.robot.subsystems;
import frc.robot.Constants.Shooter_Intake_constants;


import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake_subsystem  extends SubsystemBase{
    Spark shooter1 = new Spark(Shooter_Intake_constants.intake_motor_port);
  /** Creates a new ExampleSubsystem. */
  public Intake_subsystem() {}
   
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void setMotorsCommand(double speed) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    shooter1.set(speed);
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
