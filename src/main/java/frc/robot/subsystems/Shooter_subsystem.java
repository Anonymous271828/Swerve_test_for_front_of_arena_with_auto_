package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter_Intake_constants;

public class Shooter_subsystem extends SubsystemBase{
    // new motor and ExampleSubsystem
    Spark shooter = new Spark(Shooter_Intake_constants.shooter_motor_port); 
    CANSparkMax topWheelMotor = new CANSparkMax(10, MotorType.kBrushless);
    public Shooter_subsystem() {
      topWheelMotor.setSmartCurrentLimit(40);
    }
    
     /**
   * Example command factory method.
   *
   * @return a command
   */
  public void setMotorCommand(double speed){
    //System.out.println(speed);
    shooter.set(speed);
    topWheelMotor.set(-speed);  
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
    throw new UnsupportedOperationException("Unimplemented method 'exampleMethodCommand'");
  }
}
