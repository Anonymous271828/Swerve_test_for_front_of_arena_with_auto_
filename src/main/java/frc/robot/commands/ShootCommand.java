// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Shooter_subsystem;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShootCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter_subsystem m_subsystem;
  private final Supplier<Double> speedFunction;
  private final double constantSpeed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootCommand(Shooter_subsystem subsystem, Supplier<Double> speedFunction, double constantSpeed) {
    this.m_subsystem = subsystem;
    this.speedFunction = speedFunction;
    this.constantSpeed = constantSpeed;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (constantSpeed == -100){
    double realTimeSpeed = speedFunction.get();
    m_subsystem.setMotorCommand(realTimeSpeed);
    }
    else{
    m_subsystem.setMotorCommand(constantSpeed);
    }
  }

  public void variableShooter(){

  }

  public void setShooter(){
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setMotorCommand(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
