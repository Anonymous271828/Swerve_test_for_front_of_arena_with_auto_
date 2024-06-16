// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Climber_Subsystem;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ClimbCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Climber_Subsystem m_subsystem;
  private boolean GoUp;
  private Supplier<Double> axisNum;
  private double motorPos;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClimbCommand(Climber_Subsystem subsystem, boolean GoUp, Supplier<Double> axis, double motor) {
    this.m_subsystem = subsystem;;
    this.GoUp = GoUp;
    this.axisNum = axis;
    this.motorPos = motor;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double axis = axisNum.get();
    if (motorPos == -100){
      if (GoUp == true){
      m_subsystem.ClimberCommand(.5);
      }
      if (GoUp == false) {
      m_subsystem.ClimberCommand(-.5);
      }}
    else{
      if (motorPos == 1){
        m_subsystem.RightClimberCommand(axis);
      }
      else{
        m_subsystem.LeftClimberCommand(axis);
      }
    }
  }

  public void variableShooter(){

  }

  public void setShooter(){
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.ClimberCommand(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
