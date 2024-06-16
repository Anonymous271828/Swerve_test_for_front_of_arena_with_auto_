// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Intake_subsystem;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class intakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake_subsystem m_subsystem;
  private final Supplier<Double> speedFunction;
  private final double constantSpeed;
  private final Supplier<Double> input;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   *///, Supplier<Double> sensor
  public intakeCommand(Intake_subsystem subsystem, Supplier<Double> speedFunctionSupplier, double constantSpeed, Supplier<Double> sensor) {
    this.m_subsystem = subsystem;
    this.speedFunction = speedFunctionSupplier;
    this.constantSpeed = constantSpeed;
    this.input = sensor;
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
    m_subsystem.setMotorsCommand(realTimeSpeed);
    }
    else{
      m_subsystem.setMotorsCommand(constantSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setMotorsCommand(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false;
    if((input.get() < 0.3 && speedFunction.get() == 0) || (speedFunction.get() > 0.1 && constantSpeed != -100)){
    //System.out.println(input.get());
      return true;}
    else{
      //System.out.println(speedFunction.get() == 0);
      return false;
    }
  }
  
}

