// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlageaSubsystem;
import edu.wpi.first.wpilibj.Timer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CharacterizationCommand extends Command {
  private final Timer timer = new Timer();
  private final AlageaSubsystem alageaSubsystem; 
    private boolean isDynamicTest;
    private double voltage = 0;
  /** Creates a new CharacterizationCommand.
   *  
   * @param alageaSubsystem 
   * @param isDynamicTest  */
  public CharacterizationCommand(AlageaSubsystem alageaSubsystem, boolean isDynamicTest) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.alageaSubsystem = alageaSubsystem;
    this.isDynamicTest = isDynamicTest;
    addRequirements(alageaSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    voltage = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isDynamicTest) {
      // Gradually increase voltage over time
      voltage = timer.get() * 1.0; // Increase by 1V per second
      voltage = Math.min(voltage, 12.0); // Cap at 12V
  } else {
      // Keep voltage constant (e.g., 7V for static test)
      voltage = 7.0;
  }

  // Apply voltage to the subsystem
  alageaSubsystem.setPower(voltage / 12.0);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    alageaSubsystem.setPower(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 5.0;
  }
}
