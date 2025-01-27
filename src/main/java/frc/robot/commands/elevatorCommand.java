

package frc.robot.commands;



import edu.wpi.first.wpilibj2.command.InstantCommand;


import frc.robot.Util.ElevatorStates;
import frc.robot.subsystems.ElevatorSubsystem;

public final class ElevatorCommand {

  public static InstantCommand create(ElevatorSubsystem elevatorSubsystem , ElevatorStates elevatorStates) {
      return new InstantCommand(() -> elevatorSubsystem.setLevel(elevatorStates));
  }
}
