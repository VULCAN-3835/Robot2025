// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CollectorCommand;
import frc.robot.commands.DefaultTeleopCommand;
import frc.robot.commands.ReleasePieceCommand;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.EndAccessorySubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ChassisSubsystem chassisSubsystem = new ChassisSubsystem();
  private final EndAccessorySubsystem endAccessorySubsystem = new EndAccessorySubsystem();


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController xboxControllerDrive =
      new CommandXboxController(OperatorConstants.driverController);

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    autoChooser = AutoBuilder.buildAutoChooser();

    autoChooser.setDefaultOption("EMPTY", null);

    SmartDashboard.putData("Auto Chooser",autoChooser);
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    if(xboxControllerDrive.isConnected()){
      this.chassisSubsystem.setDefaultCommand(new DefaultTeleopCommand(this.chassisSubsystem,
       () -> -xboxControllerDrive.getLeftY(),
       () -> -xboxControllerDrive.getLeftX(),
       () -> -xboxControllerDrive.getRightX()));
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
  
  public Command createAndScheduleCommand(Command command) {
    CommandScheduler.getInstance().schedule(command);
    return command;
  }

  public Command CollectCommand() {
      return createAndScheduleCommand(new CollectorCommand(endAccessorySubsystem));
  }

  public Command ReleasePieceCommand() {
      return createAndScheduleCommand(new ReleasePieceCommand(endAccessorySubsystem));
  }

}
