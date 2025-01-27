// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Util.ElevatorStates;
import frc.robot.commands.DefaultTeleopCommand;
import frc.robot.commands.ElevatorCommand;

import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;



import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ChassisSubsystem chassisSubsystem = new ChassisSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ElevatorStates elevatorStatescoralL1 =  ElevatorStates.coralL1 ;
  private final ElevatorStates elevatorStatescoralL2 =  ElevatorStates.coralL2 ;
  private final ElevatorStates elevatorStatescoralL3 =  ElevatorStates.coralL3 ;
  private final ElevatorStates elevatorStatescoralL4 =  ElevatorStates.coralL4 ;
  private final ElevatorStates elevatorStatesRest =  ElevatorStates.rest ;
  private final ElevatorStates elevatorStatesSource =  ElevatorStates.source ;

  private final CommandXboxController xboxControllerDrive =
      new CommandXboxController(OperatorConstants.driverController);

  private SendableChooser<Command> autoChooser = new SendableChooser<>();


  public RobotContainer() {
    
    
    autoChooser = AutoBuilder.buildAutoChooser();

    autoChooser.setDefaultOption("EMPTY", null);

    SmartDashboard.putData("Auto Chooser",autoChooser);
    configureBindings();
  }

  
  private void configureBindings() {
    if(xboxControllerDrive.isConnected()){
      this.chassisSubsystem.setDefaultCommand(new DefaultTeleopCommand(this.chassisSubsystem,
       () -> -xboxControllerDrive.getLeftY(),
       () -> -xboxControllerDrive.getLeftX(),
       () -> -xboxControllerDrive.getRightX()));
    }
   
    xboxControllerDrive.b().whileTrue( ElevatorCommand.create(elevatorSubsystem, elevatorStatescoralL1 ));
    xboxControllerDrive.a().whileTrue( ElevatorCommand.create(elevatorSubsystem, elevatorStatescoralL2 ));
    xboxControllerDrive.x().whileTrue( ElevatorCommand.create(elevatorSubsystem, elevatorStatescoralL3 ));
    xboxControllerDrive.leftStick().whileTrue( ElevatorCommand.create(elevatorSubsystem, elevatorStatescoralL4 ));
    xboxControllerDrive.y().whileTrue( ElevatorCommand.create(elevatorSubsystem, elevatorStatesRest ));
    xboxControllerDrive.rightStick().whileTrue( ElevatorCommand.create(elevatorSubsystem, elevatorStatesSource ));

  }

 
  public Command getAutonomousCommand() {

    return autoChooser.getSelected();
  }
}
