// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ChassisConstants.distanceConstants;
import frc.robot.commands.DefaultTeleopCommand;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.ElevatorLevelIntake;
import frc.robot.commands.AutoDriveForword;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.EndAccessorySubsystem;
import frc.robot.subsystems.AlgeaSubsystem;

import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.Util.ElevatorStates;
import frc.robot.Util.FieldLayout;
import frc.robot.subsystems.EndAccessorySubsystem.DropAngles;
import frc.robot.subsystems.ElevatorSubsystem;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ElevatorLevelScoreCMD;
import frc.robot.commands.RemoveAlgea;
import frc.robot.commands.RestElevatorAndGripper;
import frc.robot.commands.ShootingAlgeaCmd;
import frc.robot.commands.RestAlgea;
import frc.robot.Constants.ChassisConstants;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static final ChassisSubsystem chassisSubsystem = new ChassisSubsystem();

  // private final AlgeaSubsystem algeaSubsystem = new AlgeaSubsystem();

  // private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  private final EndAccessorySubsystem endAccessorySubsystem = new EndAccessorySubsystem();

  private final CommandXboxController xboxControllerDrive = new CommandXboxController(
      OperatorConstants.driverControllerPort);

  private final CommandXboxController buttonXboxController = new CommandXboxController(
      OperatorConstants.buttonControllerPort);

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {

    NamedCommands.registerCommand("drive to nearest right branch",
        new DriveToPoseCommand(chassisSubsystem, FieldLayout.getNearestBranchRight(chassisSubsystem.getPose())));
        
    NamedCommands.registerCommand("L1", new ElevatorLevelScoreCMD(elevatorSubsystem,
        endAccessorySubsystem, ElevatorStates.coralL1, DropAngles.setDropAngleL1));

    NamedCommands.registerCommand("L2", new ElevatorLevelScoreCMD(elevatorSubsystem,
        endAccessorySubsystem, ElevatorStates.coralL2, DropAngles.setDropAngleL2));

    NamedCommands.registerCommand("L3", new ElevatorLevelScoreCMD(elevatorSubsystem,
        endAccessorySubsystem, ElevatorStates.coralL3, DropAngles.setDropAngleL3));

    NamedCommands.registerCommand("Remove algea from low",
        (new RemoveAlgea(elevatorSubsystem, endAccessorySubsystem, true)));

    NamedCommands.registerCommand("Remove algea from high",
        (new RemoveAlgea(elevatorSubsystem, endAccessorySubsystem, false)));

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("EMPTY", null);
    autoChooser.addOption("Drive Forword 1 sec", new AutoDriveForword(chassisSubsystem));

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    setUpContollers();

  }

  // setting up 1 controller that does everything and if 2 are connected then it
  // splits it to two controllers:
  // first one to the chassis
  // the seconds one to the buttons

  private void setUpContollers() {
    if (xboxControllerDrive.isConnected()) {
      chassisSubsystem.setDefaultCommand(new DefaultTeleopCommand(chassisSubsystem,
          () -> xboxControllerDrive.getLeftY(),
          () -> xboxControllerDrive.getLeftX(),
          () -> -xboxControllerDrive.getRightX()));

      configureButtonBinding(xboxControllerDrive);
      if (xboxControllerDrive.isConnected() && buttonXboxController.isConnected()) {
        configureButtonBinding(buttonXboxController);
      }
    } else {
      chassisSubsystem.setDefaultCommand(new DefaultTeleopCommand(chassisSubsystem,
          () -> xboxControllerDrive.getLeftY(),
          () -> xboxControllerDrive.getLeftX(),
          () -> -xboxControllerDrive.getRightX()));

      configureButtonBinding(buttonXboxController);
    }
  }

  private void configureButtonBinding(CommandXboxController cmdXboxController) {

    cmdXboxController.leftBumper()
        .whileTrue(new ElevatorLevelIntake(elevatorSubsystem, endAccessorySubsystem, chassisSubsystem));
        
    cmdXboxController.rightBumper().whileTrue(new RestElevatorAndGripper(elevatorSubsystem, endAccessorySubsystem));
    xboxControllerDrive.start().onTrue(new InstantCommand(() -> chassisSubsystem.zeroHeading()));

    cmdXboxController.y().whileTrue(new ElevatorLevelScoreCMD(elevatorSubsystem,
        endAccessorySubsystem, ElevatorStates.coralL1, DropAngles.setDropAngleL1));
    cmdXboxController.b().whileTrue(new ElevatorLevelScoreCMD(elevatorSubsystem,
        endAccessorySubsystem, ElevatorStates.coralL2, DropAngles.setDropAngleL2));
    cmdXboxController.x().whileTrue(new ElevatorLevelScoreCMD(elevatorSubsystem,
        endAccessorySubsystem, ElevatorStates.coralL3, DropAngles.setDropAngleL3));

    cmdXboxController.rightTrigger().whileTrue(
        new DriveToPoseCommand(chassisSubsystem, FieldLayout.getNearestBranchRight(chassisSubsystem.getPose())));
    cmdXboxController.leftTrigger()
        .whileTrue(new DriveToPoseCommand(chassisSubsystem, FieldLayout.getNearestSource(chassisSubsystem.getPose())));

    cmdXboxController.povUp().whileTrue(new RemoveAlgea(elevatorSubsystem, endAccessorySubsystem, false));
    cmdXboxController.povDown().whileTrue(new RemoveAlgea(elevatorSubsystem, endAccessorySubsystem, true));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {

    return autoChooser.getSelected();
  }
}