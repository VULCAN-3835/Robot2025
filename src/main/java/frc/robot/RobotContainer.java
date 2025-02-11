// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Util.ElevatorStates;
import frc.robot.Util.FieldLayout;
import frc.robot.Util.FieldLayout.ReefSide;
import frc.robot.commands.ClimbCMD;
import frc.robot.commands.CloseClimbCMD;
import frc.robot.commands.CollectingAlgeaCmd;
import frc.robot.commands.CoralCollectCommand;
import frc.robot.commands.DefaultTeleopCommand;
import frc.robot.commands.ElevatorLevelIntake;
import frc.robot.commands.CoralReleaseCommand;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.EndAccessorySubsystem;
import frc.robot.subsystems.EndAccessorySubsystem.DropAngles;
import frc.robot.subsystems.AlgeaSubsystem;

import frc.robot.subsystems.ClimbSubsystem;

import frc.robot.subsystems.ElevatorSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ElevatorLevelScoreCMD;
import frc.robot.commands.RestElevatorAndGripper;
import frc.robot.commands.ShootingAlgeaCmd;

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
  FieldLayout.ReefSide currentSelectedSide = ReefSide.bottom;
  // The robot's subsystems and commands are defined here...
  private final ChassisSubsystem chassisSubsystem = new ChassisSubsystem();


  private final AlgeaSubsystem algeaSubsystem = new AlgeaSubsystem();

  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

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

    autoChooser = AutoBuilder.buildAutoChooser();

    autoChooser.setDefaultOption("EMPTY", null);


    SmartDashboard.putData("Auto Chooser", autoChooser);
    NamedCommands.registerCommand("score L4 left", new ElevatorLevelScoreCMD(chassisSubsystem, elevatorSubsystem,
        endAccessorySubsystem, ElevatorStates.coralL4, DropAngles.setDropAngleL4, false, false));
    NamedCommands.registerCommand("score L3 left", new ElevatorLevelScoreCMD(chassisSubsystem, elevatorSubsystem,
        endAccessorySubsystem, ElevatorStates.coralL3, DropAngles.setDropAngleL3, false, false));
    NamedCommands.registerCommand("score L2 left", new ElevatorLevelScoreCMD(chassisSubsystem, elevatorSubsystem,
        endAccessorySubsystem, ElevatorStates.coralL2, DropAngles.setDropAngleL2, false, false));
    NamedCommands.registerCommand("score L1 left", new ElevatorLevelScoreCMD(chassisSubsystem, elevatorSubsystem,
        endAccessorySubsystem, ElevatorStates.coralL1, DropAngles.setDropAngleL1, false, false));
    NamedCommands.registerCommand("intake from source", new ElevatorLevelScoreCMD(chassisSubsystem, elevatorSubsystem,
        endAccessorySubsystem, ElevatorStates.source, DropAngles.intakeAngle, false, true));

    NamedCommands.registerCommand("score L4 right", new ElevatorLevelScoreCMD(chassisSubsystem, elevatorSubsystem,
        endAccessorySubsystem, ElevatorStates.coralL4, DropAngles.setDropAngleL4, true, false));
    NamedCommands.registerCommand("score L3 right", new ElevatorLevelScoreCMD(chassisSubsystem, elevatorSubsystem,
        endAccessorySubsystem, ElevatorStates.coralL3, DropAngles.setDropAngleL3, true, false));
    NamedCommands.registerCommand("score L2 right", new ElevatorLevelScoreCMD(chassisSubsystem, elevatorSubsystem,
        endAccessorySubsystem, ElevatorStates.coralL2, DropAngles.setDropAngleL2, true, false));
    NamedCommands.registerCommand("score L1 right", new ElevatorLevelScoreCMD(chassisSubsystem, elevatorSubsystem,
        endAccessorySubsystem, ElevatorStates.coralL1, DropAngles.setDropAngleL1, true, false));

    configureBindings();
  }

  private void configureBindings() {
    setUpContollers();
  }

  private void setUpContollers() {
    if (xboxControllerDrive.isConnected()) {
      this.chassisSubsystem.setDefaultCommand(new DefaultTeleopCommand(this.chassisSubsystem,
          () -> -xboxControllerDrive.getLeftY(),
          () -> -xboxControllerDrive.getLeftX(),
          () -> -xboxControllerDrive.getRightX()));
          

      configureXboxBinding(xboxControllerDrive);
      if (xboxControllerDrive.isConnected() && buttonXboxController.isConnected()) {
        configureXboxBinding(buttonXboxController);
      }
      xboxControllerDrive.button(2).onTrue(new InstantCommand((() -> this.currentSelectedSide = ReefSide.bottomLeft)));
    } else {
      this.chassisSubsystem.setDefaultCommand(new DefaultTeleopCommand(this.chassisSubsystem,
          () -> -xboxControllerDrive.getLeftY(),
          () -> -xboxControllerDrive.getLeftX(),
          () -> -xboxControllerDrive.getRightX()));

      configureXboxBinding(buttonXboxController);
    }
  }

  private void configureXboxBinding(CommandXboxController cmdXboxController) {

    cmdXboxController.start().onTrue(new InstantCommand(() -> chassisSubsystem.zeroHeading()));

    cmdXboxController.b().whileTrue(new ElevatorLevelIntake(chassisSubsystem, elevatorSubsystem, endAccessorySubsystem));
    cmdXboxController.b().toggleOnFalse(new RestElevatorAndGripper(elevatorSubsystem, endAccessorySubsystem));

    
    
    cmdXboxController.rightBumper().whileTrue(new InstantCommand(()-> setRight(cmdXboxController)));
    cmdXboxController.leftBumper().whileTrue(new InstantCommand(()-> setLeft(cmdXboxController)));


    cmdXboxController.a().whileTrue(new ClimbCMD(climbSubsystem));
    cmdXboxController.y().whileTrue(new CloseClimbCMD(climbSubsystem));

    cmdXboxController.leftTrigger().whileTrue(new CollectingAlgeaCmd(algeaSubsystem));
    cmdXboxController.leftTrigger().toggleOnFalse(new InstantCommand(()-> algeaSubsystem.setRestAngle()));

    cmdXboxController.rightTrigger().whileTrue(new ShootingAlgeaCmd(algeaSubsystem));
    cmdXboxController.rightTrigger().toggleOnFalse(new InstantCommand(()-> algeaSubsystem.setRestAngle()));

  }

  private void setRight(CommandXboxController cmdXboxController) {

    // scores the coral on the right side of the reef in L4
    cmdXboxController.povUp()
        .whileTrue(new ElevatorLevelScoreCMD(chassisSubsystem, elevatorSubsystem, endAccessorySubsystem,
            ElevatorStates.coralL4, DropAngles.setDropAngleL4, true, false));
    cmdXboxController.povUp().toggleOnFalse(new RestElevatorAndGripper(elevatorSubsystem, endAccessorySubsystem));

    // scores the coral on the right side of the reef in L3
    cmdXboxController.povLeft()
        .whileTrue(new ElevatorLevelScoreCMD(chassisSubsystem, elevatorSubsystem, endAccessorySubsystem,
            ElevatorStates.coralL3, DropAngles.setDropAngleL3, true, false));
    cmdXboxController.povLeft().toggleOnFalse(new RestElevatorAndGripper(elevatorSubsystem, endAccessorySubsystem));

    // scores the coral on the right side of the reef in L2
    cmdXboxController.povRight()
        .whileTrue(new ElevatorLevelScoreCMD(chassisSubsystem, elevatorSubsystem, endAccessorySubsystem,
            ElevatorStates.coralL2, DropAngles.setDropAngleL2, true, false));
    cmdXboxController.povRight().toggleOnFalse(new RestElevatorAndGripper(elevatorSubsystem, endAccessorySubsystem));

    // scores the coral on the right side of the reef in L1
    cmdXboxController.povDown()
        .whileTrue(new ElevatorLevelScoreCMD(chassisSubsystem, elevatorSubsystem, endAccessorySubsystem,
            ElevatorStates.coralL1, DropAngles.setDropAngleL1, true, false));
    cmdXboxController.povDown().toggleOnFalse(new RestElevatorAndGripper(elevatorSubsystem, endAccessorySubsystem));
  }

  private void setLeft(CommandXboxController cmdXboxController) {

    // scores the coral on the right side of the reef in L4
    cmdXboxController.povUp()
        .whileTrue(new ElevatorLevelScoreCMD(chassisSubsystem, elevatorSubsystem, endAccessorySubsystem,
            ElevatorStates.coralL4, DropAngles.setDropAngleL4, false, false));
    cmdXboxController.povUp().toggleOnFalse(new RestElevatorAndGripper(elevatorSubsystem, endAccessorySubsystem));

    // scores the coral on the right side of the reef in L3
    cmdXboxController.povLeft()
        .whileTrue(new ElevatorLevelScoreCMD(chassisSubsystem, elevatorSubsystem, endAccessorySubsystem,
            ElevatorStates.coralL3, DropAngles.setDropAngleL3, false, false));
    cmdXboxController.povLeft().toggleOnFalse(new RestElevatorAndGripper(elevatorSubsystem, endAccessorySubsystem));

    // scores the coral on the right side of the reef in L2
    cmdXboxController.povRight()
        .whileTrue(new ElevatorLevelScoreCMD(chassisSubsystem, elevatorSubsystem, endAccessorySubsystem,
            ElevatorStates.coralL2, DropAngles.setDropAngleL2, false, false));
    cmdXboxController.povRight().toggleOnFalse(new RestElevatorAndGripper(elevatorSubsystem, endAccessorySubsystem));

    // scores the coral on the right side of the reef in L1
    cmdXboxController.povDown()
        .whileTrue(new ElevatorLevelScoreCMD(chassisSubsystem, elevatorSubsystem, endAccessorySubsystem,
            ElevatorStates.coralL1, DropAngles.setDropAngleL1, false, false));
    cmdXboxController.povDown().toggleOnFalse(new RestElevatorAndGripper(elevatorSubsystem, endAccessorySubsystem));
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
