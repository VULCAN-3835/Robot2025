// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Util.ElevatorStates;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ElevatorLevelScoreCMD;
import frc.robot.subsystems.EndAccessorySubsystem.DropAngles;

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
  private final ChassisSubsystem chassisSubsystem = new ChassisSubsystem();
  // private final EndAccessorySubsystem endAccessorySubsystem = new EndAccessorySubsystem();

  private final AlgeaSubsystem alageaSubsystem = new AlgeaSubsystem();

  ClimbSubsystem climbSubsystem;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final EndAccessorySubsystem endAccessorySubsystem = new EndAccessorySubsystem();

  private final CommandXboxController xboxControllerDrive = new CommandXboxController(
      OperatorConstants.driverController);

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser();

    autoChooser.setDefaultOption("EMPTY", null);
    SmartDashboard.putData("Auto Chooser", autoChooser);
    NamedCommands.registerCommand("score L4", new ElevatorLevelScoreCMD(elevatorSubsystem,endAccessorySubsystem,ElevatorStates.coralL4,DropAngles.setDropAngleL4));
    NamedCommands.registerCommand("score L3", new ElevatorLevelScoreCMD(elevatorSubsystem,endAccessorySubsystem,ElevatorStates.coralL3,DropAngles.setDropAngleL3));
    NamedCommands.registerCommand("score L2", new ElevatorLevelScoreCMD(elevatorSubsystem,endAccessorySubsystem,ElevatorStates.coralL2,DropAngles.setDropAngleL2));
    NamedCommands.registerCommand("score L1", new ElevatorLevelScoreCMD(elevatorSubsystem,endAccessorySubsystem,ElevatorStates.coralL1,DropAngles.setDropAngleL1));
    NamedCommands.registerCommand("intake from source", new ElevatorLevelIntake(elevatorSubsystem, endAccessorySubsystem));

    configureBindings();
  }

  private void configureBindings() {
    if (xboxControllerDrive.isConnected()) {
      this.chassisSubsystem.setDefaultCommand(new DefaultTeleopCommand(this.chassisSubsystem,
          () -> -xboxControllerDrive.getLeftY(),//could use the math.pow and 3
          () -> -xboxControllerDrive.getLeftX(),
          () -> -xboxControllerDrive.getRightX()));
    }
    // xboxControllerDrive.a().whileTrue(new CollectingAlageaCmd(alageaSubsystem));
    // xboxControllerDrive.a().toggleOnFalse(new InstantCommand(()-> alageaSubsystem.setRestAngle()));



    //start with reverse direction becuase gears are flipped
    // xboxControllerDrive.a().whileTrue(alageaSubsystem.sysIdDynamic(Direction.kForward));
    // xboxControllerDrive.b().whileTrue(alageaSubsystem.sysIdDynamic(Direction.kReverse));
    // xboxControllerDrive.y().whileTrue(alageaSubsystem.sysIdQuasistatic(Direction.kForward));
    // xboxControllerDrive.x().whileTrue(alageaSubsystem.sysIdQuasistatic(Direction.kReverse));

    xboxControllerDrive.leftBumper().whileTrue(new InstantCommand(()->elevatorSubsystem.setPower(0.1)));
    xboxControllerDrive.leftBumper().toggleOnFalse(new InstantCommand(()-> elevatorSubsystem.setPower(0)));

    xboxControllerDrive.rightBumper().whileTrue(new InstantCommand(()->elevatorSubsystem.setPower(-0.1)));
    xboxControllerDrive.rightBumper().toggleOnFalse(new InstantCommand(()-> elevatorSubsystem.setPower(0)));
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
