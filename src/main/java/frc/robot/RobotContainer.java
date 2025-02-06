// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CoralCollectCommand;
import frc.robot.commands.DefaultTeleopCommand;
import frc.robot.commands.CoralReleaseCommand;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.EndAccessorySubsystem;
import frc.robot.commands.CollectingAlageaCmd;
import frc.robot.commands.DefaultTeleopCommand;
import frc.robot.commands.ShootingAlageaCmd;
import frc.robot.subsystems.AlageaSubsystem;
import frc.robot.commands.ClimbCMD;
import frc.robot.commands.CloseClimbCMD;
import frc.robot.Util.ElevatorStates;
import frc.robot.commands.DefaultTeleopCommand;
import frc.robot.commands.ResetClimbing;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.ClimbSubsystem;

import frc.robot.subsystems.ElevatorSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

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

  private final AlageaSubsystem alageaSubsystem = new AlageaSubsystem();
  private final Joystick joystic = new Joystick(0);

  ClimbSubsystem climbSubsystem;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

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
    xboxControllerDrive.a().whileTrue(alageaSubsystem.sysIdDynamic(Direction.kForward));
    xboxControllerDrive.b().whileTrue(alageaSubsystem.sysIdDynamic(Direction.kReverse));
    xboxControllerDrive.y().whileTrue(alageaSubsystem.sysIdQuasistatic(Direction.kForward));
    xboxControllerDrive.x().whileTrue(alageaSubsystem.sysIdQuasistatic(Direction.kReverse));

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
