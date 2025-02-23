// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ChassisConstants.distanceConstants;
import frc.robot.commands.DefaultTeleopCommand;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.ElevatorLevelIntake;
import frc.robot.commands.CollectingAlgeaCmd;
import frc.robot.commands.CoralReleaseCommand;
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

import java.util.logging.FileHandler;

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

    

    autoChooser = AutoBuilder.buildAutoChooser();

    //nearest source commadn
    // NamedCommands.registerCommand("drive to nearest source", new DriveToPoseCommand(chassisSubsystem,
    //     FieldLayout.getCoralSourcePose(ChassisConstants.distanceConstants.source, chassisSubsystem.getPose())));

    // // Bottom side commands
    // NamedCommands.registerCommand("drive to bottom reef right ",
    //     new DriveToPoseCommand(chassisSubsystem, FieldLayout.getBranchPose(ReefSide.bottom, true,
    //         ChassisConstants.distanceConstants.bottomReefDistance)));
    // NamedCommands.registerCommand("drive to bottom reef left ",
    //     new DriveToPoseCommand(chassisSubsystem, FieldLayout.getBranchPose(ReefSide.bottom, false,
    //         ChassisConstants.distanceConstants.bottomReefDistance)));

    // // Bottom Right side commands
    // NamedCommands.registerCommand("drive to bottom right reef right ",
    //     new DriveToPoseCommand(chassisSubsystem, FieldLayout.getBranchPose(ReefSide.bottomRight, true,
    //         ChassisConstants.distanceConstants.bottomRightReefDistance)));
    // NamedCommands.registerCommand("drive to bottom right reef left ",
    //     new DriveToPoseCommand(chassisSubsystem, FieldLayout.getBranchPose(ReefSide.bottomRight, false,
    //         ChassisConstants.distanceConstants.bottomRightReefDistance)));

    // // Top Right side commands
    // NamedCommands.registerCommand("drive to top right reef right ",
    //     new DriveToPoseCommand(chassisSubsystem, FieldLayout.getBranchPose(ReefSide.topRight, true,
    //         ChassisConstants.distanceConstants.topRightReefDistance)));
    // NamedCommands.registerCommand("drive to top right reef left ",
    //     new DriveToPoseCommand(chassisSubsystem, FieldLayout.getBranchPose(ReefSide.topRight, false,
    //         ChassisConstants.distanceConstants.topRightReefDistance)));

    // // Top side commands
    // NamedCommands.registerCommand("drive to top reef right ",
    //     new DriveToPoseCommand(chassisSubsystem, FieldLayout.getBranchPose(ReefSide.top, true,
    //         ChassisConstants.distanceConstants.topReefDistance)));
    // NamedCommands.registerCommand("drive to top reef left ",
    //     new DriveToPoseCommand(chassisSubsystem, FieldLayout.getBranchPose(ReefSide.top, false,
    //         ChassisConstants.distanceConstants.topReefDistance)));

    // // Top Left side commands
    // NamedCommands.registerCommand("drive to top left reef right ",
    //     new DriveToPoseCommand(chassisSubsystem, FieldLayout.getBranchPose(ReefSide.topLeft, true,
    //         ChassisConstants.distanceConstants.topLeftReefDistance)));
    // NamedCommands.registerCommand("drive to top left reef left ",
    //     new DriveToPoseCommand(chassisSubsystem, FieldLayout.getBranchPose(ReefSide.topLeft, false,
    //         ChassisConstants.distanceConstants.topLeftReefDistance)));

    // // Bottom Left side commands
    // NamedCommands.registerCommand("drive to bottom left reef right ",
    //     new DriveToPoseCommand(chassisSubsystem, FieldLayout.getBranchPose(ReefSide.bottomLeft, true,
    //         ChassisConstants.distanceConstants.bottomLeftReefDistance)));

    // NamedCommands.registerCommand("drive to bottom left reef left ",
    //     new DriveToPoseCommand(chassisSubsystem, FieldLayout.getBranchPose(ReefSide.bottomLeft, false,
    //         ChassisConstants.distanceConstants.bottomLeftReefDistance)));

    autoChooser.setDefaultOption("EMPTY", null);

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
        configureDriveController(xboxControllerDrive);
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

    // cmdXboxController.y().whileTrue(new ElevatorLevelScoreCMD(elevatorSubsystem,
    // endAccessorySubsystem, ElevatorStates.coralL1, DropAngles.setDropAngleL1));
    // cmdXboxController.b().whileTrue(new ElevatorLevelScoreCMD(elevatorSubsystem,
    // endAccessorySubsystem, ElevatorStates.coralL2, DropAngles.setDropAngleL2));
    // cmdXboxController.x().whileTrue(new ElevatorLevelScoreCMD(elevatorSubsystem,
    // endAccessorySubsystem, ElevatorStates.coralL3, DropAngles.setDropAngleL3));
    // cmdXboxController.a().whileTrue(new ElevatorLevelScoreCMD(elevatorSubsystem,
    // endAccessorySubsystem, ElevatorStates.coralL4, DropAngles.setDropAngleL4));

    // cmdXboxController.leftTrigger().whileTrue(new RemoveAlgea(elevatorSubsystem,
    // endAccessorySubsystem, false));
    // cmdXboxController.rightTrigger().whileTrue(new RemoveAlgea(elevatorSubsystem,
    // endAccessorySubsystem, true));
    // cmdXboxController.leftBumper().whileTrue(new
    // ElevatorLevelIntake(elevatorSubsystem, endAccessorySubsystem));

    // cmdXboxController.rightBumper().whileTrue(new
    // RestElevatorAndGripper(elevatorSubsystem, endAccessorySubsystem));

    // cmdXboxController.a().whileTrue(new CollectingAlgeaCmd(algeaSubsystem));
    // cmdXboxController.a().toggleOnFalse(new RestAlgea(algeaSubsystem));

    // cmdXboxController.b().whileTrue(new ShootingAlgeaCmd(algeaSubsystem));
    // cmdXboxController.b().toggleOnFalse(new RestAlgea(algeaSubsystem));

    cmdXboxController.leftBumper().whileTrue(new ElevatorLevelIntake(elevatorSubsystem,endAccessorySubsystem,chassisSubsystem));
    cmdXboxController.rightBumper().whileTrue(new RestElevatorAndGripper(elevatorSubsystem, endAccessorySubsystem));
    xboxControllerDrive.start().onTrue(new InstantCommand(() -> chassisSubsystem.zeroHeading()));

    cmdXboxController.y().whileTrue(new ElevatorLevelScoreCMD(elevatorSubsystem,
    endAccessorySubsystem, ElevatorStates.coralL1, DropAngles.setDropAngleL1));
    cmdXboxController.b().whileTrue(new ElevatorLevelScoreCMD(elevatorSubsystem,
    endAccessorySubsystem, ElevatorStates.coralL2, DropAngles.setDropAngleL2));
    cmdXboxController.x().whileTrue(new ElevatorLevelScoreCMD(elevatorSubsystem,
    endAccessorySubsystem, ElevatorStates.coralL3, DropAngles.setDropAngleL3));
    cmdXboxController.a().whileTrue(new ElevatorLevelScoreCMD(elevatorSubsystem,
    endAccessorySubsystem, ElevatorStates.coralL4, DropAngles.setDropAngleL4));

    cmdXboxController.povUp().whileTrue(new DriveToPoseCommand(chassisSubsystem, FieldLayout.getNearestBranchPose(chassisSubsystem.getPose(), false)));

    // cmdXboxController.a().whileTrue(new DriveToPoseCommand(chassisSubsystem, FieldLayout.getMaybeBottomPose()));

    // cmdXboxController.y().whileTrue(new ElevatorLevelScoreCMD(elevatorSubsystem,
    // endAccessorySubsystem, ElevatorStates.coralL1, DropAngles.setDropAngleL1));
    // cmdXboxController.a().whileTrue(new DriveToPoseCommand(chassisSubsystem, FieldLayout.getBranchPose(ReefSide.BOTTOM,true,Centimeters.of(-10))));
    // cmdXboxController.b().whileTrue(new ElevatorLevelScoreCMD(elevatorSubsystem, endAccessorySubsystem, ElevatorStates.coralL1, DropAngles.setDropAngleL1));
    // cmdXboxController.rightBumper().whileTrue(new DriveToPoseCommand(chassisSubsystem, FieldLayout.getNearestBranch(chassisSubsystem.getPose(), true)));


    // cmdXboxController.a().whileTrue(new DriveToPoseCommand(chassisSubsystem,
    //     FieldLayout.getCoralSourcePose(Centimeters.of(30), chassisSubsystem.getPose())));

  }



  private void configureDriveController(CommandXboxController cmdXboxController){
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
