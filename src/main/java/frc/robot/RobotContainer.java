// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CollectingAlageaCmd;
import frc.robot.commands.DefaultTeleopCommand;
import frc.robot.commands.ShootingAlageaCmd;
import frc.robot.subsystems.AlageaSubsystem;
import frc.robot.commands.ClimbCMD;
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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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
  private final AlageaSubsystem alageaSubsystem = new AlageaSubsystem();
  private final Joystick joystic = new Joystick(0);
   private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  private final CommandXboxController xboxControllerDrive = new CommandXboxController(
      OperatorConstants.driverController);

  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  public static void RWhichLevel(){
    if(Constants.OperatorConstants.rightLevelCounter > 0 && Constants.OperatorConstants.rightLevelCounter < 6){
      System.out.println("rightLevel counter is in level1");
    }
    if(Constants.OperatorConstants.rightLevelCounter > 5 && Constants.OperatorConstants.rightLevelCounter < 11){
      System.out.println("rightLevelCounter is in level2");
    }
    if(Constants.OperatorConstants.rightLevelCounter > 10 && Constants.OperatorConstants.rightLevelCounter < 16){
      System.out.println("rightLevelCounter is in level3");
    }
    if(Constants.OperatorConstants.rightLevelCounter > 15 && Constants.OperatorConstants.rightLevelCounter < 21){
      System.out.println("rightLevelCounter is in level4");
    }
  }
  public static void LWhichLevel(){
     if(Constants.OperatorConstants.leftLevelCounter > 0 && Constants.OperatorConstants.leftLevelCounter < 6){
      System.out.println("leftLevelCounterr is in level1");
    }
    if(Constants.OperatorConstants.leftLevelCounter > 5 && Constants.OperatorConstants.leftLevelCounter < 11){
      System.out.println("leftLevelCounter is in level2");
    }
    if(Constants.OperatorConstants.leftLevelCounter > 10 && Constants.OperatorConstants.leftLevelCounter < 16){
      System.out.println("leftLevelCounter is in level3");
    }
    if(Constants.OperatorConstants.leftLevelCounter > 15 && Constants.OperatorConstants.leftLevelCounter < 21){
      System.out.println("leftLevelCounter is in level4");
    }
  }
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
          () -> -xboxControllerDrive.getLeftY(),
          () -> -xboxControllerDrive.getLeftX(),
          () -> -xboxControllerDrive.getRightX()));
    }
    xboxControllerDrive.rightBumper().onTrue(new InstantCommand(()-> Constants.OperatorConstants.rightLevelCounter++));
    xboxControllerDrive.rightTrigger().onTrue(new InstantCommand(()-> Constants.OperatorConstants.rightLevelCounter--));

    xboxControllerDrive.leftBumper().onTrue(new InstantCommand(()-> Constants.OperatorConstants.leftLevelCounter++));
    xboxControllerDrive.leftTrigger().onTrue(new InstantCommand(()-> Constants.OperatorConstants.leftLevelCounter--));
    
    xboxControllerDrive.leftBumper().toggleOnTrue(new InstantCommand()-> RWhichLevel());
    xboxControllerDrive.rightBumper().toggleOnTrue(new InstantCommand()-> LWhichLevel());

    xboxControllerDrive.b().whileTrue(new ShootingAlageaCmd(alageaSubsystem));
    xboxControllerDrive.x().whileTrue(new CollectingAlageaCmd(alageaSubsystem));

    xboxControllerDrive.y().toggleOnTrue(new ClimbCMD(climbSubsystem));
    xboxControllerDrive.b().toggleOnTrue(new ResetClimbing(climbSubsystem));

    xboxControllerDrive.b().toggleOnTrue(elevatorSubsystem.setLevelElevatorCommand(ElevatorStates.coralL1));
    xboxControllerDrive.a().toggleOnTrue(elevatorSubsystem.setLevelElevatorCommand(ElevatorStates.coralL2));
    xboxControllerDrive.x().toggleOnTrue(elevatorSubsystem.setLevelElevatorCommand(ElevatorStates.coralL3));
    xboxControllerDrive.leftStick().toggleOnTrue(elevatorSubsystem.setLevelElevatorCommand(ElevatorStates.coralL4));
    xboxControllerDrive.y().toggleOnTrue(elevatorSubsystem.setLevelElevatorCommand(ElevatorStates.rest));
    xboxControllerDrive.rightStick().toggleOnTrue(elevatorSubsystem.setLevelElevatorCommand(ElevatorStates.source));
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
