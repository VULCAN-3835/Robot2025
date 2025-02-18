// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Collection;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants.EndAccessoryConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogInput;

public class EndAccessorySubsystem extends SubsystemBase {

    private TalonFX angleMotor;
    private TalonFX powerMotor;
    private Orchestra sound;

    private DutyCycleEncoder angleEncoder;
    private AnalogInput pieceDetector;

    private Timer timer;

    private ProfiledPIDController profiledPIDController;
    private Constraints constraints;

    // the SysID routine
    private SysIdRoutine sysIdRoutine;
    private Config config;

    public EndAccessorySubsystem() {
        this.angleMotor = new TalonFX(EndAccessoryConstants.angleMotorID);
        this.powerMotor = new TalonFX(EndAccessoryConstants.powerMotorID);
        this.sound = new Orchestra();
        sound.addInstrument(powerMotor);
        sound.loadMusic("src/main/deploy/output.chrp");

        this.timer = new Timer();

        this.angleEncoder = new DutyCycleEncoder(EndAccessoryConstants.angleEncoderID);
        angleEncoder.setInverted(true);
        
        // angleEncoder.setDutyCycleRange(EndAccessoryConstants.kMinAngle.in(Rotations), EndAccessoryConstants.kMaxAngle.in(Rotations));
        this.pieceDetector = new AnalogInput(EndAccessoryConstants.pieceDetectorID);

        this.constraints = new Constraints(EndAccessoryConstants.maxVelocity, EndAccessoryConstants.maxAcceleration);
        this.profiledPIDController = new ProfiledPIDController(EndAccessoryConstants.ProfiledkP,
                EndAccessoryConstants.profiledkI,
                EndAccessoryConstants.profiledkD, constraints);
        profiledPIDController.setGoal(82);

        
        // the SysID configs, for explanation on it go to the elevator subsystem in lines 77-79
        this.config = new Config(Volts.of(1).per(Seconds), Volts.of(2), Seconds.of(6));
        this.sysIdRoutine = new SysIdRoutine(config,
            new SysIdRoutine.Mechanism(this::setVoltage,
            Log -> {
                Log.motor("Angle Motor")
                .voltage(angleMotor.getMotorVoltage().getValue())
                .angularPosition(Degrees.of(getAngle().in(Degrees)))
                .angularVelocity(DegreesPerSecond.of(angleMotor.getVelocity().getValue().in(DegreesPerSecond)));
                },
            this));

        profiledPIDController.setTolerance(EndAccessoryConstants.armAngleTolerence.in(Degree));
    }

    private void setVoltage(Voltage volts) {
        angleMotor.setVoltage(volts.in(Volts));
    }

    //sysID command for the quasistatic tests
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return this.sysIdRoutine.quasistatic(direction);
    }

    //sysID command for the Dynamic tests
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return this.sysIdRoutine.dynamic(direction);
    }

    public enum DropAngles {
        setDropAngleL1, setDropAngleL2, setDropAngleL3, setDropAngleL4, restingAngle, intakeAngle;
    }

    public WaitUntilCommand waitForCoral() {
        return new WaitUntilCommand(() -> hasPiece());
    }

    public void gripperIntake() {
        powerMotor.set(EndAccessoryConstants.kMotorSpeedIntake);
    }

    public void gripperRelease(DropAngles angle){
        switch (angle) {
            case setDropAngleL1:
                powerMotor.set(EndAccessoryConstants.kMotorSpeedL1);    
            case setDropAngleL2:
                powerMotor.set(EndAccessoryConstants.kMotorSpeedL2);            
            case setDropAngleL3:
                powerMotor.set(EndAccessoryConstants.kMotorSpeedL3);    
            case setDropAngleL4:
                powerMotor.set(EndAccessoryConstants.kMotorSpeedL4);      
        }
    }
    public void gripperReleaseL1() {
        powerMotor.set(EndAccessoryConstants.kMotorSpeedL1);
    }
    public void gripperReleaseL2() {
        powerMotor.set(EndAccessoryConstants.kMotorSpeedL2);
    }    
    public void gripperReleaseL3() {
        powerMotor.set(EndAccessoryConstants.kMotorSpeedL3);
    } 
    public void gripperReleaseL4() {
        powerMotor.set(EndAccessoryConstants.kMotorSpeedL4);
    } 

    public void gripperStop() {
        powerMotor.set(0);
    }

    public void setSpeed(DropAngles dropingLevel){

    }

    public void setAngle(DropAngles dropingLevel) {
        switch (dropingLevel) {
            case setDropAngleL1:
                profiledPIDController.setGoal(EndAccessoryConstants.targetDropAngleL1.in(Degree));
                break;
            case setDropAngleL2:
                profiledPIDController.setGoal(EndAccessoryConstants.targetDropAngleL2.in(Degree));
                break;
            case setDropAngleL3:
                profiledPIDController.setGoal(EndAccessoryConstants.targetDropAngleL3.in(Degree));
                break;
            case setDropAngleL4:
                profiledPIDController.setGoal(EndAccessoryConstants.targetDropAngleL4.in(Degree));
                break;
            case restingAngle:
                profiledPIDController.setGoal(EndAccessoryConstants.targetAngleRest.in(Degree));
                break;
            case intakeAngle:
                profiledPIDController.setGoal(EndAccessoryConstants.targetIntakeAngle.in(Degree));
        }
    }

    public Angle getAngle() {
        return Rotations.of(angleEncoder.get());
    }

    public boolean hasPiece() {
        return pieceDetector.getVoltage() > EndAccessoryConstants.kHasPieceVoltageThreshold;

    }

    public boolean isAtSetpoint() {
        return Degrees.of(profiledPIDController.getSetpoint().position).minus(getAngle()).abs(Degrees) < EndAccessoryConstants.armAngleTolerence.in(Degrees); 
    }
    public boolean isAtPidSetpoint(){
        return profiledPIDController.atGoal();
    }

    // end accessory SysID to use this paste it in the configureXboxBinding method in robot container
    // xboxControllerDrive.a().whileTrue(endAccessorySubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // xboxControllerDrive.b().whileTrue(endAccessorySubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // xboxControllerDrive.y().whileTrue(endAccessorySubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // xboxControllerDrive.x().whileTrue(endAccessorySubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));


    @Override
    public void periodic() {

        
        if (RobotController.getBatteryVoltage()<12) {
            sound.play();
        } else{
            sound.pause();
            sound.stop();
        }

        double profiledPIDOutput = profiledPIDController.calculate(getAngle().in(Degrees));


        if (hasPiece()&&profiledPIDController.getGoal().position==EndAccessoryConstants.targetIntakeAngle.in(Degree)) {
            gripperStop();
        }

        if (getAngle().in(Degrees)<EndAccessoryConstants.kMinAngle.in(Degrees) && profiledPIDOutput<0) {
            angleMotor.set(0);
            System.out.println("error low");
        }
        if(getAngle().in(Degrees)<EndAccessoryConstants.kMaxAngle.in(Degrees) && profiledPIDOutput>0){
            angleMotor.set(0);
            System.out.println("error high ");
        }
        angleMotor.set(profiledPIDOutput);


        SmartDashboard.putNumber("EndAccessory Subsystem/end current angle", getAngle().in(Degrees));
        SmartDashboard.putBoolean("EndAccessory Subsystem/end has piece?", hasPiece());
        SmartDashboard.putNumber("EndAccessory Subsystem/infrared end value", pieceDetector.getVoltage());
        SmartDashboard.putBoolean("EndAccessory Subsystem/ is angle connected", angleEncoder.isConnected());
        SmartDashboard.putNumber("EndAccessory Subsystem/ pid output", profiledPIDOutput);
        SmartDashboard.putNumber("EndAccessory Subsystem/ timer", timer.get());
        SmartDashboard.putBoolean("EndAccessory Subsystem/ is at setpoint", isAtSetpoint());
        SmartDashboard.putNumber("EndAccessory Subsystem/ pid setPoint",profiledPIDController.getGoal().position);

    }
}
