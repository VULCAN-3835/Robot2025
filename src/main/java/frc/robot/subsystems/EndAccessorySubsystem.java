// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.Constants.EndAccessoryConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.AnalogInput;

public class EndAccessorySubsystem extends SubsystemBase {

    private TalonFX leftMotor;
    private TalonFX rightMotor;

    private AnalogInput pieceDetector;

    public EndAccessorySubsystem() {
        // init the subsystems motors
        this.leftMotor = new TalonFX(EndAccessoryConstants.leftMotorID);
        this.rightMotor = new TalonFX(EndAccessoryConstants.rightMotorID);

        //sets the left motor to follow the right motor
        this.leftMotor.setControl(new Follower(EndAccessoryConstants.rightMotorID, true)); 


        this.pieceDetector = new AnalogInput(EndAccessoryConstants.pieceDetectorID);

    }

    // returns a waitUntill command that returns true if there's a coral in the system
    public WaitUntilCommand waitForCoral() {
        return new WaitUntilCommand(() -> hasPiece());
    }

    // sets the gripper to intake power
    public void gripperIntake() {
        rightMotor.set(EndAccessoryConstants.intakePower);
    }

    // sets the gripper to output power
    public void gripperOut(){
        rightMotor.set(EndAccessoryConstants.outputPower);
    }

    // sets the gripper to stop
    public void gripperStop() {
        rightMotor.set(0);
    }

    // sets the gripper to the power thats in the param
    public void setPower(double power) {
        rightMotor.set(power);
    }

    // returns true if there is a piece in the subsystem
    public boolean hasPiece() {
        return pieceDetector.getVoltage() > EndAccessoryConstants.kHasPieceVoltageThreshold;

    }

    @Override
    public void periodic() {

        SmartDashboard.putBoolean("EndAccessory Subsystem/end has piece?", hasPiece());
        SmartDashboard.putNumber("EndAccessory Subsystem/infrared end value", pieceDetector.getVoltage());

    }
}