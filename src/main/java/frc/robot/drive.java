/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.controller.PIDController;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.*;


import edu.wpi.first.wpilibj.SerialPort;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

/**
 * Add your docs here.
 */

public class drive {

    public static final double L = 23;
    public static final double W = 23;

    public static final double P = .1;
    public static final double I = .01;
    public static final double D = 0;

    public static final double backLeftAngleZero = 0;
    public static final double backRightAngleZero = 0;
    public static final double frontLeftAngleZero = 0;
    public static final double frontRightAngleZero = 0;

    public static TalonFX mS_backRight, mS_backLeft, mS_frontRight, mS_frontLeft, mD_backRight, mD_backLeft, mD_frontRight, mD_frontLeft;

    public static CANCoder e_backRight, e_backLeft, e_frontRight, e_frontLeft;

    private static PIDController pS_backRight, pS_backLeft, pS_frontRight, pS_frontLeft;

    public static AHRS gyro;
    public static double PI = 3.141592653589793;
    public static void initDrive(){

        mS_backLeft = new TalonFX(3);
        e_backLeft = new CANCoder(2);
        mD_backLeft = new TalonFX(1);
        pS_backLeft = new PIDController(P, I, D, .00001);
        pS_backLeft.enableContinuousInput(-2, 2);
        mS_backLeft.setNeutralMode(NeutralMode.Brake);
        mD_backLeft.setNeutralMode(NeutralMode.Brake);


        mD_backRight = new TalonFX(4);
        e_backRight = new CANCoder(5);
        mS_backRight = new TalonFX(6);
        pS_backRight = new PIDController(P, I, D, .00001);
        pS_backRight.enableContinuousInput(-2, 2);
        mS_backRight.setNeutralMode(NeutralMode.Brake);
        mD_backRight.setNeutralMode(NeutralMode.Brake);

        mS_frontLeft = new TalonFX(9);
        e_frontLeft = new CANCoder(8);
        mD_frontLeft = new TalonFX(7);
        pS_frontLeft = new PIDController(P, I, D, .00001);
        pS_frontLeft.enableContinuousInput(-2, 2);
        mS_frontLeft.setNeutralMode(NeutralMode.Brake);
        mD_frontLeft.setNeutralMode(NeutralMode.Brake);


        mS_frontRight = new TalonFX(12);
        e_frontRight = new CANCoder(11);
        mD_frontRight = new TalonFX(10);
        pS_frontRight = new PIDController(P, I, D, .00001);
        pS_frontRight.enableContinuousInput(-2, 2);
        mS_frontRight.setNeutralMode(NeutralMode.Brake);
        mD_frontRight.setNeutralMode(NeutralMode.Brake);

        //gyro = new AHRS(SerialPort.Port.kMXP);

    }
    public static void drive (double x1, double y1, double x2, boolean calibrate) {
        double r = Math.sqrt ((L * L) + (W * W));
        y1 *= -1;
    
        double a = x1 - x2 * (L / r);
        double b = x1 + x2 * (L / r);
        double c = y1 - x2 * (W / r);
        double d = y1 + x2 * (W / r);
    
        double backRightSpeed = Math.sqrt ((a * a) + (d * d));
        double backLeftSpeed = Math.sqrt ((a * a) + (c * c));
        double frontRightSpeed = Math.sqrt ((b * b) + (d * d));
        double frontLeftSpeed = Math.sqrt ((b * b) + (c * c));
    
        double backRightAngle = Math.atan2 (a, d) / PI;
        double backLeftAngle = Math.atan2(a, c) / PI;
        double frontRightAngle = Math.atan2(b, d) / PI;
        double frontLeftAngle = Math.atan2(b, c) / PI;

        
        
        double backRightAngleOutput = pS_backRight.calculate(e_backRight.getAbsolutePosition()/180, backRightAngle + 1);
        double backLeftAngleOutput = pS_backLeft.calculate(e_backLeft.getAbsolutePosition()/180, backLeftAngle + 1);
        double frontRightAngleOutput = pS_frontRight.calculate(e_frontRight.getAbsolutePosition()/180, frontRightAngle + 1);
        double frontLeftAngleOutput = pS_frontLeft.calculate(e_frontLeft.getAbsolutePosition()/180, frontLeftAngle + 1);

        SmartDashboard.putNumber("backRight Goal", backRightAngle + 1);
        SmartDashboard.putNumber("backlLeft Goal", backLeftAngle + 1);
        SmartDashboard.putNumber("frontRight Goal", frontRightAngle + 1);
        SmartDashboard.putNumber("frontLeft Goal", frontLeftAngle + 1);
        
        SmartDashboard.putNumber("virtual back right angle", (e_backRight.getAbsolutePosition() + backRightAngleZero)/180);
        SmartDashboard.putNumber("actual back right angle", (e_backRight.getAbsolutePosition()/180));

        SmartDashboard.putNumber("virtual back left angle", (e_backLeft.getAbsolutePosition() + backLeftAngleZero)/180);
        SmartDashboard.putNumber("actual back left angle", (e_backLeft.getAbsolutePosition())/180);

        SmartDashboard.putNumber("virtual front right angle", (e_frontRight.getAbsolutePosition() + frontRightAngleZero)/180);
        SmartDashboard.putNumber("actual front right angle", (e_frontRight.getAbsolutePosition())/180);
        
        SmartDashboard.putNumber("virtual front left angle", (e_frontLeft.getAbsolutePosition() + frontLeftAngleZero)/180);
        SmartDashboard.putNumber("actual front left angle", (e_frontLeft.getAbsolutePosition())/180);

        SmartDashboard.putNumber("backRightAngleOutput", backRightAngleOutput);
        SmartDashboard.putNumber("backLeftAngleOutput", backLeftAngleOutput);
        SmartDashboard.putNumber("frontRightAngleOutput", frontRightAngleOutput);
        SmartDashboard.putNumber("frontLeftAngleOutput", frontLeftAngleOutput);

        //SmartDashboard.putNumber("gyroAngle", gyro.getAngle());

       mS_backRight.set(ControlMode.PercentOutput, backRightAngleOutput);
       mS_backLeft.set(ControlMode.PercentOutput, backLeftAngleOutput);
       mS_frontRight.set(ControlMode.PercentOutput, frontRightAngleOutput);
       mS_frontLeft.set(ControlMode.PercentOutput, frontLeftAngleOutput);

       mD_backRight.set(ControlMode.PercentOutput, backRightSpeed);
       mD_backLeft.set(ControlMode.PercentOutput, backLeftSpeed);
       mD_frontRight.set(ControlMode.PercentOutput, frontRightSpeed);
       mD_frontLeft.set(ControlMode.PercentOutput, frontLeftSpeed);


    }

}
