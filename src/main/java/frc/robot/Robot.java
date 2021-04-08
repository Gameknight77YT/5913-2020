/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */


private final WPI_TalonFX leftMaster = new WPI_TalonFX(1);
private final WPI_TalonFX rightMaster = new WPI_TalonFX(2);
private final WPI_TalonFX leftSlave = new WPI_TalonFX(4);
private final WPI_TalonFX rightSlave = new WPI_TalonFX(3);

private final DifferentialDrive robotDrive = new DifferentialDrive(leftMaster, rightMaster);

private final WPI_TalonSRX turretControl = new WPI_TalonSRX(11);
private final WPI_TalonSRX mainShooter = new WPI_TalonSRX(7);
private final WPI_TalonSRX topShooter = new WPI_TalonSRX(8);

private final WPI_VictorSPX feederWheel = new WPI_VictorSPX(13);
private final WPI_VictorSPX intakeSystem = new WPI_VictorSPX(17);
private final WPI_VictorSPX intakeWheels = new WPI_VictorSPX(14);

private final WPI_TalonSRX elevatorMaster = new WPI_TalonSRX(9);
private final WPI_VictorSPX elevatorSlave = new WPI_VictorSPX(10);

private final Compressor compressor =  new Compressor();

private final DoubleSolenoid intakeArms = new DoubleSolenoid(5, 7);
private final DoubleSolenoid ratchetArm = new DoubleSolenoid(4, 1);

private final Joystick driverJoystick = new Joystick(0);
private final Joystick manipJoystick = new Joystick(1);
CameraServer server;

private boolean limelightHasValidTarget = false;
private double m_LimelightSteerCommand = 0.0;

@Override
  public void robotInit() {
    server = CameraServer.getInstance();
    server.startAutomaticCapture(0);
    
    //interted settings
    leftMaster.setInverted(true);
    rightMaster.setInverted(true);

    //slave setups
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    elevatorSlave.follow(elevatorMaster);

    leftSlave.setInverted(InvertType.FollowMaster);
    rightSlave.setInverted(InvertType.FollowMaster);

    //init encoders
    mainShooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    topShooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    turretControl.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    leftMaster.clearStickyFaults(10);
    rightMaster.clearStickyFaults(10);
    mainShooter.clearStickyFaults(10);
    topShooter.clearStickyFaults(10);
    turretControl.clearStickyFaults(10);

    //config PIDs
    mainShooter.config_kF(0, 0.0265, 10);
    mainShooter.config_kP(0, 0.65, 10);
    mainShooter.config_kI(0, 0.0002, 10);
    mainShooter.config_kD(0, 6, 10);
    mainShooter.config_IntegralZone(0, 100, 10);

    topShooter.config_kF(0, 0.02764865, 10);
    topShooter.config_kP(0, 0.05, 10);
    topShooter.config_kI(0, 0, 10);
    topShooter.config_kD(0, 0, 10);

    //reset encoders
    leftMaster.setSelectedSensorPosition(0, 0, 10);
    rightMaster.setSelectedSensorPosition(0, 0, 10);
    mainShooter.setSelectedSensorPosition(0, 0, 10);
    topShooter.setSelectedSensorPosition(0, 0, 10);
    turretControl.setSelectedSensorPosition(0, 0, 10);
    turretControl.setSensorPhase(false);

    //set configs
    turretControl.configMotionAcceleration(450, 10);
    turretControl.configMotionCruiseVelocity(450, 10);
    turretControl.setNeutralMode(NeutralMode.Coast);
    
    mainShooter.setNeutralMode(NeutralMode.Coast);
    topShooter.setNeutralMode(NeutralMode.Coast);

    leftMaster.setNeutralMode(NeutralMode.Coast);
    rightMaster.setNeutralMode(NeutralMode.Coast);
    leftSlave.setNeutralMode(NeutralMode.Coast);
    rightSlave.setNeutralMode(NeutralMode.Coast);

    leftMaster.configOpenloopRamp(1);
    rightMaster.configOpenloopRamp(1);
    leftSlave.configOpenloopRamp(1);
    rightSlave.configOpenloopRamp(1);

     

    //start compressor
    compressor.start();

  }


  @Override
  public void autonomousInit() {
    leftMaster.setSafetyEnabled(false);
    rightMaster.setSafetyEnabled(false);
    leftSlave.setSafetyEnabled(false);
    rightSlave.setSafetyEnabled(false);
  }

  @Override
  public void autonomousPeriodic() {
  }


  @Override
  public void teleopInit() {

    
  }

  @Override
  public void teleopPeriodic() {
    //networktables
    final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    final NetworkTableEntry tx = table.getEntry("tx");
    final NetworkTableEntry ty = table.getEntry("ty");
    final NetworkTableEntry ta = table.getEntry("ta");
    final NetworkTableEntry ledMode = table.getEntry("ledMode");
    
    //read values periodically
    final double x = tx.getDouble(0.0);
    final double y = ty.getDouble(0.0);
    final double area = ta.getDouble(0.0);
    
    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    final double distAngle = (20 + y);
    final double dist = (8.1875 - 2.25) / (Math.tan(distAngle));
    SmartDashboard.putNumber("Distance", dist);


    robotDrive.arcadeDrive(driverJoystick.getY()*.65, driverJoystick.getX()* -.65);

    double suckSpeed = 0;
    intakeWheels.set(ControlMode.PercentOutput, suckSpeed);
  if (manipJoystick.getRawButton(5)==true){
    suckSpeed = -0.65;
    intakeWheels.set(ControlMode.PercentOutput, suckSpeed);
  } 
  if (manipJoystick.getRawButton(8)==true){
    suckSpeed = 0.65;
    intakeWheels.set(ControlMode.PercentOutput, suckSpeed);
  } 




      double shooterSpeed = 0;
      mainShooter.set(ControlMode.PercentOutput, shooterSpeed);
    if (manipJoystick.getRawButton(7)==true){
      shooterSpeed = 20900;
      mainShooter.set(ControlMode.Velocity, shooterSpeed);
    } 
    if (manipJoystick.getRawButton(3)==true){
      shooterSpeed = 22000;
      mainShooter.set(ControlMode.Velocity, shooterSpeed);
    } 
    if (manipJoystick.getRawButton(11)==true){
      shooterSpeed = 23500;
      mainShooter.set(ControlMode.Velocity, shooterSpeed);
    }
      
    topShooter.set(ControlMode.PercentOutput, shooterSpeed);
    if (manipJoystick.getRawButton(7)==true){
      topShooter.set(ControlMode.Velocity, (shooterSpeed * -1) - 3000);
    } 
    if (manipJoystick.getRawButton(3)==true){
      topShooter.set(ControlMode.Velocity, shooterSpeed * -1);
    } 
    if (manipJoystick.getRawButton(11)==true){
      topShooter.set(ControlMode.Velocity, shooterSpeed * -1);
    }


       limelightTracking();

       if (manipJoystick.getRawButton(1)==true){
        ledMode.setValue(0);
        if (limelightHasValidTarget){
          turretControl.set(ControlMode.PercentOutput, m_LimelightSteerCommand);
        }
        else{
          turretControl.set(ControlMode.PercentOutput, 0);
        }
      
      }
      else { 
        ledMode.setNumber(0);

            if (manipJoystick.getRawButton(4)==true){ //backwards
              turretControl.set(ControlMode.PercentOutput, -0.5);
            }
            else {
              turretControl.set(ControlMode.PercentOutput, 0);
            }     
          }
         
        

        double feederSpeed = 0;
      if (manipJoystick.getRawButton(2)==true){
        feederSpeed = -0.7;
 
      }

      
      feederWheel.set(ControlMode.PercentOutput, feederSpeed);

      double intakeSpeed = 0; ////
      if (manipJoystick.getRawButton(2)==true){
        intakeSpeed = -1;
      }
      if (manipJoystick.getRawButton(5)==true){
        intakeSpeed = -0.8;
      }
      if (manipJoystick.getRawButton(8)==true){
        intakeSpeed = 0.5;
      }
      intakeSystem.set(ControlMode.PercentOutput, intakeSpeed);


    double elevatorSpeed = 0; ////
      if (manipJoystick.getRawButton(6)==true && manipJoystick.getRawButton(10)){
        elevatorSpeed = 1;
      }
      if (manipJoystick.getRawButton(12)==true){
        elevatorSpeed = -1;
      }
      elevatorMaster.set(ControlMode.PercentOutput, elevatorSpeed);

    intakeArms.set(Value.kOff);
    if (driverJoystick.getRawButton(1)==true){
      intakeArms.set(Value.kForward);;
    }
    if (driverJoystick.getRawButton(2)==true){
        intakeArms.set(Value.kReverse);;
    }

    ratchetArm.set(Value.kOff);
    if (manipJoystick.getRawButton(6)==true){
        ratchetArm.set(Value.kReverse);
    }    
    if (manipJoystick.getRawButton(12)==true){
        ratchetArm.set(Value.kForward);
    }
  }
      
    
    

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
  public void limelightTracking() {
        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_K = 0.04;                   // how hard to turn toward the target
        final double min_command = 0.025;
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

        if (tv < 1.0)
        {
          limelightHasValidTarget = false;
      m_LimelightSteerCommand = 0.0;
      return;
    }

    limelightHasValidTarget = true;

        // Start with proportional steering
      double steer_cmd = 0.0;
       
       if (tx < 0) {
        steer_cmd = tx * STEER_K - min_command;
       }
       else if (tx > 0) {
        steer_cmd = tx * STEER_K + min_command;
       }
        m_LimelightSteerCommand = steer_cmd * -1;


        }
      }