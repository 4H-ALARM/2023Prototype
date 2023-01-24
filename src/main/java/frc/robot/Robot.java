// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
//import com.ctre.phoenix.motorcontrol.can.VictorSPX;


/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;

  private final MotorController m_leftMotor_front = new WPI_VictorSPX(02);
  private final MotorController m_leftMotor_back = new WPI_VictorSPX(05);
  MotorControllerGroup m_left = new MotorControllerGroup(m_leftMotor_front, m_leftMotor_back);

  private final MotorController m_rightMotor_front = new WPI_VictorSPX(03);
  private final MotorController m_rightMotor_back = new WPI_VictorSPX(01);
  MotorControllerGroup m_right = new MotorControllerGroup(m_rightMotor_front, m_rightMotor_back);

  final XboxController xboxpad = new XboxController(0);

  private final MotorController clawmotor = new WPI_VictorSPX(04);

  private final MotorController pulleymotor = new WPI_VictorSPX(17); 

  private final MotorController teliscopic = new WPI_VictorSPX(06);

  SlewRateLimiter filter = new SlewRateLimiter(0.5);

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_left.setInverted(true);

    m_myRobot = new DifferentialDrive(m_left, m_right);
    
  }

  @Override
  public void teleopPeriodic() {

    // m_myRobot.tankDrive(filter.calculate(xboxpad.getLeftY()), filter.calculate(xboxpad.getRightY()));
    m_myRobot.tankDrive(0.5*xboxpad.getLeftY(), 0.5*xboxpad.getRightY());


    if (xboxpad.getAButtonPressed()) {
      clawmotor.set(0.7);
    }
    if (xboxpad.getAButtonReleased()) {
      clawmotor.stopMotor();
    }

    if (xboxpad.getBButtonPressed()) {
      clawmotor.set(-0.9);
    }
    if (xboxpad.getBButtonReleased()) {
      clawmotor.stopMotor();
    }




    if (xboxpad.getYButtonPressed()) {
      pulleymotor.set(-0.8);
    }
    if (xboxpad.getYButtonReleased()) {
      pulleymotor.stopMotor();
    }

    if (xboxpad.getXButtonPressed()) {
      pulleymotor.set(0.8);
    }
    if (xboxpad.getXButtonReleased()) {
      pulleymotor.stopMotor();

    }


    if (xboxpad.getRightBumperPressed()) {
      teliscopic.set(-0.8);
    }
    if (xboxpad.getRightBumperReleased()) {
      teliscopic.stopMotor();
    }

    if (xboxpad.getLeftBumperPressed()) {
      teliscopic.set(0.8);
    }
    if (xboxpad.getLeftBumperReleased()) {
      teliscopic.stopMotor();  

    }
  }
}
