// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This sample program shows how to control a motor using a joystick. In the operator control part
 * of the program, the joystick is read and the value is written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and motor controller inputs also range from -1 to 1
 * making it easy to work together.
 *
 * <p>In addition, the encoder value of an encoder connected to ports 0 and 1 is consistently sent
 * to the Dashboard.
 */
public class Robot extends TimedRobot {
  private static final double MAX_SPEED = 0.25;
  private static final double MIN_SPEED = -0.25;

  private static final double WHEEL_ROTATIONS_PER_MOTOR_ROTATION = (1.0 / (150.0 / 7.0));

  private static final int kMotorPort = 15;
  private static final int kJoystickPort = 0;

  private final SparkMax m_motor;
  private final XboxController m_joystick;
  private final RelativeEncoder m_encoder;

  private final PIDController m_pidController = new PIDController(0.1, 0.0, 0.0);

  /** Called once at the beginning of the robot program. */
  public Robot() {
    m_motor = new SparkMax(kMotorPort, MotorType.kBrushless);
    m_joystick = new XboxController(kJoystickPort);
    m_encoder = m_motor.getEncoder();

    m_pidController.setTolerance(0.05);

    SmartDashboard.putNumber("P", m_pidController.getP());
    SmartDashboard.putNumber("I", m_pidController.getI());
    SmartDashboard.putNumber("D", m_pidController.getD());
  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Encoder", m_encoder.getPosition());
    SmartDashboard.putNumber("Joystick", m_joystick.getLeftY());
    SmartDashboard.putNumber("Motor", m_motor.getAppliedOutput());
    SmartDashboard.putNumber("Stick Angle (Radians)", calculateStickAngle());
    SmartDashboard.putNumber("Motor Setpoint", m_pidController.getSetpoint());
    SmartDashboard.putNumber("Wheel Angle (Radians)",Rotation2d.fromRotations(m_encoder.getPosition() * WHEEL_ROTATIONS_PER_MOTOR_ROTATION).getRadians());

    // Get PID values from SmartDashboard
    double p = SmartDashboard.getNumber("P", 0.0);
    double i = SmartDashboard.getNumber("I", 0.0);
    double d = SmartDashboard.getNumber("D", 0.0);

    // Update PID controller values
    if (p != m_pidController.getP()) {
      m_pidController.setP(p);
    }
    if (i != m_pidController.getI()) {
      m_pidController.setI(i);
    }
    if (d != m_pidController.getD()) {
      m_pidController.setD(d);
    }
  }

  /** The teleop periodic function is called every control packet in teleop. */
  @Override
  public void teleopPeriodic() {
    // Get the joystick angle and convert it to a setpoint for the motor
    Rotation2d rotation = Rotation2d.fromDegrees(calculateStickAngle());

    double motorSetpoint = rotation.getRotations() / WHEEL_ROTATIONS_PER_MOTOR_ROTATION;

    // Set the motor setpoint based on the joystick angle
    m_pidController.setSetpoint(motorSetpoint);

    // Calculate the PID output and clamp it to the motor speed limits
    double output = m_pidController.calculate(m_encoder.getPosition());
    output = MathUtil.clamp(output, MIN_SPEED, MAX_SPEED);

    // Set the motor speed
    if (!m_pidController.atSetpoint())  {
      m_motor.set(output);
    }
  }

  private double calculateStickAngle() {
    double x = m_joystick.getLeftX();
    double y = m_joystick.getLeftY();

    double angle = Math.atan2(y, x);
    angle += Math.PI / 2; // Adjust for the joystick orientation

    return angle;
  }
}
