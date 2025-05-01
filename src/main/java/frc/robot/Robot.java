// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

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

  private static final int kMotorPort = 10;
  private static final int kJoystickPort = 0;

  private final SparkMax m_motor;
  private final SparkClosedLoopController m_motorController;
  private final SparkFlexConfig m_motorConfig = new SparkFlexConfig();
  private final ClosedLoopConfig m_closedLoopConfig = new ClosedLoopConfig();
  private final XboxController m_joystick;
  private final RelativeEncoder m_encoder;

  private double p = 0.2;
  private double i = 0.0;
  private double d = 0.0;

  private final PIDController m_pidController = new PIDController(p, i, d);

  /** Called once at the beginning of the robot program. */
  public Robot() {
    m_motor = new SparkMax(kMotorPort, MotorType.kBrushless);
    m_motorController = m_motor.getClosedLoopController();
    m_joystick = new XboxController(kJoystickPort);
    m_encoder = m_motor.getEncoder();

    m_pidController.setTolerance(0.05);
    m_closedLoopConfig.pid(p, i, d);
    m_motorConfig.apply(m_closedLoopConfig);
    m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putNumber("P", p);
    SmartDashboard.putNumber("I", i);
    SmartDashboard.putNumber("D", d);
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
    double np = SmartDashboard.getNumber("P", 0.0);
    double ni = SmartDashboard.getNumber("I", 0.0);
    double nd = SmartDashboard.getNumber("D", 0.0);

    // Update PID controller values
    boolean setConfig = false;
    if (p != np) {
      p = np;
      m_pidController.setP(p);
      m_closedLoopConfig.p(p);
      setConfig = true;
    }
    if (i != ni) {
      i = ni;
      m_pidController.setI(i);
      m_closedLoopConfig.i(i);
      setConfig = true;
    }
    if (d != nd) {
      d = nd;
      m_pidController.setD(d);
      m_closedLoopConfig.d(d);
      setConfig = true;
    }

    if (setConfig) {
      m_motorConfig.apply(m_closedLoopConfig);
      m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  }

  /** The teleop periodic function is called every control packet in teleop. */
  @Override
  public void teleopPeriodic() {
    // Get the joystick angle and convert it to a setpoint for the motor
    Rotation2d rotation = Rotation2d.fromRadians(calculateStickAngle());
    Rotation2d currentRotation = Rotation2d.fromRotations(m_encoder.getPosition() * WHEEL_ROTATIONS_PER_MOTOR_ROTATION);
    Rotation2d difference = currentRotation.minus(rotation);

    double motorSetpoint = rotation.getRotations() / WHEEL_ROTATIONS_PER_MOTOR_ROTATION;
    double motorPosition = m_encoder.getPosition();


    // Set the motor setpoint based on the joystick angle
    /*m_pidController.setSetpoint(motorSetpoint);

    // Calculate the PID output and clamp it to the motor speed limits
    double output = m_pidController.calculate(m_encoder.getPosition());
    //output = MathUtil.clamp(output, MIN_SPEED, MAX_SPEED);

    // Set the motor speed
    if (!m_pidController.atSetpoint())  {
      m_motor.set(output);
    } else {
      m_motor.set(0);
    }
  */

   m_motorController.setReference(motorSetpoint, ControlType.kPosition);
  }

  private double calculateStickAngle() {
    double x = m_joystick.getLeftX();
    double y = m_joystick.getLeftY();

    double angle = Math.atan2(y, x);
    //angle += Math.PI / 2; // Adjust for the joystick orientation

    return angle;
  }
}
