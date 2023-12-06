// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.filter.SlewRateLimiter;
//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * ra
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private Joystick joy1 = new Joystick(0);
  private final WPI_TalonFX talon = new WPI_TalonFX(10);
  private final WPI_TalonSRX M1 = new WPI_TalonSRX(16);
  private final WPI_TalonSRX M2 = new WPI_TalonSRX(27);
  private final WPI_TalonSRX M3 = new WPI_TalonSRX(35);
  private final WPI_TalonSRX M4 = new WPI_TalonSRX(36);
  private PIDController turretpid = new PIDController(0.042, 0.0008, 0.0007);
  private PIDController turnpid = new PIDController(0.22, 0.08, 0.07);
  private PIDController speedpid = new PIDController(0.37, 0.08, 0.07);
  // private PIDController pid = new PIDController(0.042, 0.0008, 0.0007);
  private MotorControllerGroup GR = new MotorControllerGroup(M1, M2);
  private MotorControllerGroup GL = new MotorControllerGroup(M3, M4);
  private DifferentialDrive drive = new DifferentialDrive(GL, GR);
  // private Encoder encoder = new Encoder(0, 1);
  {

  }
  // private Encoder encoder = new Encoder(1, 0);
  // private SlewRateLimiter filter = new SlewRateLimiter(0.5);

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    GL.setInverted(false);
    GR.setInverted(true);

    for (int port = 5800; port <= 5807; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // encoder.reset();
    talon.setSelectedSensorPosition(kDefaultPeriod, 0, 0);
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // pid.setSetpoint(3);
        // pid.setTolerance(.1);
        // double speed = pid.calculate(encoder.get());
        // GL.set(speed);
        // GR.set(-speed);

        break;
      case kDefaultAuto:
      default:
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ta = table.getEntry("ta");
        double area = ta.getDouble(0.0);

        Boolean mSeesTarget = table.getEntry("tv").getDouble(0) == 1.0;
        turnpid.setSetpoint(0);
        speedpid.setSetpoint(50);
        double teV = talon.getSelectedSensorPosition() / 2048D / 140D / 12D * 360;
        double pidt = turnpid.calculate(teV);
        double pids = speedpid.calculate(area);
        double speed = MathUtil.clamp(pids, -0.8, 0.8);
        double turn = MathUtil.clamp(pidt, -0.8, 0.8);
        if (mSeesTarget) {
          // drive.arcadeDrive(turn, speed);
  if(teV < 1 && teV > -1){;
          drive.arcadeDrive(-speed, 0);
  }else{
    drive.arcadeDrive(0, -turn);
  }
        }
        SmartDashboard.putNumber("turret", teV);
        //SmartDashboard.putNumber("turnpid", turn);
        SmartDashboard.putNumber("ta", area);
        SmartDashboard.putNumber("speed", speed);
        SmartDashboard.putNumber("turn", turn);
        PID();

        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    turretpid.setSetpoint(0);
    // turretpid.setTolerance();

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    Limelight();
    PID();
    double speed = joy1.getRawAxis(0) * 0.9;
    double turn = joy1.getRawAxis(5) * 0.9;
    drive.arcadeDrive(speed, turn);
    SmartDashboard.putNumber("turret pos", talon.getSelectedSensorPosition());

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

  public void Limelight() {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    // read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    // post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  }

  public void PID() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    double x = tx.getDouble(0.0);
    double speed = turretpid.calculate(x);
    talon.set(-speed);
  }
}
