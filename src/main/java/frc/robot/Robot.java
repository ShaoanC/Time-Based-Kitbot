// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

  private static final int kControllerPort = 0; // 第一个手柄端口号为 0
  private static final int kLeftMasterCanID = 1; // 左侧主电机的 CAN ID
  private static final int kLeftFollowerCanID = 2; // 左侧从电机的 CAN ID
  private static final int kRightMasterCanID = 3; // 右侧主电机的 CAN ID
  private static final int kRightFollowerCanID = 4; // 右侧从电机的 CAN ID
  private static final int kFeederCanID = 5; // 供料器电机的 CAN ID
  private static final int kShooterCanID = 6; // 射手电机的 CAN ID

  private static final InvertedValue kFeederInverted = InvertedValue.CounterClockwise_Positive; // 供料器电机的反转状态
  private static final InvertedValue kShooterInverted = InvertedValue.Clockwise_Positive; // 射手电机的反转状态

  private static final double kLaunchShooterVoltage = 11.0; // 发射器电机的电压
  private static final double kLaunchFeederVoltage = 9.0; // 供料器电机的电压
  private static final double kLaunchShooterVelocityRps = 50.0;
  private static final double kLaunchFeederVelocityRps = 30.0;
  private static final double kFeederVelocityRps = 40.0;
  private static final double kIntakeShooterVelocityRps = 10.0; // 发射器电机的目标速度，单位为转每秒
  private static final double kIntakeFeederVelocityRps = -10.0; // 供料器电机的目标速度，单位为转每秒

  private static final double kShooterKs = 0.20; // 发射器电机的静态摩擦力补偿
  private static final double kShooterKv = 0.50; // 发射器电机的速度增益
  private static final double kShooterKp = 0.70; // 发射器电机的比例增益
  private static final double kShooterKi = 0.00; // 发射器电机的积分增益
  private static final double kShooterKd = 0.00; // 发射器电机的微分增益

  private static final double kFeederKs = 0.20; // 供料器电机的静态摩擦力补偿
  private static final double kFeederKv = 0.12; // 供料器电机的速度增益
  private static final double kFeederKp = 0.15; // 供料器电机的比例增益
  private static final double kFeederKi = 0.00; // 供料器电机的积分增益
  private static final double kFeederKd = 0.00; // 供料器电机的微分增益

  private final WPI_TalonSRX m_leftMaster = new WPI_TalonSRX(kLeftMasterCanID);
  private final WPI_VictorSPX m_leftFollower = new WPI_VictorSPX(kLeftFollowerCanID);
  private final WPI_TalonSRX m_rightMaster = new WPI_TalonSRX(kRightMasterCanID);
  private final WPI_VictorSPX m_rightFollower = new WPI_VictorSPX(kRightFollowerCanID);

  private final TalonFX m_feeder = new TalonFX(kFeederCanID);
  private final TalonFX m_shooter = new TalonFX(kShooterCanID);

  private final VelocityVoltage m_feederVelocityRequest = new VelocityVoltage(0.0);
  private final VelocityVoltage m_shooterVelocityRequest = new VelocityVoltage(0.0);
  private final NeutralOut m_stopRequest = new NeutralOut();

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMaster, m_rightMaster);
  private final XboxController m_controller = new XboxController(kControllerPort);

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    m_leftMaster.configFactoryDefault();
    m_leftFollower.configFactoryDefault();
    m_rightMaster.configFactoryDefault();
    m_rightFollower.configFactoryDefault();

    TalonFXConfiguration feederConfig = new TalonFXConfiguration();
    feederConfig.MotorOutput.Inverted = kFeederInverted;
    feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    feederConfig.Slot0.kS = kFeederKs;
    feederConfig.Slot0.kV = kFeederKv;
    feederConfig.Slot0.kP = kFeederKp;
    feederConfig.Slot0.kI = kFeederKi;
    feederConfig.Slot0.kD = kFeederKd;

    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.MotorOutput.Inverted = kShooterInverted;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterConfig.Slot0.kS = kShooterKs;
    shooterConfig.Slot0.kV = kShooterKv;
    shooterConfig.Slot0.kP = kShooterKp;
    shooterConfig.Slot0.kI = kShooterKi;
    shooterConfig.Slot0.kD = kShooterKd;

    m_feeder.getConfigurator().apply(feederConfig);
    m_shooter.getConfigurator().apply(shooterConfig);

    m_leftFollower.follow(m_leftMaster);
    m_rightFollower.follow(m_rightMaster);

    m_rightMaster.setInverted(true);
    m_rightFollower.setInverted(true); // 反转右侧电机

    // 底盘设置为break
    m_leftMaster.setNeutralMode(NeutralMode.Brake);
    m_leftFollower.setNeutralMode(NeutralMode.Brake);
    m_rightMaster.setNeutralMode(NeutralMode.Brake);
    m_rightFollower.setNeutralMode(NeutralMode.Brake);
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

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double forward = -m_controller.getLeftY();
    double rotation = -m_controller.getRightX();

    m_drive.arcadeDrive(forward, rotation);

    if (m_controller.getXButton()) {
      m_shooter.setControl(m_shooterVelocityRequest.withVelocity(kIntakeShooterVelocityRps));
      m_feeder.setControl(m_feederVelocityRequest.withVelocity(kIntakeFeederVelocityRps));
    } else if (m_controller.getAButton()) {
      m_shooter.setControl(new VoltageOut(kLaunchShooterVoltage));
      m_feeder.setControl(new VoltageOut(kLaunchFeederVoltage));
    } else {
      m_shooter.setControl(m_stopRequest);
      m_feeder.setControl(m_stopRequest);
    }
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
}
