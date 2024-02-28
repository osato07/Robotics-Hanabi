package frc.robot;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.Servo;

public class Robot extends TimedRobot {
    private PWMSparkMax leftMotor1, leftMotor2, rightMotor1, rightMotor2;
    private PWMSparkMax neoMotor1, neoMotor2;
    private PWMSparkMax neckMotor;
    private Joystick joystick;
    private Servo myServo;
    /*  private final DifferentialDrive m_robotDrive =
      new DifferentialDrive(leftMotor1::set, leftMotor2::set, rightMotor1::set, rightMotor2::set); */
   
    // ポート番号
    private final int neoMotor1Port = 1;
    private final int neoMotor2Port = 0;
    private final int leftMotor1Port = 2;
    private final int leftMotor2Port = 3;
    private final int rightMotor1Port = 4;
    private final int rightMotor2Port = 5;
    private final int neckMotorPort = 6;

    private final int joystickPort = 0;

    private final int servoPort =7;

    private final Timer m_timer = new Timer();

    @Override
    public void robotInit() {
        leftMotor1 = new PWMSparkMax(leftMotor1Port);
        leftMotor2 = new PWMSparkMax(leftMotor2Port);
        rightMotor1 = new PWMSparkMax(rightMotor1Port);
        rightMotor2 = new PWMSparkMax(rightMotor2Port);
        neoMotor1 = new PWMSparkMax(neoMotor1Port);
        neoMotor2 = new PWMSparkMax(neoMotor2Port);
        neckMotor = new PWMSparkMax(neckMotorPort);

        joystick = new Joystick(joystickPort);

        myServo = new Servo(servoPort);
    }

    /** This function is run once each time the robot enters autonomous mode. */
    @Override
    public void autonomousInit() {
        m_timer.restart();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        // Drive for 2 seconds
        if (m_timer.get() < 2.0) {
        // Drive forwards half speed, make sure to turn input squaring off
            neoMotor1.set(0.5);
            Timer.delay(0.5);
            neoMotor2.set(0.5);
            Timer.delay(0.5);
        } else {
            neoMotor1.set(0);
            neoMotor2.set(0);
        }
    }

    @Override
    public void teleopPeriodic() {
        if (joystick.getRawButton(5)) {
            // Joystickのボタン2が押されたとき
            neoMotor1.set(1); // NEOモーター1を50%の速度で回転
            // System.out.println("motor111");
            neoMotor2.set(-1); // NEOモーター2を50%の速度で回転
            // System.out.println("motor222");
        } else {
            // ボタンが離されたとき
            neoMotor1.set(0); // NEOモーター1を停止
            neoMotor2.set(0); // NEOモーター2を停止
        }

        if (joystick.getRawButton(6)) {
            myServo.set(0.5); 
        } else {
            myServo.set(0.0);
        }

        if (joystick.getRawButton(4)) {
            neckMotor.set(0.1);
        } else if (joystick.getRawButton(3)) {
            neckMotor.set(-0.1);
        }
        
        double left = -1 * joystick.getRawAxis(1);                                                                                                                                                                                                                                                                                                                                                                                                            
        double right = joystick.getRawAxis(5); // ジョイスティックのY軸

        if (joystick.getRawButton(7)) {
        leftMotor1.set(0);
        leftMotor2.set(0);
        rightMotor1.set(0);
        rightMotor2.set(0);
        }

        // モーターの速度と方向の計算
        double rightSpeed = right;
        double leftSpeed = left;

        // モーターを制御
        leftMotor1.set(limitSpeed(rightSpeed));
        leftMotor2.set(limitSpeed(rightSpeed));
        rightMotor1.set(limitSpeed(leftSpeed));
        rightMotor2.set(limitSpeed(leftSpeed));

        // m_robotDrive.set(-joystick.getY(), -joystick.getY(), -joystick.getX(), -joystick.getX());
    }

    private double limitSpeed(double speed) {
        return Math.max(-0.5, Math.min(0.5, speed));
    }

    /** This function is called once each time the robot enters test mode. */
    @Override
    public void testInit() {}

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}
}
