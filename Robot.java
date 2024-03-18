package frc.robot;

import edu.wpi.first.wpilibj.Timer;
// import java.util.Optional;
import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Servo;

public class Robot extends TimedRobot {
    private PWMSparkMax leftMotor1, leftMotor2, rightMotor1, rightMotor2;
    private PWMSparkMax rightNeoMotor, leftNeoMotor;
    private Joystick joystick;
    private Servo myServo;
    TalonSRX talonSRX = new TalonSRX(1);
   
    // ポート番号
    private final int rightNeoMotorPort = 0;
    private final int leftNeoMotorPort = 8;
    private final int leftMotor1Port = 2;
    private final int leftMotor2Port = 3;
    private final int rightMotor1Port = 4;
    private final int rightMotor2Port = 5;
    private final int servoPort = 6;
    private final int joystickPort = 0;
    // TalonSRX rightArmMotor = new TalonSRX(2);
    // TalonSRX leftArmMotor = new TalonSRX(3);

    private final Timer m_timer = new Timer();

    private int lastTargetPosition = 1000;

    @Override
    public void robotInit() {
        rightNeoMotor = new PWMSparkMax(rightNeoMotorPort);
        leftNeoMotor = new PWMSparkMax(leftNeoMotorPort);
        leftMotor1 = new PWMSparkMax(leftMotor1Port);
        leftMotor2 = new PWMSparkMax(leftMotor2Port);
        rightMotor1 = new PWMSparkMax(rightMotor1Port);
        rightMotor2 = new PWMSparkMax(rightMotor2Port);
        myServo = new Servo(servoPort);
        joystick = new Joystick(joystickPort);

        // // エンコーダーをフィードバックデバイスとして設定
        talonSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

        // // エンコーダーのリセット
        talonSRX.setSelectedSensorPosition(0, 0, 10);

        // // PID制御パラメータの設定（実際の値は調整が必要）
        talonSRX.config_kP(0, 0.1, 10);
        talonSRX.config_kI(0, 0.0, 10);
        talonSRX.config_kD(0, 0.0, 10);
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
            System.out.println("autoMa");
        } else {
            System.out.println("autoMi");
        }
        new WaitCommand(5.0);
    }

    @Override
    public void teleopPeriodic() { 

        boolean key5 = joystick.getRawButton(5);
        boolean key6 = joystick.getRawButton(6);


        if (key5) {
            // shoot
            rightNeoMotor.set(-1);
            leftNeoMotor.set(1);
            System.out.println("shoot");
        } else if (!(key5) && key6) {
            // collect
            rightNeoMotor.set(0.5);
            leftNeoMotor.set(-0.5);
            System.out.println("collect");
        } else {
            System.out.println("KEY8:" + key6);
            System.out.println("KEY7:" + key5);
            rightNeoMotor.set(0);
            leftNeoMotor.set(0);
        }


        if (key5 && key6) {
            // servo
            myServo.set(0.6);
        } else {
            myServo.set(0.86);
        }


        if (joystick.getRawButton(2)) {
            lastTargetPosition = 3000; // 目標位置を設定
        } else if (joystick.getRawButton(3)) {
            lastTargetPosition = 0; // 目標位置を設定
        } else {
            talonSRX.set(ControlMode.Position, lastTargetPosition);
        }


        SmartDashboard.putNumber("Encoder Position", talonSRX.getSelectedSensorPosition());
        SmartDashboard.putNumber("Target Position", lastTargetPosition);


        // ArmMotor
        // if (joystick.getRawButton(4)) {
        //     rightArmMotor.set(ControlMode.PercentOutput, 1);
        //     leftArmMotor.set(ControlMode.PercentOutput, -1);
        // } else if (joystick.getRawButton(1)) {
        //     rightArmMotor.set(ControlMode.PercentOutput, -1);
        //     leftArmMotor.set(ControlMode.PercentOutput, 1);
        // } else {
        //     rightArmMotor.set(ControlMode.PercentOutput, 0);
        //     leftArmMotor.set(ControlMode.PercentOutput, 0);
        // }

        
        // driveBase
        double left = -1 * joystick.getRawAxis(1);          
        double right = joystick.getRawAxis(5); // ジョイスティックのY軸
        double rightSpeed = right;
        double leftSpeed = left;
        leftMotor1.set(limitSpeed(rightSpeed));
        leftMotor2.set(limitSpeed(rightSpeed));
        rightMotor1.set(limitSpeed(leftSpeed));
        rightMotor2.set(limitSpeed(leftSpeed));
    }

    private double limitSpeed(double speed) {
        return Math.max(-0.5, Math.min(0.5, speed));
    }

    /** This function is called once each time the robot enters test mode. */
    @Override
    public void testInit() {}

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
        
    }
}
