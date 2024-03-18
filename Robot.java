package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.cameraserver.CameraServer;


public class Robot extends TimedRobot {
    private PWMSparkMax leftMotor1, leftMotor2, rightMotor1, rightMotor2;
    private PWMSparkMax rightNeoMotor, leftNeoMotor;
    private Joystick joystick;
    private Servo myServo;
    TalonSRX talonSRX = new TalonSRX(1);
    TalonSRX rightArmMotor = new TalonSRX(2);
    TalonSRX leftArmMotor = new TalonSRX(3);

   
    // ポート番号
    private final int rightNeoMotorPort = 0;
    private final int leftNeoMotorPort = 1;
    private final int leftMotor1Port = 2;
    private final int leftMotor2Port = 3;
    private final int rightMotor1Port = 4;
    private final int rightMotor2Port = 5;
    private final int servoPort = 6;
    private final int joystickPort = 0;

    private final Timer m_timer = new Timer();

    // private int lastTargetPosition = 0;
   
    private double shootSpeed = 0.3;

    private final double kDriveTick2Feet = 1.0 / 4096 * 6 * Math.PI / 12;
    // kDriveTick2Feet が、距離を測るための変数で、エンコーダーのギア数とか色々計算してくれてる
    // 6という数字について: ホイールの直径（この例では6インチ）とπを掛け合わせて、ホイールの円周をインチで計算します。ホイールが1回転すると、ロボットはこの距離だけ進むことになります。

    private static final String kLeftAuto = "leftSide";
    private static final String kRightAuto = "rightSide";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

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

        talonSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        talonSRX.setSelectedSensorPosition(0, 0, 10);

        // Creates UsbCamera and MjpegServer [1] and connects them
        CameraServer.startAutomaticCapture();

        lastError = 0;
        lastTimeStamp = 0;

        m_chooser.setDefaultOption("Left Auto", kLeftAuto);
        m_chooser.addOption("Right Auto", kRightAuto);
        SmartDashboard.putData("Auto choices", m_chooser);
    }

    /** This function is run once each time the robot enters autonomous mode. */
    @Override
    public void autonomousInit() {
        m_autoSelected = m_chooser.getSelected();
        System.out.println("Auto selected: " + m_autoSelected);

        m_timer.reset();
        m_timer.start();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        switch (m_autoSelected) {
            case kLeftAuto:
                new WaitCommand(1.0);
                myServo.set(0.22);
                if (m_timer.get() < 4.5) {
                    rightNeoMotor.set(-2);
                    leftNeoMotor.set(2);
                    if(m_timer.get() > 2.0 && m_timer.get() < 4.5) {
                        myServo.set(0.48);
                    }
                } else if (m_timer.get() >= 4.5 && m_timer.get() < 10.0) {
                    rightNeoMotor.set(0);
                    leftNeoMotor.set(0);
                    myServo.set(0.22);
                    leftMotor1.set(limitSpeed(-0.4));
                    leftMotor2.set(limitSpeed(-0.4));
                    rightMotor1.set(limitSpeed(-0.5));
                    rightMotor2.set(limitSpeed(-0.5));
                }
                new WaitCommand(1.0);
                break;
               
            case kRightAuto:
            default:
                new WaitCommand(1.0);
                myServo.set(0.22);
                if (m_timer.get() < 4.5) {
                    rightNeoMotor.set(-2);
                    leftNeoMotor.set(2);
                    if(m_timer.get() > 2.0 && m_timer.get() < 4.5) {
                        myServo.set(0.48);
                    }
                } else if (m_timer.get() >= 4.5 && m_timer.get() < 10.0) {
                    rightNeoMotor.set(0);
                    leftNeoMotor.set(0);
                    myServo.set(0.22);
                    leftMotor1.set(limitSpeed(-0.5));
                    leftMotor2.set(limitSpeed(-0.5));
                    rightMotor1.set(limitSpeed(-0.3));
                    rightMotor2.set(limitSpeed(-0.3));
                }
                new WaitCommand(1.0);
                break;
        }
    }

    @Override
    public void teleopInit() {
        m_timer.reset();
        m_timer.start();
    }

   
    // クラス内のグローバル変数として定義
    double kP = 3; // Pゲイン
    double kD = 0.55; // Dゲイン
    double setpoint = -0.2; // 目標位置
    double lastError = 0; // 前回の偏差
    double lastTimeStamp = 0; // 前回のタイムスタンプ

    @Override
    public void teleopPeriodic() {
       
        boolean key1 = joystick.getRawButton(1);
        // boolean key2 = joystick.getRawButton(2);
        // boolean key3 = joystick.getRawButton(3);
        boolean key4 = joystick.getRawButton(4);
        boolean key5 = joystick.getRawButton(5);
        boolean key6 = joystick.getRawButton(6);
        boolean key7 = joystick.getRawButton(7);
        boolean key8 = joystick.getRawButton(8);
        boolean key9 = joystick.getRawButton(9);
        boolean key10 = joystick.getRawButton(10);

        if (key5) {
            // shoot
            rightNeoMotor.set(-1 * shootSpeed);
            leftNeoMotor.set(shootSpeed);
        } else if (!(key5) && key6) {
            // collect
            rightNeoMotor.set(0.5);
            leftNeoMotor.set(-0.5);
        } else {
            rightNeoMotor.set(0);
            leftNeoMotor.set(0);
        }

        if (key5 && key6) {
            // servo
            myServo.set(0.22);
        } else {
            myServo.set(0.48);
        }


        // set the shooter speed
        if (key7) {
            shootSpeed = 0.4;
        } else if (key8) {
            shootSpeed = 2;
        }

        //ArmMotor
        if (key4) {
            rightArmMotor.set(ControlMode.PercentOutput, 1);
            leftArmMotor.set(ControlMode.PercentOutput, -1);
        } else if (key1) {
            rightArmMotor.set(ControlMode.PercentOutput, -1);
            leftArmMotor.set(ControlMode.PercentOutput, 1);
        } else {
            rightArmMotor.set(ControlMode.PercentOutput, 0);
            leftArmMotor.set(ControlMode.PercentOutput, 0);
        }
       
        // move driveBase
        double left = -1 * joystick.getRawAxis(1);          
        double right = joystick.getRawAxis(5); // ジョイスティックのY軸
        double rightSpeed = right;
        double leftSpeed = left;
        leftMotor1.set(limitSpeed(rightSpeed));
        leftMotor2.set(limitSpeed(rightSpeed));
        rightMotor1.set(limitSpeed(leftSpeed));
        rightMotor2.set(limitSpeed(leftSpeed));

       
        // ボタンによって目標位置を変える
        if (key9) {
            setpoint = -0.2;  
        } else if (key10) {
            setpoint = -0.35;
        }

        // teleopPeriodic内や任意の周期的に呼び出されるメソッド内
        double sensorPosition = talonSRX.getSelectedSensorPosition() * kDriveTick2Feet; // 現在位置の読み取り
        double error = setpoint - sensorPosition; // 偏差の計算

        // 現在の時間を取得し、前回からの経過時間を計算
        double currentTime = m_timer.get();
        double dt = currentTime - lastTimeStamp;

        // 変化率（D成分）の計算
        double errorRate = dt > 0 ? (error - lastError) / dt : 0;

        // 出力の計算
        double outputSpeed = kP * error + kD * errorRate;

        // モーター出力の制限 (-1.0 から 1.0 の範囲内)
        outputSpeed = Math.max(-1.0, Math.min(outputSpeed, 1.0));

        // モーターを動かす
        talonSRX.set(ControlMode.PercentOutput, outputSpeed);

        // スマートダッシュボードへの値の表示（デバッグ用）
        SmartDashboard.putNumber("Output Speed", outputSpeed);
        SmartDashboard.putNumber("Encoder Value", sensorPosition);
        SmartDashboard.putNumber("set Point", setpoint);
        SmartDashboard.putNumber(" Error", error);
        SmartDashboard.putNumber("errorRate", errorRate);
        SmartDashboard.putNumber("dt", dt);

        // 前回の値の更新
        lastError = error;
        lastTimeStamp = currentTime;
    }

    private double limitSpeed(double speed) {
        return Math.max(-0.2, Math.min(0.2, speed));
    }

    /** This function is called once each time the robot enters test mode. */
    @Override
    public void testInit() {}

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
       
    }
}
