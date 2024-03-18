package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;

public class Robot extends TimedRobot {
    private PWMSparkMax leftMotor1, leftMotor2, rightMotor1, rightMotor2;
    private PWMSparkMax rightNeoMotor, leftNeoMotor;
    private Joystick joystick;
    private Servo myServo;
    TalonSRX talonSRX = new TalonSRX(1);
    TalonSRX rightArmMotor = new TalonSRX(2);
    TalonSRX leftArmMotor = new TalonSRX(3);

    private Encoder encoder = new Encoder(0, 1, true, EncodingType.k4X); // encoderの読み取り速度？質？を4倍にする
   
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
    private double autonomousSpeed = 0;


    private final double kDriveTick2Feet = 1.0 / 4096 * 6 * Math.PI / 12;
    // kDriveTick2Feet が、距離を測るための変数で、エンコーダーのギア数とか色々計算してくれてる
    // 6という数字について: ホイールの直径（この例では6インチ）とπを掛け合わせて、ホイールの円周をインチで計算します。ホイールが1回転すると、ロボットはこの距離だけ進むことになります。

    private static final String kLeftAutp = "leftSide";
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

        // Creates UsbCamera and MjpegServer [1] and connects them
        CameraServer.startAutomaticCapture();

        encoder.reset();
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
            if (m_timer.get() < 2.0) {
                // Drive forwards half speed, make sure to turn input squaring off
                autonomousSpeed = 0.1;
                leftMotor1.set(limitSpeed(autonomousSpeed));
                leftMotor2.set(limitSpeed(autonomousSpeed));
                rightMotor1.set(limitSpeed(autonomousSpeed));
                rightMotor2.set(limitSpeed(autonomousSpeed));
            } else if (m_timer.get() >= 2.0 && m_timer.get() < 4.0) {
                autonomousSpeed = -0.3;
                leftMotor1.set(limitSpeed(autonomousSpeed));
                leftMotor2.set(limitSpeed(autonomousSpeed));
                rightMotor1.set(limitSpeed(autonomousSpeed));
                rightMotor2.set(limitSpeed(autonomousSpeed));        
            } else {
                autonomousSpeed = 0;
                leftMotor1.set(limitSpeed(0));
                leftMotor2.set(limitSpeed(0));
                rightMotor1.set(limitSpeed(0));
                rightMotor2.set(limitSpeed(0));   
            }
            new WaitCommand(5.0);
            
            break;
                
          case kRightAuto:
          default:
            if (m_timer.get() < 2.0) {
                // Drive forwards half speed, make sure to turn input squaring off
                autonomousSpeed = 0.1;
                leftMotor1.set(limitSpeed(autonomousSpeed));
                leftMotor2.set(limitSpeed(autonomousSpeed));
                rightMotor1.set(limitSpeed(autonomousSpeed));
                rightMotor2.set(limitSpeed(autonomousSpeed));
            } else if (m_timer.get() >= 2.0 && m_timer.get() < 4.0) {
                autonomousSpeed = -0.3;
                leftMotor1.set(limitSpeed(autonomousSpeed));
                leftMotor2.set(limitSpeed(autonomousSpeed));
                rightMotor1.set(limitSpeed(autonomousSpeed));
                rightMotor2.set(limitSpeed(autonomousSpeed));        
            } else {
                autonomousSpeed = 0;
                leftMotor1.set(limitSpeed(0));
                leftMotor2.set(limitSpeed(0));
                rightMotor1.set(limitSpeed(0));
                rightMotor2.set(limitSpeed(0));   
            }
            new WaitCommand(5.0);
            
            break;
        }
    }

    @Override
    public void teleopInit() {
        m_timer.reset();
        m_timer.start();
    }

    
    final double kP = 0.0007;
    final double kD = 0.01;

    double setpoint = 0; // 目標距離の変数
    double lastTimeStamp = 0; // 最新の時間
    double lastError = 0; // 最新のerror（目標までの距離）


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

        // 315 add feedback device 
        if (key10) {
            double feedforward = -0.1;
            int targetPos = 636;
            talonSRX.set(ControlMode.MotionMagic, targetPos, DemandType.ArbitraryFeedForward, feedforward);
        } else if (key9) {
            double feedforward = 0.1;
            int targetPos = 1600;
            talonSRX.set(ControlMode.MotionMagic, targetPos, DemandType.ArbitraryFeedForward, feedforward);
        } else {
            talonSRX.set(ControlMode.PercentOutput, 0);
        }



        SmartDashboard.putNumber("Encoder Position", talonSRX.getSelectedSensorPosition());
        SmartDashboard.putNumber("Target Position", lastTargetPosition);


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
        SmartDashboard.putNumber("right Position", rightSpeed);
        SmartDashboard.putNumber("left Position", leftSpeed);
        leftMotor1.set(limitSpeed(rightSpeed));
        leftMotor2.set(limitSpeed(rightSpeed));
        rightMotor1.set(limitSpeed(leftSpeed));
        rightMotor2.set(limitSpeed(leftSpeed));



        // ひとまず、dashboardだけで、正確な値の方に encoder.get or encoder.getDistanceやってから追加
        SmartDashboard.putNumber("Encoder getDistance", encoder.getDistance());
        SmartDashboard.putNumber("Encoder get", encoder.get());
        
        // ボタンによって目標位置を変える
        if (joystick.getRawButton(9)) {
            setpoint = 0;  
        } else if (joystick.getRawButton(10)) {
            setpoint = 300;
        }
        
        double sensorPosition = encoder.get() * kDriveTick2Feet;
        
        double error = setpoint - sensorPosition; // 目標までの距離
        double dt = m_timer.get() - lastTimeStamp;

        double errorRate = (error - lastError) / dt;

        // outputSpeed そのまま
        double outputSpeed = kP * error + kD * errorRate;
        SmartDashboard.putNumber("output ", outputSpeed);
        
        // テストの時までコメントアウトする、モーターを動かすコード
        // talonSRX.set(ControlMode.PercentOutput, outputSpeed);

        SmartDashboard.putNumber("encoder value", encoder.get() * kDriveTick2Feet);

        // update last data
        lastTimeStamp = m_timer.get();
        lastError = error;
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
