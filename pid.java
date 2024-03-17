package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Encoder.EncodingType;
    
public class Robot extends TimedRobot {
    private PIDController pid;
    private Joystick joystick = new Joystick(0);
    private TalonSRX talonSRX = new TalonSRX(1);
    private Encoder encoder = new Encoder(0, 1, true, EncodingType.k4X); // encoderの読み取り速度？質？を4倍にする

    private final double kDriveTick2Feet = 1.0 / 4096 * 6 * Math.PI / 12;
    // kDriveTick2Feet が、距離を測るための変数で、エンコーダーのギア数とか色々計算してくれてる
    // 6という数字について: ホイールの直径（この例では6インチ）とπを掛け合わせて、ホイールの円周をインチで計算します。ホイールが1回転すると、ロボットはこの距離だけ進むことになります。

    private final Timer m_timer = new Timer();

    @Override
    public void robotInit() {        
        // コードがデプロイされた、本当に最初の時に実行される
        encoder.reset();
        lastError = 0;
        lastTimeStamp = 0;
    }

    @Override
    public void teleopPeriodic() {
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
        // ひとまず、dashboardだけで、正確な値の方に encoder.get or encoder.getDistanceやってから追加
        SmartDashboard.putNumber("Encoder getDistance", encoder.getDistance());
        SmartDashboard.putNumber("Encoder get", encoder.get());

        
        // PID controller　っていうクラスを使えば簡単に行きそうだけど、簡単すぎて怖いからコメントアウトしとく
        
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html 
        // PIDController pid = new PIDController(kP, kI, kD);
        // motor.set(pid.calculate(encoder.getDistance(), setpoint));

        
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
        
        lastTimeStamp = m_timer.get();
        lastError = error;
    }
}
