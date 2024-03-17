package frc.robot;

import edu.wpi.first.wpilibj.PIDController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;

    
public class Robot extends TimedRobot {
    private PIDController pid;
    private Joystick joystick = new Joystick(0);
    private TalonSRX talonSRX = new TalonSRX(1);
    private Encoder encoder = new Encoder(0, 1, true, EncodingType.k4X);

    private final double kDriveTick2Feet = 1.0 / 128 * 6 * Math.PI / 12;

    private final Timer m_timer = new Timer();

    @Override
    public void robotInit() {        
        // コードがデプロイされた、本当に最初の時に実行される
        encoder.reset();
        lastError = 0;
        lastTimeStamp = 0;
    }

    @Override
    public void autonomousInit() {
        m_timer.restart();
    }
 
    final double kP = 0.0007; 
    final double kD = 0.01;

    double setpoint = 0;
    double lastTimeStamp = 0;
    double lastError = 0;

    @Override
    public void teleopPeriodic() {
        // ひとまず、dashboardだけで、正確な値の方に encoder.get or encoder.getDistanceやってから追加
        SmartDashboard.putNumber("Encoder getDistance", encoder.getDistance());
        SmartDashboard.putNumber("Encoder get", encoder.get());

        // pid cotrole by PID class, it is so easy yeah but "easy" are followed by "complex"
        // PIDController pid = new PIDController(kP, kI, kD);
        // motor.set(pid.calculate(encoder.getDistance(), setpoint));
        
        
        if (joystick.getRawButton(9)) {
            setpoint = 0;   
        } else if (joystick.getRawButton(10)) {
            setpoint = 300;
            // 目標角度のことを指してるけど、うまくできてるかはわからん
        }

        double sensorPosition = encoder.get() * kDriveTick2Feet;

        double error = setpoint - sensorPosition;
        double dt = m_timer.get() - lastTimeStamp;

        double errorRate = (error - lastError) / dt;

        double outputSpeed = kP * error + kD * errorRate;
        SmartDashboard.putNumber("output ", outputSpeed);
        
        // talonSRX.set(ControlMode.Position, outputSpeed);
        
        lastTimeStamp = m_timer.get();
        SmartDashboard.putNumber("encoder value", encoder.get() * kDriveTick2Feet);
        lastError = error;
    }

    
}
