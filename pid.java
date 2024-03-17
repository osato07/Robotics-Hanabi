package frc.robot;

import edu.wpi.first.wpilibj.PIDController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// Import other necessary classes

public class Robot extends TimedRobot {
    private PIDController pid;
    private Joystick joystick = new Joystick(0);
    private TalonSRX TalonSRX = new TalonSRX(1);
    private Encoder encoder(0, 1, true, EncodingType.k4X);

    private final double kDriveTick2Feet = 1.0 / 128 * 6 * Math.PI / 12;

    @Override
    public void robotInit() {        
        encoder.reset();
        lastError = 0;
        lastTimeStamp = Timer.getFPGATimeStamp();
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
            setpoint = 300
        }

        double sensorPosition = encoder.get() * kDriveTick2Feet;

        double error = setpoint - sensorPosition;
        double dt = Timer.getFPGATimeStamp() - lastTimeStamp;
        if (Math.abs(error) < iLimit) {
            errorSum += error * dt;
        }

        double errorRate = (error - lastError) / dt;

        double outputSpeed = kP * error + kI * errorSum + kD * errorRate;
        

        TalonSRX.set(Position, outputSpeed);
        
        lastTimeStamp = Timer.getFPGATimeStamp();
        SmartDashboard.putNumber("encoder value", encoder.get() * kDriveTick2Feet);
        lastError = error;
    }

    
}
