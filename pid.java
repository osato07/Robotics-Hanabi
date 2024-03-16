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
        errorSum = 0;
        lastError = 0;
        lastTimeStamp = Timer.getFPGATimeStamp();
    }

    final double kP = 0.0007; 
    final double kI = 0.0007;
    final double kD = 0.01;
    final double iLimit = 1;

    double setpoint = 0;
    double errorSum = 0;
    double lastTimeStamp = 0;
    double lastError = 0;

    @Override
    public void teleopPeriodic() {
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
