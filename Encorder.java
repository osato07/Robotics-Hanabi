import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class Robot extends TimedRobot {
    private WPI_TalonSRX talon = new WPI_TalonSRX(1); // ID 1のTalon SRX
    private Joystick joystick = new Joystick(0); // ポート0に接続されたジョイスティック
    private JoystickButton button1 = new JoystickButton(joystick, 1); // ボタン1
    private JoystickButton button2 = new JoystickButton(joystick, 2); // ボタン2

    private final double degreesToEncoderUnits = 4096 * 100 / 360.0; // 1度あたりのエンコーダー単位

    @Override
    public void teleopPeriodic() {
        if (button1.get()) {
            talon.set(ControlMode.Position, 30 * degreesToEncoderUnits); // 30度の位置に移動
        } else if (button2.get()) {
            talon.set(ControlMode.Position, 120 * degreesToEncoderUnits); // 120度の位置に移動
        }
    }
}
