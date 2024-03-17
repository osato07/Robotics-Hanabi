package frc.robot;

// the import section is omitted...

public class Robot extends TimedRobot {
    private static final String kDefaultAuto = "leftSide";
    private static final String kCustomAuto = "rightSide";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    @Override
    public void robotInit() {
        m_chooser.setDefaultOption("Left Auto", kDefaultAuto);
        m_chooser.addOption("Right Auto", kCustomAuto);
        SmartDashboard.putData("Auto choices", m_chooser);
    }

    /** This function is run once each time the robot enters autonomous mode. */
    @Override
    public void autonomousInit() {
        m_autoSelected = m_chooser.getSelected();
        System.out.println("Auto selected: " + m_autoSelected);
        m_timer.restart();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        switch (m_autoSelected) {
          case kCustomAuto:
            if (m_timer.get() < 2.0) {
                System.out.println("autoMa");
            } else {
                System.out.println("autoMi");
            }
            new WaitCommand(5.0);
            
            break;
          case kDefaultAuto:
          default:
            if (m_timer.get() < 2.0) {
                System.out.println("autoMa");
            } else {
                System.out.println("autoMi");
            }
            new WaitCommand(5.0);
            
            break;
        }
    }

    @Override
    public void teleopPeriodic() {}

    /** This function is called once each time the robot enters test mode. */
    @Override
    public void testInit() {}

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
        
    }
}
