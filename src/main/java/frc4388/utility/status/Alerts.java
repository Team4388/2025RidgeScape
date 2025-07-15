package frc4388.utility.status;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc4388.robot.RobotContainer;

// Class to update a series of WPILIB Alerts
public class Alerts {
  private static Alert no_auto = new Alert("No auto has been selected!", AlertType.kError);

  public static void UpdateAlerts(RobotContainer m_RobotContainer) {
    no_auto.set(!m_RobotContainer.autoChooserUpdated);
  }
}
