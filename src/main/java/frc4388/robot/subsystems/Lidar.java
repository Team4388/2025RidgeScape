package frc4388.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc4388.robot.Constants.LiDARConstants;
import frc4388.utility.Status;
import frc4388.utility.Subsystem;
import frc4388.utility.Status.ReportLevel;

// https://girlsofsteeldocs.readthedocs.io/en/latest/technical-resources/sensors/LIDAR-Lite-Distance-Sensor.html#minimal-roborio-interface
public class Lidar extends Subsystem {

    private double distance = -1;
    Counter LidarPWM = new Counter(LiDARConstants.LIDAR_DIO_CHANNEL);

    public Lidar() {
        LidarPWM.setMaxPeriod(1.00); //set the max period that can be measured
        LidarPWM.setSemiPeriodMode(true); //Set the counter to period measurement
        LidarPWM.reset();
    }

    @Override
    public void periodic() {
        if(LidarPWM.get() < 1)
            distance = -1;
        else
            distance = (LidarPWM.getPeriod() * LiDARConstants.SECONDS_TO_MICROS) / LiDARConstants.LIDAR_MICROS_TO_CM;
    }

    public double getDistance(){
        return distance;
    }

    public boolean withinDistance(){
        if(distance == -1) return false;
        return distance < LiDARConstants.LIDAR_DETECT_DISTANCE;
    }

    ShuffleboardLayout subsystemLayout = Shuffleboard.getTab("Subsystems")
            .getLayout(getSubsystemName(), BuiltInLayouts.kList)
            .withSize(2, 2);

    GenericEntry sbDistance = subsystemLayout
            .add("Distance", 0)
            .withWidget(BuiltInWidgets.kGraph)
            .getEntry();

    GenericEntry sbWithinDistance = subsystemLayout
            .add("Within Distance", 0)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .getEntry();

    @Override
    public String getSubsystemName() {
        return "Lidar";
    }

    @Override
    public void queryStatus() {
        sbDistance.setDouble(distance);
        sbWithinDistance.setBoolean(withinDistance());
    }

    @Override
    public Status diagnosticStatus() {
        Status s = new Status();
        if(distance == -1){
            s.addReport(ReportLevel.ERROR, "LIDAR DISCONNECTED");
        }else{
            s.addReport(ReportLevel.INFO, "LIDAR CONNECTED");
        }
        return s;
    }
    
}
