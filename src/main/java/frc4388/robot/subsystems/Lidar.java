package frc4388.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.constants.Constants.LiDARConstants;
import frc4388.utility.status.Status;
import frc4388.utility.status.FaultReporter;
import frc4388.utility.status.Queryable;
import frc4388.utility.status.Status.ReportLevel;

// https://girlsofsteeldocs.readthedocs.io/en/latest/technical-resources/sensors/LIDAR-Lite-Distance-Sensor.html#minimal-roborio-interface
public class Lidar extends SubsystemBase implements Queryable {

    private Counter LidarPWM;
    private String name = "Lidar";

    private double distance = -1;
    public Lidar(int port, String name) {
        FaultReporter.register(this);

        this.name = name;
        LidarPWM = new Counter(port);
        LidarPWM.setMaxPeriod(1.00); //set the max period that can be measured
        LidarPWM.setSemiPeriodMode(true); //Set the counter to period measurement
        LidarPWM.reset();

        
    subsystemLayout = Shuffleboard.getTab("Subsystems")
    .getLayout(getName(), BuiltInLayouts.kList)
    .withSize(2, 2);

    sbDistance = subsystemLayout
    .add("Distance", 0)
    .withWidget(BuiltInWidgets.kGraph)
    .getEntry();

    sbWithinDistance = subsystemLayout
    .   add("Within Distance", 0)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();
    }

    @Override
    public void periodic() {
        if(LidarPWM.get() < 1)
            distance = -1;
        else
            distance = (LidarPWM.getPeriod() * LiDARConstants.SECONDS_TO_MICROS) / LiDARConstants.LIDAR_MICROS_TO_CM;
    }

    @AutoLogOutput
    public double getDistance(){
        return distance;
    }

    public boolean withinDistance(){
        if(distance == -1) return false;
        return distance < LiDARConstants.LIDAR_DETECT_DISTANCE;
    }

    ShuffleboardLayout subsystemLayout;
    GenericEntry sbDistance;
    GenericEntry sbWithinDistance;

    @Override
    public String getName() {
        return "Lidar " + name;
    }

    // @Override
    // public void queryStatus() {
    //     sbDistance.setDouble(distance);
    //     sbWithinDistance.setBoolean(withinDistance());
    // }

    @Override
    public Status diagnosticStatus() {
        Status s = new Status();

        if(distance == -1)
            s.addReport(ReportLevel.ERROR, "LiDAR DISCONNECTED");
        
        s.addReport(ReportLevel.INFO, "LiDAR Distance: " + distance);

        return s;
    }
    
}
