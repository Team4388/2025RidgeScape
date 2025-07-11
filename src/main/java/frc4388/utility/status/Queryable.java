package frc4388.utility.status;

public interface Queryable {
    // Get name of subsystem, for use in log.
    String getName();
    // Get what the subystem is currently doing, such as "Shooter spun up". This should post to SmartDashboard
    void queryStatus();
    // Proactivly search for any errors in each subsystem
    Status diagnosticStatus(); 
}
