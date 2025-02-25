package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoTimer extends SubsystemBase {

    private DoublePublisher m_autoTimePub;
    private Timer m_autoTimer;

    public AutoTimer() {
        m_autoTimer = new Timer();
        m_autoTimePub = NetworkTableInstance.getDefault()
                .getTable("Autonomous")
                .getDoubleTopic("Completion Time")
                .publish();
    }

    public void start() {
        m_autoTimer.start();
    }

    public void stop() {
        m_autoTimer.stop();
    }       

    public void reset() {
        m_autoTimer.reset();
    }   

    public void resetAndStart() {
        m_autoTimer.reset();
        m_autoTimer.start();
    }  

    public void publish() {
        m_autoTimePub.set(m_autoTimer.get());
    }   
    
    public void stopAndPublish() {
        m_autoTimer.stop();
        publish();
    }
}
