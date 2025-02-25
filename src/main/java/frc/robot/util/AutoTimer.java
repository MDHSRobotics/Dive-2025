package frc.robot.util;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class AutoTimer {

    private DoublePublisher m_autoTimePub = NetworkTableInstance.getDefault()
            .getTable("PathPlanner")
            .getDoubleTopic("Completion Time")
            .publish();
    private Timer m_autoTimer = new Timer();

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
