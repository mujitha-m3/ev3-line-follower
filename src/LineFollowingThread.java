import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.utility.Delay;

public class LineFollowingThread extends Thread {
    private EV3LargeRegulatedMotor leftMotor;
    private EV3LargeRegulatedMotor rightMotor;
    private ColorDetectionThread colorDetectionThread;
    private ObstacleDetectionThread obstacleDetectionThread;
    private static final int BASE_SPEED = 200;
    private static final int SEARCH_SPEED = BASE_SPEED / 2;

    public LineFollowingThread(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, ColorDetectionThread colorDetectionThread, ObstacleDetectionThread obstacleDetectionThread) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.colorDetectionThread = colorDetectionThread;
        this.obstacleDetectionThread = obstacleDetectionThread;
        // Set motor speeds
        leftMotor.setSpeed(BASE_SPEED);
        rightMotor.setSpeed(BASE_SPEED);
    }

    @Override
    public void run() {
        boolean lineFound = false;
        while (!Thread.currentThread().isInterrupted()) {
            int colorId = colorDetectionThread.getLineColorId();
            boolean obstacleDetected = obstacleDetectionThread.isObstacleDetected();

            if (obstacleDetected && !lineFound) {
                System.out.println("Obstacle found...");
                // Code for avoiding the obstacle
            } else {
                System.out.println("Obstacle not found...");
            }

            // Add a small delay to control loop execution frequency
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }
}
