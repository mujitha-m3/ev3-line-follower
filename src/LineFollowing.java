import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.utility.Delay;

public class LineFollowing {
    public static void main(String[] args) {
        EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.B);
        EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.C);
        EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S1);

        // Create color detection thread
        ColorDetectionThread colorDetectionThread = new ColorDetectionThread();
        colorDetectionThread.start();

        // Create obstacle detection thread
        ObstacleDetectionThread obstacleDetectionThread = new ObstacleDetectionThread(ultrasonicSensor, leftMotor, rightMotor);
        obstacleDetectionThread.start();

        // Create line following thread
        LineFollowingThread lineFollowingThread = new LineFollowingThread(leftMotor, rightMotor, colorDetectionThread, obstacleDetectionThread);
        lineFollowingThread.start();

        // Create time tracker thread
        LineFollowingTimeTracker timeTrackerThread = new LineFollowingTimeTracker();
        timeTrackerThread.start();

        // Wait for button press to start the program.
        System.out.println("Press any button to start...");
        Button.waitForAnyPress();

        // Main loop
        while (true) {
            if (Button.ESCAPE.isDown()) {
                // Stop the motors if ESCAPE button is pressed
                leftMotor.stop(true);
                rightMotor.stop();
                break; // Exit the loop
            }
            Delay.msDelay(10); // Delay to avoid busy waiting
        }

        // Close the sensors when the program ends
        ultrasonicSensor.close();
    }
}
