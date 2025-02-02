import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.utility.Delay;

public class LineFollowing {
    public static void main(String[] args) {
        EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.B);
        EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.C);
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S3);
        EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S1);

        // Create color detection thread
        ColorDetectionThread colorDetectionThread = new ColorDetectionThread(colorSensor);
        colorDetectionThread.start();

        // Create obstacle detection thread
        ObstacleDetectionThread obstacleDetectionThread = new ObstacleDetectionThread(ultrasonicSensor);
        obstacleDetectionThread.start();

        // Create line following thread
        LineFollowingThread lineFollowingThread = new LineFollowingThread(leftMotor, rightMotor, colorDetectionThread, obstacleDetectionThread);
        lineFollowingThread.start();

        // Wait for button press to start the program.
        System.out.println("Press any button to start...");
        Button.waitForAnyPress();

        while (!Button.ESCAPE.isDown()) {
            // Add a small delay to control loop execution frequency
            Delay.msDelay(10);
        }

        // Stop the motors and close the sensors when the program ends
        leftMotor.stop(true);
        rightMotor.stop();
        colorSensor.close();
        ultrasonicSensor.close();
    }
}