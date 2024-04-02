import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;


public class LineFollowing {
    public static void main(String[] args) {
        Brick brick = BrickFinder.getDefault();
        EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.B);
        EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.C);
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S3); // Changed to SensorPort.S3
        EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S1); // Ultrasonic sensor on port 1



        // Set motor speeds
        Int baseSpeed = 200;
        Int searchSpeed = baseSpeed / 1; // Reduced speed for searching
        leftMotor.setSpeed(baseSpeed);
        rightMotor.setSpeed(baseSpeed);

        // Get the color ID mode of the color sensor
        SampleProvider colorProvider = colorSensor.getColorIDMode();
        float[] sample = new float[colorProvider.sampleSize()];

        boolean lineFound = false;

        // Create obstacle detection thread
        ObstacleDetectionThread obstacleDetectionThread = new ObstacleDetectionThread(leftMotor, rightMotor, ultrasonicSensor);
        obstacleDetectionThread.start();

        // Wait for button press to start the program.
        System.out.println("Press any button to start...");
        Button.waitForAnyPress();

        try {
            while (!Button.ESCAPE.isDown()) {
                // Read color from the color sensor
                colorProvider.fetchSample(sample, 0);
                int colorId = (int) sample[0];

                if (colorId == 7) {
                    // Color ID 7: Line detected
                    leftMotor.forward();
                    rightMotor.forward();
                    lineFound = true;
                    System.out.println("Line detected");
                } else {
                    // Line missing
                    if (lineFound) {
                        // Curve handling
                        System.out.println("Line missing, curve handling...");
                        leftMotor.setSpeed(searchSpeed / 2);
                        rightMotor.setSpeed(baseSpeed);
                        leftMotor.forward();
                        rightMotor.forward();
                        Delay.msDelay(100); // Adjust time as needed
                        leftMotor.setSpeed(baseSpeed);
                        rightMotor.setSpeed(searchSpeed / 2);
                        leftMotor.forward();
                        rightMotor.forward();
                        Delay.msDelay(200); // Adjust time as needed
                        System.out.println("Curve handling complete");
                        // Continue searching for line
                        leftMotor.backward();
                        rightMotor.forward();
                        lineFound = false;
                    } else {
                        // Continue searching for line
                        leftMotor.backward();
                        rightMotor.forward();
                    }
                }

                // Add a small delay to control loop execution frequency
                Delay.msDelay(10);
            }
        } finally {
            // Stop the motors and close the color sensor and ultrasonic sensor when the program ends
            leftMotor.stop(true);
            rightMotor.stop();
            colorSensor.close();
            ultrasonicSensor.close();
        }
    }

    static class ObstacleDetectionThread extends Thread {
        private EV3LargeRegulatedMotor leftMotor;
        private EV3LargeRegulatedMotor rightMotor;
        private EV3UltrasonicSensor ultrasonicSensor;

        public ObstacleDetectionThread(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, EV3UltrasonicSensor ultrasonicSensor) {
            this.leftMotor = leftMotor;
            this.rightMotor = rightMotor;
            this.ultrasonicSensor = ultrasonicSensor;
        }

        @Override
        public void run() {
            SampleProvider distanceProvider = ultrasonicSensor.getDistanceMode();
            float[] sample = new float[distanceProvider.sampleSize()];

            while (!Button.ESCAPE.isDown()) {
                // Read distance from the ultrasonic sensor
                distanceProvider.fetchSample(sample, 0);
                float distance = sample[0];

                // Implement obstacle detection logic here
                // For simplicity, let's assume obstacle detection involves checking the distance
                // If an obstacle is detected, stop the motors

                // Simulated obstacle detection logic (you need to replace this with actual obstacle detection)
                if (distance < 0.2) { // Adjust the distance threshold as needed
                    // Obstacle detected, perform action
                    System.out.println("Obstacle detected. Stopping motors.");
                    Sound.beepSequenceUp(); 
                    leftMotor.stop(true);
                    rightMotor.stop();
                }

                // Add a small delay to control loop execution frequency
                Delay.msDelay(50);
            }
        }
    }
}
