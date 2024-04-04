import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LineFollowing {
    // Define the color ID of the line
    private static final int lineColorId = 7;

    public static void main(String[] args) {
        Brick brick = BrickFinder.getDefault();
        EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.B);
        EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.C);
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S3); 
        EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S1); // Ultrasonic sensor on port 1

        
        int baseSpeed = 320;
        leftMotor.setSpeed(baseSpeed);
        rightMotor.setSpeed(baseSpeed);

       
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

    static class ColorDetectionThread extends Thread {
        private EV3ColorSensor colorSensor;
        private int lineColorId;

        public ColorDetectionThread(EV3ColorSensor colorSensor) {
            this.colorSensor = colorSensor;
        }

        @Override
        public void run() {
            SampleProvider colorProvider = colorSensor.getColorIDMode();
            float[] sample = new float[colorProvider.sampleSize()];

            while (!Button.ESCAPE.isDown()) {
                colorProvider.fetchSample(sample, 0);
                lineColorId = (int) sample[0];
            }
            colorSensor.close();
        }

        public int getLineColorId() {
            return lineColorId;
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
                    //Sound.beepSequenceUp(); 
                    leftMotor.stop(true);
                    rightMotor.stop();
                }

                // Add a small delay to control loop execution frequency
                Delay.msDelay(50);
            }
        }
    }
}
