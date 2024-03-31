import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;

public class LineFollowingThreaded {
    public static void main(String[] args) {
        // Initialize motors and color sensor
        Brick brick = BrickFinder.getDefault();
        EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.B);
        EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.C);
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S3);


        // Set Motor Speeds
        int baseSpeed = 300;
        int searchSpeed = baseSpeed / 2; // Reduced speed for searching
        leftMotor.setSpeed(baseSpeed);
        rightMotor.setSpeed(baseSpeed);

        // Start color sensing thread
        ColorSensingThread colorThread = new ColorSensingThread(colorSensor, leftMotor, rightMotor, baseSpeed, searchSpeed);
        colorThread.start();

        // Wait for button press to start the program
        System.out.println("Press any button to start...");
        Button.waitForAnyPress();

        // Main program loop
        while (!Button.ESCAPE.isDown()) {
            // Continue looping until the escape button is pressed
        }

        // Stop the color sensing thread and close resources
        colorThread.terminate();
        colorSensor.close();
    }
}

class ColorSensingThread extends Thread {
    private EV3ColorSensor colorSensor;
    private EV3LargeRegulatedMotor leftMotor;
    private EV3LargeRegulatedMotor rightMotor;
    private int baseSpeed;
    private int searchSpeed;
    private volatile boolean running;

    public ColorSensingThread(EV3ColorSensor colorSensor, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, int baseSpeed, int searchSpeed) {
        this.colorSensor = colorSensor;
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.baseSpeed = baseSpeed;
        this.searchSpeed = searchSpeed;
        this.running = true;
    }

    public void run() {
        SampleProvider colorProvider = colorSensor.getColorIDMode();
        float[] sample = new float[colorProvider.sampleSize()];

        while (running) {
            // Read color from the color sensor
            colorProvider.fetchSample(sample, 0);
            int colorId = (int) sample[0];

            if (colorId == 7) {
                // Color ID 7: Line detected
                leftMotor.forward();
                rightMotor.forward();
                System.out.println("Line detected");
            } else {
                // Line missing
                // Curve handling
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
            }

            // Add a small delay to control loop execution frequency
            Delay.msDelay(10);
        }
    }

    public void terminate() {
        running = false;
    }
}
