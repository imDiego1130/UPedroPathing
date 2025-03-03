package Teamcode;

import static com.qualcomm.robotcore.util.Range.scale;
import static java.lang.Math.abs;

import android.annotation.SuppressLint;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp DOUBLE Driver RED", group="TeleOp Double")
public class TeleOpDoubleDriverRed extends OpMode {
    /* Declare OpMode members. */
    Hardware robot = new Hardware();
    double MAX_SPEED = .7;
    double Arm = 0;
    int Lift = 0;
    double Slide = 0;
    double Depositarm = 0;
    double Gripper = 0;

    ElapsedTime timer = new ElapsedTime();
    boolean isGripperClosed = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.rB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hey Driver");

        //robot.liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Slide = GlobalConstants.slideBack;
        Arm = GlobalConstants.intakeArmUp;
        Depositarm = GlobalConstants.depositIn;
        Lift = GlobalConstants.liftStart;
        Gripper = GlobalConstants.gripperOpen;

        intakeArm(Arm);
        slideLinkage(Slide);
        deposit(Depositarm);
        lift_POS(Lift);
        grip(Gripper);
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @SuppressLint("SuspiciousIndentation")
    @Override
    public void loop() {
        if (gamepad1.left_stick_button) {
            MAX_SPEED = .3;
        }
        if (gamepad1.right_stick_button) {
            MAX_SPEED = .7;
        }
        double Speed = -gamepad1.left_stick_y;
        double Turn = gamepad1.right_stick_x;
        double Strafe = -gamepad1.left_stick_x;
        holonomic(Speed, Turn, Strafe, MAX_SPEED);
        telemetry.addData("MAX Speed", "%.2f", MAX_SPEED);

        //INTAKE CODE BEGINS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        if (Lift >= 400){
            MAX_SPEED = .3;
        }
        if (gamepad1.left_trigger > 0){
            intake(1);
        } else if (gamepad1.right_trigger > 0){
            intake(-1);
        } else {
            intake(0);
        }
        if (gamepad1.right_trigger > 0){
            //Slide = GlobalConstants.slideMid;
            Arm = GlobalConstants.intakeArmDown;
        }
        if (gamepad1.dpad_up) {
            if (Slide == GlobalConstants.slideBack) {
                Slide = GlobalConstants.slideMid;
                Arm = GlobalConstants.intakeArmMid;
            } else if (Slide == GlobalConstants.slideMid) {
                Slide = GlobalConstants.slideOut;
            }
        }
        //LIFT CODE BEGINS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        if (gamepad1.y){
            Lift = GlobalConstants.sampleHIGH;
            Arm = GlobalConstants.intakeArmMid;
            Depositarm = GlobalConstants.depositOut;
        }
        if (gamepad1.b){
            Lift = GlobalConstants.specimenHIGH;
            Depositarm = GlobalConstants.depositMid;
        }
        if (gamepad1.x){
            Arm = GlobalConstants.intakeArmMid;
            Lift = GlobalConstants.sampleLOW;
            Depositarm = GlobalConstants.depositOut;
        }
        if (gamepad1.a){
            Lift = GlobalConstants.liftStart;
            Arm = GlobalConstants.intakeArmMid;
            Depositarm = GlobalConstants.depositIn;
            Gripper = GlobalConstants.gripperOpen;
            Slide = GlobalConstants.slideBack;
        }
        if (gamepad1.dpad_down){
            Depositarm = GlobalConstants.depositS;
        }
        if (gamepad1.left_bumper){
            Gripper = GlobalConstants.gripperOpen;
        }
        if (gamepad1.right_bumper){
            if ((Arm > GlobalConstants.intakeArmUp) && (Slide == GlobalConstants.slideBack)){
                Slide = GlobalConstants.slideMid;
            }else if (Slide == GlobalConstants.slideMid) {
                Slide = GlobalConstants.slideOut;
            }
        }
        if(gamepad2.right_bumper){
            Gripper = GlobalConstants.gripperClose;
        }
        if (gamepad2.left_bumper){
            Gripper = GlobalConstants.gripperOpen;
        }
        if(gamepad2.y){
            Lift = GlobalConstants.sampleHIGH;
            Depositarm = GlobalConstants.depositOut;
        }
        if(gamepad2.a){
            Lift = GlobalConstants.liftStart;

        }
        //MANUAL MODE BEGINS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        if (gamepad2.dpad_right){
            Slide = GlobalConstants.slideOut;
        }
        if (gamepad2.dpad_down && Lift > -5){  // changelift linkage (hopefully) wont break again cuz the lift wont be able to go down when its alrdy at the lowest
            Lift -= 12;
        }
        if (gamepad2.right_stick_y > 0) {
            Slide -= .01;
        }
        if (gamepad2.right_stick_y < 0) {
            Slide += .01;
        }
        if (gamepad2.dpad_up) {
            Lift += 12;
        }
        if (gamepad1.dpad_left){
            Depositarm -= 0.01;
        }
        if (gamepad1.dpad_right){
            Depositarm += 0.01;
        }

//////     ///////   ////// color/distance sensor stuff ////////          /////       /////      ////////////     ///////
        if (Arm > GlobalConstants.intakeArmUp) {

            // Read color sensor values
            int red = robot.intakeColor.red();
            int green = robot.intakeColor.green();
            int blue = robot.intakeColor.blue();

            // Convert RGB to HSV
            float[] hsv = new float[3];
            Color.RGBToHSV(red, green, blue, hsv);

            float hue = hsv[0];        // Hue angle (0-360 degrees)
            float saturation = hsv[1]; // Saturation (0-1)
            float value = hsv[2];      // Value/Brightness (0-1)

            // Check for low intensity (no sample detected)
            if (value > 0.2) { // Avoid noise under low light

                // Detect RED sample
                if (hue < 30 || hue > 330) {
                    telemetry.addData("Detected Color: RED", "Hue: %.2f", hue);
                    Arm = GlobalConstants.intakeArmUp;
                    Slide = GlobalConstants.slideBack;
                }
                // Detect YELLOW sample
                else if (hue > 40 && hue < 80) {
                    telemetry.addData("Detected Color: YELLOW", "Hue: %.2f", hue);
                    Arm = GlobalConstants.intakeArmUp;
                    Slide = GlobalConstants.slideBack;

                }
                // Detect BLUE sample
                else if (hue > 200 && hue < 260) {
                    telemetry.addData("Detected Color: BLUE", "Hue: %.2f", hue);
                    Arm = GlobalConstants.intakeArmDown;
                    Slide = GlobalConstants.slideOut;
                    gamepad2.rumble(550);
                    intake(1);

                }
                // Unknown Color
                else {
                    telemetry.addData("Unknown Color", "Hue: %.2f", hue);
                }
            } else {
                telemetry.addData("No Sample Detected", "Value too low: %.2f", value);
            }
        }
        double distance = robot.armDistance.getDistance(DistanceUnit.INCH);
        if (distance <= 0.4) {
            // Read color sensor values
            int red = robot.intakeColor.red();
            int green = robot.intakeColor.green();
            int blue = robot.intakeColor.blue();

            // Convert RGB to HSV
            float[] hsv = new float[3];
            Color.RGBToHSV(red, green, blue, hsv);

            float hue = hsv[0];        // Hue angle (0-360 degrees)
            float saturation = hsv[1]; // Saturation (0-1)
            float value = hsv[2];      // Value/Brightness (0-1)

            // Check for low intensity (no sample detected)
            if (value > 0.2) { // Avoid noise under low light

                // Detect RED sample
                if (hue < 30 || hue > 330) {
                    //Gripper = GlobalConstants.gripperClose;
                    closeGripper();
                    gamepad1.rumble(600);
                    telemetry.addData("Detected Color: RED", "Hue: %.2f", hue);
                    isGripperClosed = true;
                    timer.reset();
                    if (isGripperClosed && timer.seconds() > 0.5) {
                        // Move the DepositArm to depositMid position
                        Depositarm = GlobalConstants.depositMid;
                        deposit(Depositarm);  // Call the deposit function to set the arm position

                        // Reset the flag so this block doesn't run repeatedly
                        isGripperClosed = false;
                    }
                }
                // Detect YELLOW sample
                else if (hue > 40 && hue < 80) {
                    //Gripper = GlobalConstants.gripperClose;
                    closeGripper();
                    gamepad1.rumble(600);
                    telemetry.addData("Detected Color: YELLOW", "Hue: %.2f", hue);
                    isGripperClosed = true;
                    timer.reset();
                    if (isGripperClosed && timer.seconds() > 0.5) {
                        // Move the DepositArm to depositMid position
                        Depositarm = GlobalConstants.depositMid;
                        deposit(Depositarm);  // Call the deposit function to set the arm position

                        // Reset the flag so this block doesn't run repeatedly
                        isGripperClosed = false;
                    }
                }

            }

        }

/////         //////////                  ////////////            //////////           ////////////

        slideLinkage(Slide);
        intakeArm(Arm);
        lift_POS(Lift);
        deposit(Depositarm);
        grip(Gripper);
        // Update telemetry or perform other actions as needed
        telemetry.addData("LIFT LEFT", robot.liftL.getCurrentPosition());
        telemetry.addData("LIFT RIGHT", robot.liftR.getCurrentPosition());
        telemetry.addData("colorSensor", robot.intakeColor.getNormalizedColors());
        telemetry.addData("distance Sensor", robot.armDistance.getDistance(DistanceUnit.INCH));
        telemetry.update();
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    public void holonomic(double Speed, double Turn, double Strafe, double MAX_SPEED) {
//      Left Front = +Speed + Turn - Strafe      Right Front = +Speed - Turn + Strafe
//      Left Rear  = +Speed + Turn + Strafe      Right Rear  = +Speed - Turn - Strafe
        double Magnitude = abs(Speed) + abs(Turn) + abs(Strafe);
        Magnitude = (Magnitude > 1) ? Magnitude : 1; //Set scaling to keep -1,+1 range

        robot.lF.setPower(scale((Speed + Turn - Strafe),
                -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));

        if (robot.lB != null) {
            robot.lB.setPower(scale((Speed + Turn + Strafe),
                    -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        }
        robot.rF.setPower(scale((Speed - Turn + Strafe),
                -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        if (robot.rB != null) {
            robot.rB.setPower(scale((Speed - Turn - Strafe),
                    -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        }
    }
    public void closeGripper() {
        Gripper = GlobalConstants.gripperClose; // Close the gripper
        gamepad1.rumble(600); // Give feedback through the gamepad
        timer.reset();        // Start/restart the timer
        isGripperClosed = true; // Set the flag to indicate that the gripper has closed
    }
    public void intakeArm(double POSITION){
        robot.intakeArmL.setPosition(POSITION);
        robot.intakeArmR.setPosition(0.96-POSITION);
        telemetry.addData("ARM LEFT", robot.intakeArmL.getPosition());
        telemetry.addData("ARM RIGHT", robot.intakeArmR.getPosition());
    }
    public void lift_POS(int POSITION) {
        robot.liftL.setTargetPosition(POSITION);
        robot.liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftL.setPower(1); // Set motor power to move to position
        robot.liftR.setTargetPosition(POSITION);
        robot.liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftR.setPower(1);
    }
    public void resetEncoder(){
        robot.liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void slideLinkage(double POSITION){
        robot.slideL.setPosition(POSITION);
        robot.slideR.setPosition(.90-POSITION);
        telemetry.addData("SLIDE LEFT", robot.slideL.getPosition());
        //telemetry.addData("SLIDE RIGHT", robot.slideR.getPosition());
    }
    public void deposit(double POSITION) {
        robot.Depositarm.setPosition(POSITION);
        telemetry.addData("DEPOSIT", robot.Depositarm.getPosition());
    }
    public void intake(double POWER){
        robot.rightRoller.setPower(-POWER);
        robot.leftRoller.setPower(POWER);
    }
    public void grip(double POSITION){
        robot.gripper.setPosition(POSITION);
        telemetry.addData("GRIPPER", robot.gripper.getPosition());
    }
}
