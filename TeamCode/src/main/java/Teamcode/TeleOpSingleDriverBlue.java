package Teamcode;

import static com.qualcomm.robotcore.util.Range.scale;
import static Teamcode.GlobalConstants.hangDown;
import static Teamcode.GlobalConstants.intakeArmMid;
import static Teamcode.GlobalConstants.intakeDistance;
import static Teamcode.GlobalConstants.liftStart;
import static java.lang.Math.abs;

import android.annotation.SuppressLint;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp SINGLE Driver BLUE", group="TeleOp")
public class TeleOpSingleDriverBlue extends OpMode {
    /* Declare OpMode members. */
    Hardware robot = new Hardware();
    double MAX_SPEED = .7;
    double Arm = 0;
    int Lift = 0;
    double Slide = 0;
    double Depositarm = 0;
    double Gripper = 0;
    int Hang = 0;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime loopTimer = new ElapsedTime();

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

        robot.hangL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hangR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Slide = GlobalConstants.slideBack;
        Arm = GlobalConstants.intakeArmUp;
        Depositarm = GlobalConstants.depositIn;
        Lift = liftStart;
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
        loopTimer.reset();

        if (gamepad1.left_stick_button) {
            MAX_SPEED = .3;
        }
        if (gamepad1.right_stick_button) {
            MAX_SPEED = .66;
        }
        double Speed = -gamepad1.left_stick_y;
        double Turn = gamepad1.right_stick_x;
        double Strafe = -gamepad1.left_stick_x;
        holonomic(Speed, Turn, Strafe, MAX_SPEED);

        //INTAKE CODE BEGINS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        if (Lift >= 415){
            MAX_SPEED = .32;
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
        if (gamepad1.left_trigger > 0){
            //Slide = GlobalConstants.slideMid;
            Arm = GlobalConstants.intakeArmDown;
        }
        //LIFT CODE BEGINS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        if (gamepad1.y){
            Lift = GlobalConstants.sampleHIGH;
            Arm = GlobalConstants.intakeArmMid;
            Depositarm = GlobalConstants.depositTeleOp;
        }
        if (gamepad1.b){
            Lift = GlobalConstants.specimenHIGH;
            Depositarm = GlobalConstants.depositMid;
        }
        if (gamepad1.x){
            Arm = GlobalConstants.intakeArmMid;
            Lift = GlobalConstants.sampleLOW;
            Depositarm = GlobalConstants.depositMid;
        }
        if (gamepad1.a){
            Lift = liftStart;
            Arm = GlobalConstants.intakeArmMid;
            Depositarm = GlobalConstants.depositIn;
            Gripper = GlobalConstants.gripperOpen;
            Slide = GlobalConstants.slideBack;
        }
        if (gamepad1.left_bumper){
            Gripper = GlobalConstants.gripperOpen;
        }
        if (gamepad1.right_bumper){
            if (Arm == GlobalConstants.intakeArmUp){
                Gripper = GlobalConstants.gripperClose;
            }else if ((Arm > GlobalConstants.intakeArmUp) && (Slide == GlobalConstants.slideBack)){
                Slide = GlobalConstants.slideMid;
            }else if (Slide == GlobalConstants.slideMid) {
                Slide = GlobalConstants.slideOut;
            }
        }
        if (gamepad1.dpad_up){ // HANGINGG// was 600, 1100, 600, 300, 100, 3300
            Hang = GlobalConstants.hangUp;
        }
        if (gamepad1.dpad_down){
            Hang = hangDown;
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
        if (Arm > GlobalConstants.intakeArmMid) {
            colorDetect();
        }
        double distance = robot.armDistance.getDistance(DistanceUnit.INCH);
        if (distance <= intakeDistance) {    // was 0.37
           if (colorDetect() == "Blue") {
               Gripper = GlobalConstants.gripperClose;
               gamepad1.rumble(600);
           }else if(colorDetect() == "Yellow"){
               Gripper = GlobalConstants.gripperClose;
               gamepad1.rumble(600);
           }
        }

/////         //////////                  ////////////            //////////           ////////////

        slideLinkage(Slide);
        intakeArm(Arm);
        lift_POS(Lift);
        deposit(Depositarm);
        grip(Gripper);
        hang_POS(Hang);
        // Update telemetry or perform other actions as needed
        telemetry.addData("MAX Speed", "%.2f", MAX_SPEED);
        telemetry.addData("LIFT LEFT", robot.liftL.getCurrentPosition());
        telemetry.addData("LIFT RIGHT", robot.liftR.getCurrentPosition());
        telemetry.addData("colorSensor", robot.intakeColor.getNormalizedColors());
        telemetry.addData("distance Sensor", robot.armDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("ARM LEFT", robot.intakeArmL.getPosition());
        telemetry.addData("ARM RIGHT", robot.intakeArmR.getPosition());
        telemetry.addData("Loop Time (ms)", "%.2f", loopTimer.milliseconds());
        telemetry.addData("SLIDE LEFT", robot.slideL.getPosition());
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

    public void intakeArm(double POSITION){
        robot.intakeArmL.setPosition(POSITION);
        robot.intakeArmR.setPosition(0.98-POSITION);
    }
    public void lift_POS(int POSITION) {
        // Get the current position of the lift motor
        int currentPositionL = robot.liftL.getCurrentPosition();
        int currentPositionR = robot.liftR.getCurrentPosition();

        // Check if the lift is coming down to liftStart
        double power = (POSITION < currentPositionL && POSITION == liftStart) ? 0.6 : 1.0;

        // Set target position and power for the left lift motor
        robot.liftL.setTargetPosition(POSITION);
        robot.liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftL.setPower(power);

        // Set target position and power for the right lift motor
        robot.liftR.setTargetPosition(POSITION);
        robot.liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftR.setPower(power);
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
    }
    public void deposit(double POSITION) {
        robot.Depositarm.setPosition(POSITION);
        //telemetry.addData("DEPOSIT", robot.Depositarm.getPosition());
    }
    public void intake(double POWER){
        robot.rightRoller.setPower(-POWER);
        robot.leftRoller.setPower(POWER);
    }
    public void grip(double POSITION){
        robot.gripper.setPosition(POSITION);
       // telemetry.addData("GRIPPER", robot.gripper.getPosition());
    }
    public void hang_POS(int POSITION){
        robot.hangL.setTargetPosition(POSITION);
        robot.hangR.setTargetPosition(POSITION);
        robot.hangL.setPower(1);
        robot.hangR.setPower(1);
        robot.hangL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.hangR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //telemetry.addData("HANG", robot.hangL.getCurrentPosition());
        telemetry.update();
    }
    // Function to check the color detected by the color sensor
    public String colorDetect() {
        float[] hsvValues = new float[3];  // Store HSV values from the sensor
        Color.RGBToHSV((int) (robot.intakeColor.red() * 255),
                (int) (robot.intakeColor.green() * 255),
                (int) (robot.intakeColor.blue() * 255),
                hsvValues);
        float hue = hsvValues[0]; // Get the hue value from HSV

        // Compare the HSV values to thresholds and return the color name
        if (hue < 30 || hue > 330) {  // Hue close to red
            Arm = intakeArmMid;
            intake(1);
            return "Red";
        } else if (hue > 40 && hue < 80) {  // Hue close to yellow
            Arm = GlobalConstants.intakeArmUp;
            Slide = GlobalConstants.slideBack;
            return "Yellow";
        } else if (hue > 200 && hue < 260) {  // Hue close to blue
            Arm = GlobalConstants.intakeArmUp;
            Slide = GlobalConstants.slideBack;
            return "Blue";
        } else {
            return "Unknown";
        }
    }
}
