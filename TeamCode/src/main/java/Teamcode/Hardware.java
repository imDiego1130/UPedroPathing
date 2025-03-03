package Teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Hardware
{
    /* Public OpMode members. */

    public DcMotor  lF  = null;
    public DcMotor  lB  = null;
    public DcMotor  rF  = null;
    public DcMotor  rB  = null;
    public DcMotor liftL = null;
    public DcMotor liftR = null;
    public DcMotor hangL = null;
    public DcMotor hangR = null;

    public Servo intakeArmL = null;
    public Servo intakeArmR = null;
    public Servo Depositarm = null;
    public Servo slideL = null;
    public Servo slideR = null;
    public CRServo rightRoller = null;
    public CRServo leftRoller = null;
    public Servo gripper = null;

    public IMU imu;
    public ColorRangeSensor intakeColor;
    public DistanceSensor armDistance;

    /* local OpMode members. */
    HardwareMap hwMap =  null;

    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Hardware(){

    }

    /* Initialize standard Techi_Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Techi_Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        lF = hwMap.get(DcMotor.class, "lF");
        lB = hwMap.get(DcMotor.class, "lB");
        rF = hwMap.get(DcMotor.class, "rF");
        rB = hwMap.get(DcMotor.class, "rB");
        liftL = hwMap.get(DcMotor.class,"liftL");
        liftR = hwMap.get(DcMotor.class, "liftR");
        hangL = hwMap.get(DcMotor.class, "hangL");
        hangR = hwMap.get(DcMotor.class, "hangR");

        intakeArmL = hwMap.get(Servo.class, "intakeArmL");
        intakeArmR = hwMap.get(Servo.class, "intakeArmR");
        slideL = hwMap.get(Servo.class, "slideL");
        slideR = hwMap.get(Servo.class, "slideR");
        Depositarm = hwMap.get(Servo.class, "deposit");
        rightRoller= hwMap.get(CRServo.class, "RollerR");
        leftRoller= hwMap.get(CRServo.class, "RollerL");
        gripper =hwMap.get(Servo.class, "gripper");

        intakeColor = hwMap.get(ColorRangeSensor.class, "intakeColor");
        armDistance = hwMap.get(DistanceSensor.class, "distance");
        imu = hwMap.get(IMU.class, "imu");

        lF.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        lB.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rF.setDirection(DcMotor.Direction.FORWARD);
        rB.setDirection(DcMotor.Direction.FORWARD);
        liftL.setDirection(DcMotorSimple.Direction.REVERSE);
        liftR.setDirection(DcMotorSimple.Direction.FORWARD);
        hangL.setDirection(DcMotorSimple.Direction.REVERSE);
        hangR.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set all motors to zero power
        lF.setPower(0);
        lB.setPower(0);
        rF.setPower(0);
        rB.setPower(0);
        liftL.setPower(0);
        liftR.setPower(0);
        hangL.setPower(0);
        hangR.setPower(0);

        // Set all drive train motors to run without encoders.
        lF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
