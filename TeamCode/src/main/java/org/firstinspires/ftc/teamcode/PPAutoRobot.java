package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.revextension2.ExpansionHubMotor;
import org.firstinspires.ftc.teamcode.revextension2.ExpansionHubServo;
import org.firstinspires.ftc.teamcode.control.StackTracker;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.ExpansionHubs;
import org.firstinspires.ftc.teamcode.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateEstimator;


/**
 * Motor naming convention:
 *     Drivetrain
 *         Front Left Wheel -> LF
 *         Back Left Wheel   -> LR
 *         Front Right Wheel -> RF
 *         Back Right Wheel  -> RR
 *      Elevator
 *         Left Motor -> Elev Left
 *         Right Motor -> Elev Right
 *     Arm
 *         Arm Servo  -> Arm
 *      Claw
 *          Gripper -> Claw
 * Misc. sensors naming convention:

 */
public class PPAutoRobot {
    //private  RevBlinkinLedDriver lights;
    private TimeProfiler matchRuntime;
    private ExpansionHubs expansionHubs;
    private RobotStateEstimator robotStateEstimator;
    private Drive drive;
    private Feeder feeder;
    private StackTracker stackTracker;
    private RevMotor[] motors;
    private RevServo[] servos;


    public void init(HardwareMap hardwareMap) {

//        setExpansionHubs(new ExpansionHubs(this,
//                hardwareMap.get(ExpansionHubEx.class, "Control Hub"),
//                hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2"))
//        );

        setMotors(new RevMotor[] {
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("motorLift")), false, true, false, false, Motor.GOBILDA_312_RPM.getENCODER_TICKS_PER_REVOLUTION(), 0.7402879093)
        });

        setServos(new RevServo[] {
                new RevServo((ExpansionHubServo)(hardwareMap.get("claw")))
        });
        setStackTracker(new StackTracker());
        setFeeder(new Feeder(getStackTracker(),getMotors()[0], getServos()[0]));
        setMatchRuntime(new TimeProfiler(false));
    }
    public RevMotor[] getMotors() {
        return motors;
    }

    public void setMotors(RevMotor[] motors) {
        this.motors = motors;
    }

    public RevServo[] getServos() {
        return servos;
    }

    public void setServos(RevServo[] servos) {
        this.servos = servos;
    }

    public ExpansionHubs getExpansionHubs() {
        return expansionHubs;
    }

    public void setExpansionHubs(ExpansionHubs expansionHubs) {
        this.expansionHubs = expansionHubs;
    }

    public RobotStateEstimator getRobotStateEstimator() {
        return robotStateEstimator;
    }

    public void setRobotStateEstimator(RobotStateEstimator robotStateEstimator) {
        this.robotStateEstimator = robotStateEstimator;
    }

    public Drive getDrive() {
        return drive;
    }

    public void setDrive(Drive drive) {
        this.drive = drive;
    }

    public TimeProfiler getMatchRuntime() {
        return matchRuntime;
    }

    public void setMatchRuntime(TimeProfiler matchRuntime) {
        this.matchRuntime = matchRuntime;
    }

    public Pose2d getRobotPose() {
        return getRobotStateEstimator().getPose();
    }

    public double getRobotSpeed() {
        return getRobotStateEstimator().getVelocityPose().getTranslation().norm() +
                Math.abs(getRobotStateEstimator().getVelocityPose().getRotation().getRadians());
    }
    public Feeder getFeeder() {
        return feeder;
    }


    public void setFeeder(Feeder feeder){
        this.feeder = feeder;
    }

    public StackTracker getStackTracker() {
        return stackTracker;
    }

    public void setStackTracker(StackTracker stackTracker) {
        this.stackTracker = stackTracker;
    }



}