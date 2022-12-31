package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.annotations.PIDSVA;
import org.firstinspires.ftc.teamcode.lib.control.ControlConstants;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.lib.motion.IMotionProfile;
import org.firstinspires.ftc.teamcode.lib.motion.ResidualVibrationReductionMotionProfilerGenerator;
import org.firstinspires.ftc.teamcode.lib.util.Time;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.DbgLog;
import org.firstinspires.ftc.teamcode.control.StackTracker;
import org.firstinspires.ftc.teamcode.states.FeederConeGripperStateMachine;
import org.firstinspires.ftc.teamcode.states.FeederExtensionStateMachine;
import org.firstinspires.ftc.teamcode.states.FeederStateMachine;


@PIDSVA(name = "Extend",
        P = 0.4d,
        I = 0d,
        D = 0d,
        S = 0.10d,
        V = (1 - 0.10d) / 15d,
        A = 0d
)

@PIDSVA(name = "Retract",
        P = 0.08d,
        I = 0d,
        D = 0d,
        S = 0.06d,
        V = 1 / 2200d,
        A = 0d
)
public class Feeder implements ISubsystem<FeederStateMachine, FeederStateMachine.State> {
    private static final Time CONE_IN_ROBOT_TIME_THRESHOLD = new Time(0.5d, TimeUnits.SECONDS);
    private static final double CONE_IN_ROBOT_DISTANCE_THRESHOLD = 2.5d;
    private static final ControlConstants EXTEND_CONTROL_CONSTANTS;
    private static final ControlConstants RETRACT_CONTROL_CONSTANTS;

    private static FeederStateMachine feederStateMachine;
    private static FeederExtensionStateMachine feederExtensionStateMachine;
    private static FeederConeGripperStateMachine feederConeGripperStateMachine;

    private static StackTracker stackTracker;
    private RevMotor leftExtension;
    private RevServo clawServo;

    private static IMotionProfile extensionProfile = null;
    private static double setpoint = 0d;
    private static double desiredSetpoint = 0d;
    private double lastError;
    private double runningSum;

    static {
        new Thread(ResidualVibrationReductionMotionProfilerGenerator::init).start();
        PIDSVA[] controllers = Feeder.class.getAnnotationsByType(PIDSVA.class);
        if(controllers.length == 2) {
            PIDSVA extendController;
            PIDSVA retractController;
            if(controllers[0].name().equals(FeederExtensionStateMachine.State.EXTEND.getName())) {
                extendController  = controllers[0];
                retractController = controllers[1];
            } else {
                extendController  = controllers[1];
                retractController = controllers[0];
            }

            EXTEND_CONTROL_CONSTANTS = new ControlConstants(
                    extendController.P(), extendController.I(), extendController.D(),
                    extendController.S(), extendController.V(), extendController.A()
            );

            RETRACT_CONTROL_CONSTANTS = new ControlConstants(
                    retractController.P(), retractController.I(), retractController.D(),
                    retractController.S(), retractController.V(), retractController.A()
            );
        } else {
            EXTEND_CONTROL_CONSTANTS  = new ControlConstants();
            RETRACT_CONTROL_CONSTANTS = new ControlConstants();
        }

        FeederExtensionStateMachine.setRunExtension((setpoint) -> {
            if(getExtensionProfile() != null) {
                getExtensionProfile().start();
                Feeder.setpoint = setpoint;
            }
        });
    }

    public Feeder(StackTracker stackTracker, RevMotor leftExtension, RevServo clawServo) {
        setFeederStateMachine(new FeederStateMachine());
        setFeederExtensionStateMachine(new FeederExtensionStateMachine(this));
        setFeederConeGripperStateMachine(new FeederConeGripperStateMachine());
        setStackTracker(stackTracker);
        setLeftExtension(leftExtension);
        setClawServo(clawServo);
        setLastError(0d);
        resetRunningSum();
    }

    public void resetRunningSum() {
        setRunningSum(0d);
    }

    @Override
    public FeederStateMachine getStateMachine() {
        return feederStateMachine;
    }

    @Override
    public FeederStateMachine.State getState() {
        return getStateMachine().getState();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        getLeftExtension().setPower(0d);
        getLeftExtension().resetEncoder();
        setSetpoint(0d, false);
        setDesiredSetpoint(0d);
        //getConeDetector().close();
    }

    @Override
    public String getName() {
        return "Feeder";
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        getStateMachine().update(dt);
        getFeederExtensionStateMachine().update(dt);
        getFeederConeGripperStateMachine().update(dt);

        double error                = getSetpoint() - getLeftExtension().getPosition(); //error in how high the lift goes
        double setpointVelocity     = 0d;
        double setpointAcceleration = 0d;
        if(getExtensionProfile() != null && !getExtensionProfile().isDone()) {
            setpointVelocity     = getExtensionProfile().getVelocity();
            setpointAcceleration = getExtensionProfile().getAcceleration();
        }

        setRunningSum(getRunningSum() + error * dt);
        double output;
        if(getFeederExtensionStateMachine().getState().equals(FeederExtensionStateMachine.State.EXTEND)) {
            output = getExtendControlConstants().getOutput(dt, error, getLastError(), getRunningSum(), setpointVelocity, setpointAcceleration, false);
            output += getExtendControlConstants().kS();

        } else {
            output = getRetractControlConstants().getOutput(dt, error, getLastError(), getRunningSum(), setpointVelocity, setpointAcceleration, true);
        }
        setLastError(error);
        getLeftExtension().setPower(output);
        getClawServo().setPosition(getFeederConeGripperStateMachine().getState().getPosition());
    }

    public void extend() {
        resetRunningSum();
        setSetpoint(getStackTracker().getExtensionHeight(), false);
    }

    public void extendPoles() {
        resetRunningSum();
        setSetpoint(getStackTracker().getPoleExtensionHeight(), true);
    }

    public void retract() {
        resetRunningSum();
        setSetpoint(0d, false);
        setDesiredSetpoint(0d);
    }

    public boolean closeToSetpoint(double threshold) {
        return Math.abs(getSetpoint() - getLeftExtension().getPosition()) <= threshold;
    }

    public static void setFeederStateMachine(FeederStateMachine feederStateMachine) {
        Feeder.feederStateMachine = feederStateMachine;
    }

    public static FeederExtensionStateMachine getFeederExtensionStateMachine() {
        return feederExtensionStateMachine;
    }

    public static void setFeederExtensionStateMachine(FeederExtensionStateMachine feederExtensionStateMachine) {
        Feeder.feederExtensionStateMachine = feederExtensionStateMachine;
    }

    public static FeederConeGripperStateMachine getFeederConeGripperStateMachine(){
        return feederConeGripperStateMachine;
    }

    public static void setFeederConeGripperStateMachine(FeederConeGripperStateMachine feederConeGripperStateMachine){
        Feeder.feederConeGripperStateMachine = feederConeGripperStateMachine;
    }

    public RevMotor getLeftExtension() {
        return leftExtension;
    }

    public void setLeftExtension(RevMotor leftExtension) {
        this.leftExtension = leftExtension;
    }

    public RevServo getClawServo(){ return clawServo; }

    public void setClawServo(RevServo clawServo){ this.clawServo = clawServo; }

    public static double getSetpoint() {
        return setpoint;
    }

    public void setSetpoint(double setpoint, boolean forPoles) {
        setDesiredSetpoint(setpoint);
        if(setpoint != getSetpoint() && (getExtensionProfile() == null || getExtensionProfile().isDone())) {
            if(setpoint != 0d) {
                //Extending
                getFeederExtensionStateMachine().updateState(FeederExtensionStateMachine.State.EXTEND);
                this.setpoint = setpoint;
                String x;
                if (!forPoles){
                    setExtensionProfile(getStackTracker().motionProfilerSetpoints(true));
                    x = String.format("Setpoint: %f, Motion Profile Setpoint: %f", setpoint, getStackTracker().motionProfilerSetpoints(true).getPosition());
                }
                else {
                    setExtensionProfile(getStackTracker().pole_motionProfilerSetpoints(true));
                    x = String.format("Setpoint: %f, Motion Profile Setpoint: %f", setpoint, getStackTracker().pole_motionProfilerSetpoints(true).getPosition());
                }
                DbgLog.msg(x);
            } else {
                //Retracting
                getFeederExtensionStateMachine().updateState(FeederExtensionStateMachine.State.RETRACT);
                setExtensionProfile(new ResidualVibrationReductionMotionProfilerGenerator(
                        getLeftExtension().getPosition(), -getLeftExtension().getPosition(), 25d, 50d
                ));
                this.setpoint = setpoint;
            }
        }
        if(getExtensionProfile() == null){
            DbgLog.msg("EXTENSION PROFILE NULL");
        } //Debugging purposes 12/20
        else {
            DbgLog.msg("EXTENSION PROFILE");
        }
    }

    public static IMotionProfile getExtensionProfile() {
        return extensionProfile;
    }

    public static void setExtensionProfile(IMotionProfile extensionProfile) {
        Feeder.extensionProfile = extensionProfile;
    }

    public static ControlConstants getExtendControlConstants() {
        return EXTEND_CONTROL_CONSTANTS;
    }

    public static ControlConstants getRetractControlConstants() {
        return RETRACT_CONTROL_CONSTANTS;
    }

    public double getLastError() {
        return lastError;
    }

    public void setLastError(double lastError) {
        this.lastError = lastError;
    }

    public double getRunningSum() {
        return runningSum;
    }

    public void setRunningSum(double runningSum) {
        this.runningSum = runningSum;
    }

   public static StackTracker getStackTracker() {
        return stackTracker;
    }

    public static void setStackTracker(StackTracker stackTracker) {
        Feeder.stackTracker = stackTracker;
    }

    public static double getDesiredSetpoint() {
        return desiredSetpoint;
    }

    public static void setDesiredSetpoint(double desiredSetpoint) {
        Feeder.desiredSetpoint = desiredSetpoint;
    }
    public static Time getConeInRobotTimeThreshold() {
        return CONE_IN_ROBOT_TIME_THRESHOLD;
    }

    public static double getConeInRobotDistanceThreshold() {
        return CONE_IN_ROBOT_DISTANCE_THRESHOLD;
    }

}
