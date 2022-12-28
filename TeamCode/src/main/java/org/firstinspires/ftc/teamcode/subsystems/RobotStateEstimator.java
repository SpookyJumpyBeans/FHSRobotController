package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Robot.getD1;
import static org.firstinspires.ftc.teamcode.Robot.getD2;
import static org.firstinspires.ftc.teamcode.Robot.getL1;
import static org.firstinspires.ftc.teamcode.Robot.getL2;
import static org.firstinspires.ftc.teamcode.Robot.getWheelRadius;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.PPRobot;
import org.firstinspires.ftc.teamcode.states.IState;

public class RobotStateEstimator implements ISubsystem {
    private PPRobot ppRobot;
    private BNO055IMU imu;
    private volatile Pose2d pose;
    private volatile Pose2d lastPose;
    private Pose2d velocityPose;

    public RobotStateEstimator(PPRobot ppRobot, BNO055IMU imu, Pose2d initialPose) {
        setSkystoneRobot(ppRobot);
        setImu(imu);
        setPose(initialPose);
        setLastPose(initialPose);
        setVelocityPose(new Pose2d());
        initializeIMU();
    }

    @Override
    public IState getStateMachine() {
        return null;
    }

    @Override
    public Enum getState() {
        return null;
    }

    @Override
    public void start() {
        setPose(new Pose2d());
    }

    @Override
    public void update(double dt) {
        double backLeftVelocity = getSkystoneRobot().getDrive().getBackLeft().getVelocity() / getWheelRadius();
        double frontLeftVelocity = getSkystoneRobot().getDrive().getFrontLeft().getVelocity() / getWheelRadius();
        double backRightVelocity = getSkystoneRobot().getDrive().getBackRight().getVelocity() / getWheelRadius();
        double frontRightVelocity = getSkystoneRobot().getDrive().getFrontRight().getVelocity() / getWheelRadius();
        Rotation2d heading = getHeading();

        double cosPsi = heading.cos();
        double sinPsi = heading.sin();
        double denominator = (getD1() + getD2() + getL1() + getL2()) * getWheelRadius();

        //The convention here is that x is forward and y is left
        double velocityX = getWheelRadius() * ((-frontLeftVelocity + frontRightVelocity + backLeftVelocity - backRightVelocity) * sinPsi +
                (frontLeftVelocity + frontRightVelocity + backLeftVelocity + backRightVelocity) * cosPsi) / 4d;
        double velocityY = getWheelRadius() * ((-frontLeftVelocity + frontRightVelocity + backLeftVelocity - backRightVelocity) * cosPsi -
                (frontLeftVelocity + frontRightVelocity + backLeftVelocity + backRightVelocity) * sinPsi) / 4d;
        setVelocityPose(new Pose2d(-velocityY, velocityX, getHeading().rotateBy(getPose().getRotation().inverse()).multiply(1 / dt, false)));
        setLastPose(getPose());
        setPose(new Pose2d(getPose().getTranslation().translateBy(new Translation2d(-velocityY * dt, velocityX * dt)), heading));
        //setPose(getPose().transformBy(new Pose2d(-velocityY * dt, velocityX * dt, new Rotation2d(heading.getRadians() - getPose().getRotation().getRadians(), true))));
    }

    @Override
    public void stop() {

    }

    public SimpleMatrix getDriveState() {
        return new SimpleMatrix(6, 1, false, new double[] {
                getPose().getTranslation().y(),
                getVelocityPose().getTranslation().y(),
                -getPose().getTranslation().x(),
                -getVelocityPose().getTranslation().x(),
                getPose().getRotation().getRadians(),
                getVelocityPose().getRotation().getRadians()
        });
    }

    public void initializeIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile  = "BNO055IMUCalibration.json";
        parameters.loggingEnabled       = true;
        parameters.loggingTag           = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new AccelerationIntegrator();

        getImu().initialize(parameters);
    }

    public Rotation2d getHeading() {
        return new Rotation2d(Math.toRadians(getImu().getAngularOrientation().firstAngle), false);
    }

    public void resetPose() {
        setPose(Pose2d.identity());
        setVelocityPose(Pose2d.identity());
    }

    @Override
    public String toString() {
        return "Position: " + getPose();
    }

    public Pose2d getPose() {
        return pose;
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    public BNO055IMU getImu() {
        return imu;
    }

    public void setImu(BNO055IMU imu) {
        this.imu = imu;
    }

    public Pose2d getVelocityPose() {
        return velocityPose;
    }

    public void setVelocityPose(Pose2d velocityPose) {
        this.velocityPose = velocityPose;
    }

    public PPRobot getSkystoneRobot() {
        return ppRobot;
    }

    public void setSkystoneRobot(PPRobot skystoneRobot) {
        this.ppRobot = skystoneRobot;
    }

    public Pose2d getLastPose() {
        return lastPose;
    }

    public void setLastPose(Pose2d lastPose) {
        this.lastPose = lastPose;
    }

    @Override
    public String getName() {
        return null;
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }
}