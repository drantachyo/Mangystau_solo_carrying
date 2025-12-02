package org.firstinspires.ftc.teamcode.MyAutos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@Autonomous(name = "RBCC_Mangystau", group = "Auto")
public class RedBaseCloseCorner extends LinearOpMode {

    // PIDF настройки
    double kP = 33.0, kI = 0, kD = 2.0, kF = 14.0;

    // Целевая скорость
    double targetRPM = 2850;
    double holdRPM = 1800;
    double targetTPS, holdTPS;

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(61, 14, Math.toRadians(-180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        // Настройка PID шутера
        DcMotorEx shooterMotor = shooter.getMotor();
        shooterMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF));

        targetTPS = targetRPM / 60.0 * 28.0;
        holdTPS = holdRPM / 60.0 * 28.0;

        waitForStart();

        // Раскрутка шутера на полную скорость на старте
        shooterMotor.setVelocity(targetTPS);

        // Стартовый короткий выезд intake
        intake.setPower(1.0);
        sleep(300);
        intake.setPower(0.0);

        if (isStopRequested()) return;

        boolean isBlue = false; // red side
        double side = isBlue ? 1 : -1;
        double shootX = -27.5;
        double shootY = -27.5;
        double shootH = -133;

        // === 1. Первый выезд к обелиску ===
        Action toObelisk1 = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(shootX, shootY * side), Math.toRadians(shootH) * side)
                .build();
        Actions.runBlocking(toObelisk1);
        fireSequencePID(shooter, intake);
        shooterMotor.setVelocity(holdTPS);

        // === 2. Первый забег за шарами ===
        intake.setPower(1.0);
        Action toBalls1 = drive.actionBuilder(new Pose2d(shootX, shootY * side, Math.toRadians(shootH) * side))
                .strafeToLinearHeading(new Vector2d(-12, -30 * side), Math.toRadians(-90) * side)
                .strafeToLinearHeading(new Vector2d(-12, -52 * side), Math.toRadians(-90) * side)
                .build();
        Actions.runBlocking(toBalls1);
        intake.setPower(0.0);
        shooterMotor.setVelocity(targetTPS);

        // === 3. Второй выезд к обелиску ===
        Action toObelisk2 = drive.actionBuilder(new Pose2d(-12, -52 * side, Math.toRadians(-90) * side))
                .strafeToLinearHeading(new Vector2d(shootX, shootY * side), Math.toRadians(shootH) * side)
                .build();
        Actions.runBlocking(toObelisk2);
        fireSequencePID(shooter, intake);
        shooterMotor.setVelocity(holdTPS);

        // === 4. Второй забег ===
        intake.setPower(1.0);
        Action toBalls2 = drive.actionBuilder(new Pose2d(shootX, shootY * side, Math.toRadians(shootH) * side))
                .strafeToLinearHeading(new Vector2d(11.6, -30 * side), Math.toRadians(-90) * side)
                .strafeToLinearHeading(new Vector2d(11.6, -60 * side), Math.toRadians(-90) * side)
                .strafeToConstantHeading(new Vector2d(11.6, -40 * side))
                .build();
        Actions.runBlocking(toBalls2);
        intake.setPower(0.0);
        shooterMotor.setVelocity(targetTPS);

        // === 5. Третий выезд к обелиску ===
        Action toObelisk3 = drive.actionBuilder(new Pose2d(11.6, -40 * side, Math.toRadians(-90) * side))
                .strafeToLinearHeading(new Vector2d(shootX, shootY * side), Math.toRadians(shootH) * side)
                .build();
        Actions.runBlocking(toObelisk3);
        fireSequencePID(shooter, intake);
        shooterMotor.setVelocity(holdTPS);

        // === 6. Третий забег ===
        intake.setPower(1.0);
        Action toBalls3 = drive.actionBuilder(new Pose2d(shootX, shootY * side, Math.toRadians(shootH) * side))
                .strafeToLinearHeading(new Vector2d(34.5, -30 * side), Math.toRadians(-90) * side)
                .strafeToLinearHeading(new Vector2d(34.5, -60 * side), Math.toRadians(-90) * side)
                .build();
        Actions.runBlocking(toBalls3);
        intake.setPower(0.0);
        shooterMotor.setVelocity(targetTPS);

        // === 7. Четвёртый выезд ===
        Action toObelisk4 = drive.actionBuilder(new Pose2d(34.5, -60 * side, Math.toRadians(-90) * side))
                .strafeToLinearHeading(new Vector2d(shootX, shootY * side), Math.toRadians(shootH) * side)
                .build();
        Actions.runBlocking(toObelisk4);
        fireSequencePID(shooter, intake);

        // === Выезд из зоны (если нужно) ===
        Action outOfZone = drive.actionBuilder(new Pose2d(shootX, shootY * side, Math.toRadians(shootH) * side))
                .strafeToLinearHeading(new Vector2d(shootX + 15, shootY + 5 * side), Math.toRadians(shootH) * side)
                .build();
        Actions.runBlocking(outOfZone);
    }

    // 🔥 PID Fire Sequence
    private void fireSequencePID(Shooter shooter, Intake intake) throws InterruptedException {
        shooter.openGate();
        Thread.sleep(200);
        intake.setPower(1.0);
        Thread.sleep(1000);
        intake.setPower(0.0);
        shooter.closeGate();
        Thread.sleep(500);
    }
}
