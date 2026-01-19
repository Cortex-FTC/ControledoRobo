package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "NeuroSpark PID Shooter", group = "TeleOp")
public class NEURO extends LinearOpMode {

    // ===== DRIVE =====
    private static final double DEADZONE = 0.05;
    DcMotor fe, fd, te, td;

    // ===== BRAÇO =====
    private static final double BRACO_DESCER = 0.0;
    private static final double BRACO_INICIAL = 0.4;
    private static final long TEMPO_BRACO_MS = 900;

    Servo servoBraco;
    ElapsedTime tempoBraco = new ElapsedTime();
    boolean bracoEmMovimento = false;
    boolean bLast = false;

    // ===== CORE HEX =====
    DcMotor motorCoreHex;
    boolean coreHexLigado = false;
    boolean xLast = false;

    // ===== SHOOTERS =====
    DcMotorEx shooterL, shooterR;

    final double RPM_A = 1480;
    final double RPM_B = 1290;
    final double RPM_X = 1445;
    final double RPM_Y = 1740;

    boolean shooterOn = false;
    double targetRPM = 0;

    // ===== PID =====
    double powerL = 0, powerR = 0;
    double iL = 0, iR = 0;
    double lastErrL = 0, lastErrR = 0;
    long lastTime;

    final double kP = 0.0004;
    final double kI = 0.0000006;
    final double kD = 0.00008;

    final double POWER_MAX = 1.0;
    final double POWER_MIN = 0.0;

    boolean aLast = false, b2Last = false, x2Last = false, yLast = false;

    @Override
    public void runOpMode() {

        // ===== DRIVE =====
        fe = hardwareMap.get(DcMotor.class, "motorFrenteEsquerda");
        fd = hardwareMap.get(DcMotor.class, "motorFrenteDireita");
        te = hardwareMap.get(DcMotor.class, "motorTrasEsquerda");
        td = hardwareMap.get(DcMotor.class, "motorTrasDireita");

        fe.setDirection(DcMotor.Direction.FORWARD);
        te.setDirection(DcMotor.Direction.FORWARD);
        fd.setDirection(DcMotor.Direction.REVERSE);
        td.setDirection(DcMotor.Direction.REVERSE);

        // ===== CORE HEX =====
        motorCoreHex = hardwareMap.get(DcMotor.class, "motorCoreHex");
        motorCoreHex.setDirection(DcMotor.Direction.REVERSE);
        motorCoreHex.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorCoreHex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // ===== BRAÇO =====
        servoBraco = hardwareMap.get(Servo.class, "servoBraco");
        servoBraco.setPosition(BRACO_INICIAL);

        // ===== SHOOTERS =====
        shooterL = hardwareMap.get(DcMotorEx.class, "shooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");

        shooterL.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterR.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("NeuroSpark + Shooter PID PRONTO");
        telemetry.update();

        waitForStart();
        lastTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            driveMecanum();
            controlarBraco();
            controlarCoreHex();
            controlarShooterPID();
            telemetryShooter();
        }
    }

    // ===== DRIVE =====
    private void driveMecanum() {
        double drive = -deadzone(gamepad1.left_stick_y);
        double strafe = -deadzone(gamepad1.left_stick_x);
        double turn = -deadzone(gamepad1.right_stick_x);

        double fl = drive + strafe + turn;
        double fr = drive - strafe - turn;
        double bl = drive - strafe + turn;
        double br = drive + strafe - turn;

        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                        Math.max(Math.abs(fr),
                                Math.max(Math.abs(bl), Math.abs(br)))));

        fe.setPower(fl / max);
        fd.setPower(fr / max);
        te.setPower(bl / max);
        td.setPower(br / max);
    }

    // ===== BRAÇO =====
    private void controlarBraco() {
        boolean bAgora = gamepad1.b;

        if (bAgora && !bLast && !bracoEmMovimento) {
            bracoEmMovimento = true;
            servoBraco.setPosition(BRACO_DESCER);
            tempoBraco.reset();
        }

        if (bracoEmMovimento && tempoBraco.milliseconds() >= TEMPO_BRACO_MS) {
            servoBraco.setPosition(BRACO_INICIAL);
            bracoEmMovimento = false;
        }

        bLast = bAgora;
    }

    // ===== CORE HEX =====
    private void controlarCoreHex() {
        boolean xAgora = gamepad1.x;

        if (xAgora && !xLast) {
            coreHexLigado = !coreHexLigado;
        }

        motorCoreHex.setPower(coreHexLigado ? 1.0 : 0.0);
        xLast = xAgora;
    }

    // ===== SHOOTER PID =====
    private void controlarShooterPID() {

        if (gamepad2.a && !aLast) toggleShooter(RPM_A);
        if (gamepad2.b && !b2Last) toggleShooter(RPM_B);
        if (gamepad2.x && !x2Last) toggleShooter(RPM_X);
        if (gamepad2.y && !yLast) toggleShooter(RPM_Y);

        aLast = gamepad2.a;
        b2Last = gamepad2.b;
        x2Last = gamepad2.x;
        yLast = gamepad2.y;

        if (shooterOn) {
            pidShooter();
            shooterL.setPower(powerL);
            shooterR.setPower(powerR);
        } else {
            shooterL.setPower(0);
            shooterR.setPower(0);
        }
    }

    private void pidShooter() {
        long now = System.currentTimeMillis();
        double dt = (now - lastTime) / 1000.0;
        if (dt <= 0) dt = 0.01;
        lastTime = now;

        double errL = targetRPM - Math.abs(shooterL.getVelocity());
        double errR = targetRPM - Math.abs(shooterR.getVelocity());

        iL += errL * dt;
        iR += errR * dt;

        double dL = (errL - lastErrL) / dt;
        double dR = (errR - lastErrR) / dt;

        lastErrL = errL;
        lastErrR = errR;

        powerL = clip(powerL + kP * errL + kI * iL + kD * dL);
        powerR = clip(powerR + kP * errR + kI * iR + kD * dR);
    }

    private double clip(double v) {
        return Math.max(POWER_MIN, Math.min(POWER_MAX, v));
    }

    private void toggleShooter(double rpm) {
        shooterOn = !(shooterOn && targetRPM == rpm);
        targetRPM = shooterOn ? rpm : 0;

        powerL = powerR = 0.35;
        iL = iR = 0;
        lastErrL = lastErrR = 0;
        lastTime = System.currentTimeMillis();
    }

    private void telemetryShooter() {
        telemetry.addData("Shooter ON", shooterOn);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("RPM L", shooterL.getVelocity());
        telemetry.addData("RPM R", shooterR.getVelocity());
        telemetry.addData("Power L", powerL);
        telemetry.addData("Power R", powerR);
        telemetry.update();
    }

    private double deadzone(double v) {
        return Math.abs(v) < DEADZONE ? 0.0 : v;
    }
}
