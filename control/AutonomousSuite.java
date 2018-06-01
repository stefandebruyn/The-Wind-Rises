package org.firstinspires.ftc.teamcode.vv7797.opmode.control;

/**
 * OVERVIEW, DESIGN PROCESS, AND PERFORMANCE
 *
 * The primary driver class for our Relic Recovery autonomous. Given a designated plate within the arena and a hardware map, the suite
 * assumes complete control of the robot and directs all of its actions throughout the phase.
 *
 * Throughout the design process of this class, an emphasis was placed on speed. Many of the instruction implementations allow for
 * additional instructions to be run simultaneously with them. An overarching concurrency schedule allows specific code to be
 * executed at very specific points in time, such as to retract the jewel arm halfway through the plate dismount procedure (if it was
 * retracted immediately, it would not hit the jewel, and if it was retracted afterwards, it would first clip the cryptobox).
 *
 * Cryptobox alignment is performed with a long arm inserted into the cryptobox. One sensor measures distance from the plastic divider,
 * and another measures distance from the wall. The arm was printed specifically long so that alignment could be performed at a distance.
 * When lining up, the robot purposely undershoots (or overshoots, if it's on the blue side) its target column to ensure that the arm
 * sensor (referred to as the "latch" in the code) catches the divider.
 *
 * Unfortunately, the time we had to prepare for UIL did not allow for the development of a back (far) plate multi-glyph. A basic 85 was
 * written, and nothing more. We instead decided to focus on a front (close) plate multi-glyph. Using range sensors inlaid in the glyph bay,
 * glyph gathering was made persistent; the robot will push itself into the glyph pit, and it'll keep pushing until it gets something.
 * If this fails, the robot will back out and park before time runs out. All of this together allows for highly consistent autonomous
 * performance--85 points are guaranteed, and 100-115 points are achieved the vast majority of the time (the auto is programmed to
 * be satisfied with just one additional glyph--however, if RNG is on our side, a second can enter the intake before the robot realizes
 * it has the first).
 *
 * PROBLEMS
 *
 * There is a single hard-coded movement in the entire auto (all other movements are guided by sensors and mapping): a 4-inch left strafe
 * adjustment the robot makes before realigning for a multi-glyph delivery (to ensure that the latch arm catches its target divider).
 * Originally, we planned to use computer vision to realign the robot with the cryptobox after returning from the glyph pit (hence the
 * phone's ability to swivel). Again, time did not permit. Instead, it was assumed that the robot would not become significantly displaced
 * parallel to its cryptobox during glyph gathering. If it were to become significantly displaced, whether by collision with another robot or
 * something else, it could miss its target column, miss the cryptobox entirely, or ram the alignment arm into a plastic divider (code was
 * written to predict and prevent this, but disabled in competition for lack of sufficient testing). Luckily, these things happened
 * extremely infrequently in practice and never in competition. The chance was minuscule, but it was there nonetheless.
 *
 * Driving was perfected through two PIDs: one which corrects heading, and another which corrects velocity. These were perfectly tuned so
 * as to never overpower one another. However, despite our best efforts, there are rare oddities that can cause this to happen. If the robot is
 * somehow lifted up and dropped again, the gyro PID can be tricked into making massive power corrections during the period of time in which
 * the robot loses contact with the ground. The robot will charge full speed ahead across the arena, the velocity PID being unable to
 * reciprocate fast enough. This was observed once in practice when the robot backed up into a misplaced glyph during cryptobox realignment.
 * A corner of the glyph caught under the jewel collector, and lifted the back of the robot clean off the floor. Time did not permit for an
 * adequate failsafe to be implemented and tested. Fortunately, this malfunction is caused by something so unusual that it was safe to
 * assume that it would never happen in competition (which it didn't), and instead devote time to more pressing features.
 *
 * HINDSIGHT
 *
 * This code is by no means exemplar. If I had anticipated how large this class would become, I would have organized instructions into their
 * own classes instead of methods.
 *
 * Using computer vision to guide parallel cryptobox realignment and then transferring into alignment via arm would have been faster, more
 * reliable, and eliminated the previously described risk of breaking the arm on the plastic divider.
 *
 * Had we planned for dead wheels in the drivetrain design, I would have liked to use Kalman filtering and spline generation for the robot's
 * pathfinding. Kalman filtering had been implemented successfully on a previous robot, but because of the troubleshooting it required and
 * the fact that this robot was guided primarily with sensors, I decided against it. The ease with which a robot can accurately localize
 * using dead wheels would remove the need for a sensor arm, thereby greatly increasing alignment speed. The robot could also move more
 * atypically when navigating the glyph pit, heightening multi-glyph potential.
 *
 *
 * Stefan deBruyn, 2017/18 7797 Software Lead
 * stefanriverdebruyn@gmail.com
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import org.firstinspires.ftc.teamcode.vv7797.opmode.navigation.ArenaMap;
import org.firstinspires.ftc.teamcode.vv7797.opmode.navigation.KalmanFilter;
import org.firstinspires.ftc.teamcode.vv7797.opmode.navigation.Point;
import org.firstinspires.ftc.teamcode.vv7797.opmode.util.CryptoboxMap;
import org.firstinspires.ftc.teamcode.vv7797.opmode.util.Matrix;
import org.firstinspires.ftc.teamcode.vv7797.opmode.util.MatrixBuilder;
import org.firstinspires.ftc.teamcode.vv7797.opmode.util.OpModeUtil.GlyphColor;
import org.firstinspires.ftc.teamcode.vv7797.opmode.util.OpModeUtil.Plate;
import org.firstinspires.ftc.teamcode.vv7797.opmode.util.OpModeUtil.TeamColor;
import org.firstinspires.ftc.teamcode.vv7797.opmode.util.OpModeUtil;
import org.firstinspires.ftc.teamcode.vv7797.opmode.util.exception.HeartbeatException;
import org.firstinspires.ftc.teamcode.vv7797.opmode.util.exception.AbortException;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.CryptoboxDetector;

import java.util.ArrayList;
import java.util.Locale;

public final class AutonomousSuite {
    private enum Instruction { DRIVE, TURN, NONE }
    private enum ArmConfig { UPRIGHT, CONSTRAINED, UPRIGHT_FROM_DETECT, DETECT_JEWEL, DETECT_CRYPTOBOX }
    private enum ScoreMethod { DUNK, DUMP, EJECT }
    private enum AlignMethod { JEWEL, LATCH }
    private enum DismountMethod { SENSOR, HARD, FAST }
    private enum FishingStatus { SUCCESS, OUT_OF_TIME, NONE }

    public interface NestedInstruction {
        String run() throws HeartbeatException, AbortException;
    }

    public static class ConcurrentInstruction {
        public final NestedInstruction NESTED;
        public final double TIME;

        public ConcurrentInstruction(NestedInstruction nested, double time) {
            NESTED = nested;
            TIME = time;
        }
    }

    private LinearOpMode client;
    private TeamColor teamColor;
    private Plate plate;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private ElapsedTime runtime;

    private DcMotorEx drvFrontLeft, drvFrontRight, drvBackLeft, drvBackRight, drvIntakeLeft, drvIntakeRight;
    private DcMotorEx[] drivetrain;
    private Servo srvRamp, srvPhone, srvArmSwivel, srvArmShoulder, srvLatch;
    private BNO055IMU gyro;
    private ColorSensor snsColorRampBottom, snsColorRampTop, snsColorJewel;
    private DistanceSensor snsRangeJewel, snsRangeRampBottom, snsRangeRampTop, snsRangeLatchWall, snsRangeLatchDivider;
    private ModernRoboticsI2cRangeSensor snsRangeBack, snsRangeFront;

    private VuforiaTrackable relicTemplate;
    private VuforiaTrackables relicTrackables;
    private VuforiaLocalizer vuforia;
    private RelicRecoveryVuMark targetColumn = RelicRecoveryVuMark.UNKNOWN;
    private GlyphColor[] payload = { GlyphColor.NONE, GlyphColor.NONE };
    private CryptoboxDetector cryptoboxDetector;
    private ArenaMap arenaMap;
    private Point origin;
    private MatrixBuilder matrixBuilder;
    private Matrix robotState;
    private Orientation orientation;
    private Instruction currentInstruction = Instruction.NONE;
    private ArrayList<ConcurrentInstruction> concurrentInstructions = new ArrayList<>();
    private KalmanFilter kalman;
    private CryptoboxMap cryptoboxMap = new CryptoboxMap();
    private PIDController pidContGyroStrafe, pidContGyroDrive, pidContVelocityStrafe, pidContVelocityDrive, pidContTurn;
    private PIDController[] pids;
    private ScoreMethod scoreMethod = ScoreMethod.DUNK;
    private AlignMethod alignMethod = AlignMethod.LATCH;
    private DismountMethod dismountMethod = DismountMethod.FAST;
    private FishingStatus fishingStatus = FishingStatus.NONE;

    private double currentFieldHeading;
    private double gyroFieldHeadingDifference;
    private double lastPidTime = -1;
    private double currentAbsoluteDriveHeading;
    private double currentRelativeDriveHeading;
    private double pidTargetHeading;
    private double pidTargetVelocity;
    private boolean pidGyroEnabled = false;
    private boolean pidVelocityEnabled = true;
    private boolean preemptivePidDriveBreaking = true;
    private boolean driveConclusionOrientationCorrection = false;

    private int payloadConfirmations = 0;
    private int payloadConfirmationsMax = 3;

    private final double DRIVETRAIN_GEAR_RATIO = 1.5;
    private final double DRIVETRAIN_WHEEL_DIAMETER = 4.0;
    private final double DRIVETRAIN_TICKS_PER_REV = 1120;
    private final double LINEAR_TO_TICKS = DRIVETRAIN_TICKS_PER_REV / (Math.PI * DRIVETRAIN_GEAR_RATIO * DRIVETRAIN_WHEEL_DIAMETER);
    private final double TICKS_TO_LINEAR = 1 / LINEAR_TO_TICKS;
    private final double POSITION_EQU_THRESHOLD = 0.25;
    private final double ORIENTATION_EQU_THRESHOLD = 2;
    private final double SECONDARY_ORIENTATION_EQU_THRESHOLD = 1;
    private final double IDEAL_INTAKE_POWER = 0.75;
    private final double DEAD_RECKON_INTERVAL = 0.05;
    private final double PID_INTERVAL = 0.01;
    private final double DRIVE_TO_STRAFE_RATIO = 1.247975;
    private final double BACK_SENSOR_IMPRESSION = 4.5;
    private final double DRIVETRAIN_VELOCITY_ZERO_THRESHOLD = 0.25;
    private final double MULTIGLYPHS_BEFORE_ABORT = 1;

    public final double PHASE_LENGTH = 30;
    public final double FISHING_ABORT_THRESHOLD = 24;
    public final double MINIMUM_ALIGNMENT_TIME = 5;

    private final double PHONE_POSITION_PICTOGRAM = 0.65;
    private final double PHONE_POSITION_CRYPTOBOX = 0.5;
    private final double PHONE_POSITION_DODGE = 1;
    private final double SWIVEL_POSITION_CRYPTOBOX = 0;
    private final double SWIVEL_POSITION_LEFT = 0.425;
    private final double SWIVEL_POSITION_RIGHT = 0.575;
    private final double SWIVEL_POSITION_CENTER = 0.5;
    private final double ARM_POSITION_CRYPTOBOX = 0.6;
    private final double ARM_POSITION_UP = 1;
    private final double ARM_POSITION_DOWN = 0.495;
    private final double ARM_POSITION_CONSTRAINED = 0.875;
    private final double LATCH_POSITION_TUCKED = 0;
    private final double LATCH_POSITION_EXTENDED = 1;
    private final double RAMP_POSITION_DOWN = 0;
    private final double RAMP_POSITION_FLAT = 0.33;
    private final double RAMP_POSITION_UP = 1;

    private final double VEL_NORM_DRIVE = 5;
    private final double VEL_FAST_DRIVE = 7;
    private final double VEL_GONE_FISHING = 10;
    private final double VEL_CREEP_DRIVE = 2;
    private final double VEL_SCAN_STRAFE = 2;
    private final double VEL_CORRECTION = 1;
    private final double VEL_DISMOUNT = 6;
    private final double VEL_CAREFUL_DISMOUNT = 3.5;
    private final double VEL_FISH = 7;
    private final double VEL_NORM_STRAFE = 4;

    private final boolean ATTEMPT_FRONT_MULTIGLYPH = true;
    private final boolean ATTEMPT_BACK_MULTIGLYPH = false;
    private final boolean BLUE_STEAL_JEWEL = true;
    private final boolean RED_STEAL_JEWEL = true;
    private final boolean ATTEMPT_FAILED_SCORE_RECOVERY = false;
    private final boolean SCORE_MULTIGLYPH_IN_UNIQUE_COLUMN = false;
    private final boolean RAMP_SENSORS_ABORT_MULTIGLYPH = true;
    private final boolean SCATTER_GLYPHS_ON_PILE_ENTRY = false;
    private final boolean SUSPEND_HALT_WHILE_NONZERO_VELOCITY = true;
    private final boolean PREDICT_AND_CORRECT_DIVIDER_COLLISIONS = false;



    /**
     * @param client Calling opmode
     * @param teamColor Robot team color
     * @param plate Robot balance stone
     * @param hardwareMap Client hardware map
     * @param telemetry Client telemetry pipeline
     */
    public AutonomousSuite(final LinearOpMode client, final TeamColor teamColor, final Plate plate, final HardwareMap hardwareMap, final Telemetry telemetry) {
        this.client = client;
        this.teamColor = teamColor;
        this.plate = plate;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.runtime = new ElapsedTime();

        initialize();
        prereadPictogram();
    }



    /**
     * Configure hardware and initialize control systems
     */
    private void initialize() {
        telemetry.setCaptionValueSeparator(" | ");
        telemetry.setMsTransmissionInterval(100);
        telemetry.setAutoClear(false);



        // Vuforia setup
        telemetry("Initialization", "Warming up Vuforia...");

        final int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        vuforiaParams.vuforiaLicenseKey = "AQvrVpP/////AAAAGauRujoAs0gLpZI/MZh9rk1y2u54G7gQhPA5nvoC4Qit6yi5xypatqoAZ2Gdu3xd/2h+b9LshTtiv4yz9Acm7xgXseQR/GF/wAB3SBfEGXSCE5QFkSft5kizzRw7I5S89pULXjxHFkZFSn/Yb5Dlb/IdG7IhIF9FqGw93TLFalkw3BDjY1dsZ51+nPcXKfDWYBX10UzLyDrjlUiChvtbMT67kCDFWsUUsUxaSbexM6rbK/eBR+nFRcEh5KMs7PHNCAgssnwdpkxYa2s5XqEvXIXAZiIiGJO9xKrHSYSrSrMvoTx3b7KQsSbwNdsyJMfEuwficrD2fDbSEGGcqGmScHRCnLLbmXNFzNENNm9hObsU";
        vuforiaParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.createVuforiaLocalizer(vuforiaParams);
        relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);

        relicTrackables.activate();



        // Hardware setup
        telemetry("Initialization", "Mapping hardware...");

        drvFrontLeft = (DcMotorEx)hardwareMap.dcMotor.get("drvFrontLeft");
        drvFrontRight = (DcMotorEx)hardwareMap.dcMotor.get("drvFrontRight");
        drvBackLeft = (DcMotorEx)hardwareMap.dcMotor.get("drvBackLeft");
        drvBackRight = (DcMotorEx)hardwareMap.dcMotor.get("drvBackRight");
        drvIntakeLeft = (DcMotorEx)hardwareMap.dcMotor.get("drvIntakeLeft");
        drvIntakeRight = (DcMotorEx)hardwareMap.dcMotor.get("drvIntakeRight");

        drvFrontRight.setDirection(DcMotor.Direction.REVERSE);
        drvBackRight.setDirection(DcMotor.Direction.REVERSE);
        drvIntakeRight.setDirection(DcMotor.Direction.REVERSE);

        drivetrain = new DcMotorEx[] { drvFrontLeft, drvFrontRight, drvBackLeft, drvBackRight };

        for (DcMotorEx motor : drivetrain)
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        srvLatch = hardwareMap.servo.get("srvLatch");
        srvRamp = hardwareMap.servo.get("srvRamp");
        srvPhone = hardwareMap.servo.get("srvPhone");
        srvArmSwivel = hardwareMap.servo.get("srvArmSwivel");
        srvArmShoulder = hardwareMap.servo.get("srvArmShoulder");

        srvRamp.setDirection(Servo.Direction.REVERSE);

        snsColorJewel = hardwareMap.get(ColorSensor.class, "snsColorJewel");
        snsColorRampBottom = hardwareMap.colorSensor.get("snsColorRampBottom");
        snsColorRampTop = hardwareMap.colorSensor.get("snsColorRampTop");

        snsRangeRampBottom = hardwareMap.get(DistanceSensor.class, "snsColorRampBottom");
        snsRangeRampTop = hardwareMap.get(DistanceSensor.class, "snsColorRampTop");
        snsRangeBack = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "snsRangeBack");
        snsRangeFront = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "snsRangeFront");
        snsRangeLatchWall = hardwareMap.get(DistanceSensor.class, "snsRangeLatchWall");
        snsRangeLatchDivider = hardwareMap.get(DistanceSensor.class, "snsRangeLatchDivider");



        // Gyro setup
        telemetry("Initialization", "Contacting gyro...");

        BNO055IMU.Parameters gyroParams = new BNO055IMU.Parameters();
        gyroParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyroParams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyroParams.calibrationDataFile = "BNO055IMUCalibration.json";
        gyroParams.loggingEnabled = true;
        gyroParams.loggingTag = "IMU";
        gyroParams.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(gyroParams);

        updateOrientation();



        // DogeCV setup
        telemetry("Initialization", "Taming the doge...");

        cryptoboxDetector = new CryptoboxDetector();
        cryptoboxDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        cryptoboxDetector.downScaleFactor = 0.5;
        cryptoboxDetector.detectionMode = CryptoboxDetector.CryptoboxDetectionMode.RED;
        cryptoboxDetector.speed = CryptoboxDetector.CryptoboxSpeed.SLOW;
        cryptoboxDetector.rotateMat = true;



        // Navigation setup
        telemetry("Initialization", "Orienting robot...");

        arenaMap = new ArenaMap(plate, teamColor);
        origin = arenaMap.getStartingPosition();
        matrixBuilder = new MatrixBuilder(false);
        robotState = matrixBuilder.build(4, 1, 0, 0, 0, 0);

        robotState.set(0, 0, origin.X);
        robotState.set(1, 0, origin.Y);

        currentFieldHeading = (teamColor == TeamColor.RED ? 270 : 90);
        gyroFieldHeadingDifference = -currentFieldHeading;

        pidContGyroStrafe = new PIDController(0.005, 0, 0.0025, runtime);
        pidContGyroDrive = new PIDController(0.005, 0, 0.0025, runtime);
        pidContVelocityDrive = new PIDController(0.02, 0, 0, runtime);
        pidContVelocityStrafe = new PIDController(0.045, 0, 0, runtime);
        pidContTurn = new PIDController(0.00155, 0, 0.0012, runtime);

        pids = new PIDController[] { pidContGyroStrafe, pidContGyroDrive, pidContVelocityStrafe, pidContVelocityDrive, pidContTurn };



        // Kalman filter setup
        telemetry("Initialization", "Constructing Kalman filter...");

        kalman = new KalmanFilter(DEAD_RECKON_INTERVAL, true, KalmanFilter.StateDimensions.S2D, matrixBuilder);

        kalman.setCurrentState(robotState);
        kalman.setPreviousState(robotState);
        kalman.setObservationErrors(0, 0, 0, 0);
        kalman.setProcessErrors(0, 0, 0, 0);
        kalman.setStateUpdates(0, 0);



        // Lock servos in place
        srvRamp.setPosition(RAMP_POSITION_DOWN);
        srvArmSwivel.setPosition(SWIVEL_POSITION_CRYPTOBOX);
        srvArmShoulder.setPosition(ARM_POSITION_CONSTRAINED);
        srvLatch.setPosition(LATCH_POSITION_TUCKED);



        telemetry("Initialization", "The wind rises!");
    }



    /**
     * Make an attempt at reading the pictogram before the match starts
     */
    private void prereadPictogram() {
        final int tries = 10;

        for (int i = 0; i < tries; i++) {
            targetColumn = RelicRecoveryVuMark.from(relicTemplate);

            if (targetColumn != RelicRecoveryVuMark.UNKNOWN) {
                telemetry("Vuforia", "Saw " + targetColumn);
                break;
            }
        }
    }



    /**
     * Attempt an autonomous run specific to the arena location specified at construction
     */
    public void autonomous() {
        telemetry.clearAll();
        runtime.reset();

        try {
            /* * * * * * * JEWEL AND PICTOGRAM * * * * * * */
            boolean jewelKnocked = true;

            // Steal the jewel while reading the pictogram
            if (RED_STEAL_JEWEL && teamColor == TeamColor.RED || BLUE_STEAL_JEWEL && teamColor == TeamColor.BLUE) {
                jewelKnocked = stealJewel(new NestedInstruction() {
                    // Scan for a pictogram in parallel
                    @Override
                    public String run() {
                        RelicRecoveryVuMark oldPictogram = targetColumn;

                        if (targetColumn == RelicRecoveryVuMark.UNKNOWN)
                            targetColumn = RelicRecoveryVuMark.from(relicTemplate);

                        if (targetColumn != RelicRecoveryVuMark.UNKNOWN && targetColumn != oldPictogram)
                            telemetry("Vuforia", "Saw " + targetColumn);

                        return "";
                    }
                });
            }

            // Pictogram reading failed; prompt Vuforia to scan
            if (targetColumn == RelicRecoveryVuMark.UNKNOWN)
                updateTargetColumn();

            relicTrackables.deactivate();

            // Preemptively swivel the phone out of the way of the alignment arm
            srvPhone.setPosition(PHONE_POSITION_DODGE);

            // Schedule the jewel arm to be retracted either immediately after or shortly after the robot dismounts
            final double timeBeforeRetract = (jewelKnocked ? 0 : 0.8);

            scheduleConcurrent(new NestedInstruction() {
                @Override public String run() {
                    srvArmShoulder.setPosition(ARM_POSITION_UP);
                    srvArmSwivel.setPosition(SWIVEL_POSITION_CENTER);

                    return "";
                }
            }, timeBeforeRetract);

            if (alignMethod == AlignMethod.JEWEL)
                scheduleConcurrent(new NestedInstruction() {
                    @Override public String run() {
                        srvArmSwivel.setPosition(SWIVEL_POSITION_CRYPTOBOX);

                        return "";
                    }
                }, timeBeforeRetract + 0.75);

            // Schedule gyro PID to be enabled shortly after the robot has left the plate. It was observed that, before doing this,
            // the unpredictable ways in which the robot would move as it rolls from the plate could trick the gyro PID
            // into making unnecessary corrections that spiral the robot out of control
            scheduleConcurrent(new NestedInstruction() {
                @Override public String run() {
                    pidGyroEnabled = true;

                    return "";
                }
            }, 2);



            /* * * * * * * DISMOUNT AND ASSUME APPROACH ALGORITHM POSITION * * * * * * */

            // Experimentation with cryptobox alignment via bouncing range sensors off of either balancing plate
            if (plate == Plate.FRONT) {
                if (dismountMethod != DismountMethod.FAST)
                    dismountPlate(VEL_NORM_DRIVE, false, true);

                if (dismountMethod == DismountMethod.SENSOR) {
                    telemetry("Plate Bounce", "Starting");
                    pause(1);
                    setDrivetrainVelocity(VEL_NORM_DRIVE, (teamColor == TeamColor.RED ? 270 : 90));

                    final double[] bounceDistances = (teamColor == TeamColor.RED ? arenaMap.FRONT_RED_BOUNCE_DISTANCES : arenaMap.FRONT_BLUE_BOUNCE_DISTANCES);
                    final double range = bounceDistances[OpModeUtil.getColumnOrdinal(targetColumn) - 1];
                    final ModernRoboticsI2cRangeSensor sensor = (teamColor == TeamColor.RED ? snsRangeFront : snsRangeBack);
                    int confirmations = 0, confirmationsMax = 1;

                    while (confirmations < confirmationsMax) {
                        heartbeat();

                        final double reading = sensor.getDistance(DistanceUnit.INCH);

                        if (Math.abs(reading - range) < POSITION_EQU_THRESHOLD || Math.signum(reading - range) > 0)
                            confirmations++;
                        else
                            confirmations = 0;

                        if (confirmations == confirmationsMax)
                            telemetry("Plate Bounce", "Broke on reading " + String.format(Locale.getDefault(), "%.2f", reading) +
                                "Targeted " + String.format(Locale.getDefault(), "%.2f", range));
                    }

                } else if (dismountMethod == DismountMethod.HARD)
                    driveRelative((teamColor == TeamColor.RED ? 270 : 90), arenaMap.FRONT_ROUGH_DISTANCES[OpModeUtil.getColumnOrdinal(targetColumn) - 1], VEL_NORM_DRIVE);
            }

            halt();

            // Back plate requires a separate dismount that will be perpendicular to its approach of the cryptobox latch position
            if (plate == Plate.BACK) {
                dismountPlate(VEL_NORM_DRIVE, false, true);
                driveRelative((teamColor == TeamColor.RED ? 270 : 90), 16, VEL_NORM_DRIVE);

                if (teamColor == TeamColor.RED)
                    turnPid(-180);
            }

            // Proceed to the position required by the alignment algorithm
            Point columnDestination = arenaMap.getColumnLatchPosition(targetColumn);
            final double distanceToInsertion = (plate == Plate.BACK ? columnDestination.X - robotState.get(0, 0) : columnDestination.Y - robotState.get(1, 0));

            // On the front plate, the dismount and latch approach instructions are one and the same
            if (plate == Plate.FRONT) {
                if (dismountMethod == DismountMethod.FAST)
                    driveRelative((teamColor == TeamColor.RED ? 270 : 90), Math.abs(distanceToInsertion), VEL_DISMOUNT);

                scheduleConcurrent(new NestedInstruction() {
                    @Override public String run() {
                        srvLatch.setPosition(LATCH_POSITION_EXTENDED);

                        return "";
                    }
                }, 1);
                turnPid(-90);
            } else
                driveRelative((teamColor == TeamColor.RED ? 0 : 180), Math.abs(distanceToInsertion), VEL_NORM_STRAFE);

            // Front plate delivers delivers glyphs with the flipper
            if (plate == Plate.FRONT) {
                alignWithCryptobox();
                deliverPayload(false);
                shovePayload();
            // Back plate ejects glyphs through the intake
            } else {
                setIntakePower(-IDEAL_INTAKE_POWER);
                pause(2);
                driveRelative(90, 12, VEL_NORM_DRIVE);
                driveRelative(270, 6, VEL_NORM_DRIVE);
            }

            // Check for a dunk failure
            if (ATTEMPT_FAILED_SCORE_RECOVERY && plate == Plate.FRONT && getPayloadSize() > 0) {
                driveRelative(90, 2, VEL_NORM_STRAFE);
                driveRelative(180, 2, VEL_NORM_STRAFE);
                alignWithCryptobox();
                deliverPayload(false);
                shovePayload();
            }



            /* * * * * * * MULTI-GLYPH * * * * * * */

            // Front plate multi-glyph will attempt to retain a rough alignment with the key column to heighten the chances
            // that latch realignment succeeds
            if (ATTEMPT_FRONT_MULTIGLYPH && plate == Plate.FRONT) {
                // Drive to the edge of the glyph pit
                Point fishingPoint = arenaMap.getFishingPosition();
                final double fishingDistance = Math.abs(fishingPoint.X - robotState.get(0, 0));

                setIntakePower(IDEAL_INTAKE_POWER);
                driveRelative(90, fishingDistance, VEL_GONE_FISHING);

                // Activate the fishing algorithm
                fishForGlyphs();

                // Score additional glyphs
                if (fishingStatus == FishingStatus.SUCCESS) {
                    // Back the robot out of the glyph pit
                    retreatToRange(VEL_NORM_DRIVE, 20);

                    // Strafe to another column if specified
                    if (SCORE_MULTIGLYPH_IN_UNIQUE_COLUMN) {
                        // Pick column most likely to succeed
                        if (targetColumn == RelicRecoveryVuMark.CENTER)
                            targetColumn = RelicRecoveryVuMark.RIGHT;
                        else
                            targetColumn = (targetColumn == RelicRecoveryVuMark.LEFT ? RelicRecoveryVuMark.RIGHT : RelicRecoveryVuMark.LEFT);

                        final Point newTargetPosition = arenaMap.getColumnLatchPosition(targetColumn);
                        final double latchDistance = newTargetPosition.X - robotState.get(0, 0);

                        // Extend the latch to save some time during alignment
                        srvLatch.setPosition(LATCH_POSITION_EXTENDED);

                        // Make the adjustment
                        if (teamColor == TeamColor.RED)
                            driveRelative(latchDistance < 0 ? 0 : 180, Math.abs(latchDistance), VEL_NORM_STRAFE);
                        else
                            driveRelative(latchDistance < 0 ? 180 : 0, Math.abs(latchDistance), VEL_NORM_STRAFE);

                    // Target the same column. Strafe a bit to ensure the latch catches
                    } else {
                        srvLatch.setPosition(LATCH_POSITION_EXTENDED);
                        driveRelative(180, 3, VEL_NORM_STRAFE);
                    }

                    // Score
                    alignWithCryptobox();
                    deliverPayload(false);
                    shovePayload();
                }

                // Park if fishing failed
                if (fishingStatus == FishingStatus.OUT_OF_TIME)
                    retreatToRange(VEL_NORM_DRIVE, 15);
                // Disconnect from scored glyphs
                else
                    driveRelative(90, 6, VEL_NORM_DRIVE);
            }

            // Conclude
            setArmConfig(ArmConfig.UPRIGHT, true);
            halt();

        } catch (HeartbeatException e) {
            telemetry("Fatal", "Heartbeat failed during instruction " + currentInstruction);

        } catch (AbortException e) {
            telemetry("Fatal", "Out of time");

        } finally {
            telemetry("Concluding", "The wind falls");
            kill();
        }
    }



    /**
     * Steal the jewel. After scanning, the arm remains down and is presumably retracted concurrently with the robot's dismount
     *
     * @param instructions Instructions to be executed in parallel with jewel heist
     * @throws HeartbeatException If heartbeat protocol fails
     * @throws AbortException If a nested instruction finds reason to abort the calling instruction
     * @return Whether or not the jewel was swiped, or left to be swiped as the robot dismounts
     */
    private boolean stealJewel(final NestedInstruction... instructions) throws HeartbeatException, AbortException {
        telemetry("Jewel Thief", "Deploying");

        final double servoIncrement = -0.02;
        final double incrementBuffer = 0.1;
        final double scanArc = 0.01;
        final double scanReadings = 3;

        waitForServo(srvArmShoulder, ARM_POSITION_UP);
        waitForServo(srvArmSwivel, SWIVEL_POSITION_CENTER);
        waitForServo(srvArmShoulder, ARM_POSITION_DOWN + scanArc);

        // Swivel until detection
        while (snsColorJewel.red() == 0 && snsColorJewel.blue() == 0 && srvArmSwivel.getPosition() > 0) {
            srvArmSwivel.setPosition(srvArmSwivel.getPosition() + servoIncrement);
            pause(incrementBuffer, instructions);
        }

        // Scanning arc to double- and triple-check the color reading
        int blueTicker = 0, redTicker = 0;

        for (int i = 0; i < scanReadings; i++) {
            if (snsColorJewel.red() > snsColorJewel.blue())
                redTicker++;
            else if (snsColorJewel.blue() > snsColorJewel.red())
                blueTicker++;

            waitForServo(srvArmShoulder, srvArmShoulder.getPosition() + scanArc / scanReadings);
        }

        final TeamColor jewelColor = (redTicker > blueTicker ? TeamColor.RED : TeamColor.BLUE);
        final boolean convenient = (jewelColor != teamColor ^ teamColor == TeamColor.BLUE);

        telemetry("Jewel Thief", "Color: " + jewelColor + " | Convenient: " + convenient);

        // Jewel should be hit now
        if (!convenient) {
            waitForServo(srvArmSwivel, (teamColor == TeamColor.RED ? SWIVEL_POSITION_RIGHT : SWIVEL_POSITION_LEFT), instructions);
            return true;
        // Jewel should be hit as the robot leaves the plate
        } else {
            srvArmSwivel.setPosition(0.5);
            return false;
        }
    }



    /**
     * Move the robot
     *
     * @param relativeDirection Relative direction of movement (unsigned, inches, multiples of 45 on [0, 315])
     * @param distance Distance to drive (unsigned, inches)
     * @param velocity Drivetrain velocity (unsigned, in/s)
     * @param instructions Instructions to be executed each iteration of the driving loop. An instruction which returns
     *                     "abort" will forcibly end the drive
     * @return Whether or not the instruction completed without aborts
     * @throws HeartbeatException If heartbeat protocol fails
     * @throws AbortException If a nested instruction finds reason to abort the calling instruction
     */
    private boolean driveRelative(final int relativeDirection, final double distance, final double velocity, final NestedInstruction... instructions) throws HeartbeatException, AbortException {
        // Update some state stuff
        currentInstruction = Instruction.DRIVE;
        currentRelativeDriveHeading = relativeDirection;
        currentAbsoluteDriveHeading = OpModeUtil.getAbsRelToAbs(currentFieldHeading, relativeDirection);
        pidTargetHeading = OpModeUtil.wrapAngleGyro(currentFieldHeading + gyroFieldHeadingDifference);
        pidTargetVelocity = velocity;

        updateOrientation();
        telemetry(String.format(Locale.getDefault(), "Driving %.2fin @ rel %d° (%.2f° field, %.2f° pid)", distance, relativeDirection, currentFieldHeading, pidTargetHeading),
            String.format(Locale.getDefault(), "[%.2f, %.2f, %.2f, %.2f]", robotState.get(0, 0), robotState.get(1, 0), robotState.get(2, 0), robotState.get(3, 0)));

        // Calculations for localization
        final double movementTypeCompensation = (relativeDirection == 180 || relativeDirection == 0 ? DRIVE_TO_STRAFE_RATIO : 1);
        final int ticks = (int)(Math.abs(distance) * LINEAR_TO_TICKS * movementTypeCompensation);
        final int[] wheelSigns = OpModeUtil.getMecanumWheelSigns(relativeDirection);
        final double xInitial = robotState.get(0, 0);
        final double yInitial = robotState.get(1, 0);
        final double xDestination = xInitial + Math.cos(Math.toRadians(currentAbsoluteDriveHeading)) * distance;
        final double yDestination = yInitial + Math.sin(Math.toRadians(currentAbsoluteDriveHeading)) * distance;

        boolean driveComplete = false;
        boolean driveAborted = false;

        // Prepare the drivetrain
        for (int i = 0; i < drivetrain.length; i++) {
            drivetrain[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drivetrain[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            drivetrain[i].setTargetPosition(ticks * wheelSigns[i]);
        }

        // Record initial encoder position error signs for RUN_TO_POSITION simulation
        final double[] initEncoderDiffSigns = new double[drivetrain.length];

        for (int i = 0; i < drivetrain.length; i++)
            initEncoderDiffSigns[i] = Math.signum(drivetrain[i].getTargetPosition() - drivetrain[i].getCurrentPosition());

        // Driving loop
        while (!driveComplete) {
            // Slow down during the last portion of the journey
            final double slowThreshold = 4;
            final double distanceLeft = Math.abs(drvFrontLeft.getTargetPosition() - drvFrontLeft.getCurrentPosition()) * TICKS_TO_LINEAR / movementTypeCompensation;
            pidTargetVelocity = (!preemptivePidDriveBreaking || distanceLeft > slowThreshold ? velocity : VEL_CORRECTION);

            heartbeat();

            // Simulated RUN_TO_POSITION
            boolean drivetrainIsBusy = true;

            for (int i = 0; i < drivetrain.length; i++) {
                final double positionDiff = drivetrain[i].getTargetPosition() - drivetrain[i].getCurrentPosition();

                if (Math.abs(positionDiff) <= drivetrain[i].getTargetPositionTolerance() || Math.signum(positionDiff) != initEncoderDiffSigns[i]) {
                    drivetrainIsBusy = false;
                    break;
                }
            }

            // Perform nested instructions
            for (NestedInstruction instruction :instructions)
                if (instruction.run().equals("abort"))
                    driveAborted = true;

            if (driveAborted)
                break;

            // Track movement while driving. This way, if the instruction is aborted, the robot's position remains up-to-date
            final double distanceTraveled = Math.abs(drvFrontLeft.getCurrentPosition()) * TICKS_TO_LINEAR;

            robotState.set(0, 0, xInitial + distanceTraveled * Math.cos(Math.toRadians(currentAbsoluteDriveHeading)));
            robotState.set(1, 0, yInitial + distanceTraveled * Math.sin(Math.toRadians(currentAbsoluteDriveHeading)));

            // Verify completion
            driveComplete = !drivetrainIsBusy;
        }

        robotState.set(0, 0, xDestination);
        robotState.set(1, 0, yDestination);

        // Conclude
        halt();

        if (driveConclusionOrientationCorrection)
            correctOrientation();

        telemetry("Drive complete",
            String.format(Locale.getDefault(), "[%.2f, %.2f, %.2f, %.2f]", robotState.get(0, 0), robotState.get(1, 0), robotState.get(2, 0), robotState.get(3, 0)));

        return (!driveAborted);
    }



    /**
     * Drive to a specific position on the field by traversing the x and y distances individually
     *
     * @param position Destination
     * @param yBiased Traverse y-component first?
     * @param instructions Nested instructions
     * @throws HeartbeatException If heartbeat protocol fails
     * @throws AbortException If a nested instruction finds reason to abort the calling instruction
     */
    private void driveManhattan(final Point position, final boolean yBiased, NestedInstruction... instructions) throws HeartbeatException, AbortException {
        telemetry("Manhattan", "At " + String.format(Locale.getDefault(), "[%.2f, %.2f] @%.2f° | ", robotState.get(0, 0), robotState.get(1, 0), currentFieldHeading) +
            "Targeting " + String.format(Locale.getDefault(), "[%.2f, %.2f]", position.X, position.Y));

        final double xJump = position.X - robotState.get(0, 0);
        final double yJump = position.Y - robotState.get(1, 0);

        int firstDirection, secondDirection;
        double firstJump, secondJump;
        double firstPower, secondPower;

        if (yBiased) {
            firstDirection = (int)OpModeUtil.getRelEquOfAbs(yJump < 0 ? 270 : 90, currentFieldHeading);
            firstJump = yJump;
            firstPower = (firstDirection == 0 || firstDirection == 180 ? VEL_NORM_STRAFE : VEL_NORM_DRIVE);
            secondDirection = (int)OpModeUtil.getRelEquOfAbs(xJump < 0 ? 180 : 0, currentFieldHeading);
            secondJump = xJump;
            secondPower = (secondDirection == 0 || secondDirection == 180 ? VEL_NORM_STRAFE : VEL_NORM_DRIVE);
        } else {
            firstDirection = (int)OpModeUtil.getRelEquOfAbs(xJump < 0 ? 180 : 0, currentFieldHeading);
            firstJump = xJump;
            firstPower = (firstDirection == 0 || firstDirection == 180 ? VEL_NORM_STRAFE : VEL_NORM_DRIVE);
            secondDirection = (int)OpModeUtil.getRelEquOfAbs(yJump < 0 ? 270 : 90, currentFieldHeading);
            secondJump = yJump;
            secondPower = (secondDirection == 0 || secondDirection == 180 ? VEL_NORM_STRAFE : VEL_NORM_DRIVE);
        }

        telemetry("Manhattan", String.format(Locale.getDefault(), "<%d, %.2f, %.2f>", firstDirection, firstJump, firstPower) + " " +
            String.format(Locale.getDefault(), "<%d, %.2f, %.2f>", secondDirection, secondJump, secondPower));
        driveRelative(firstDirection, Math.abs(firstJump), firstPower, instructions);
        pause(0.2);
        driveRelative(secondDirection, Math.abs(secondJump), secondPower, instructions);
    }



    /**
     * Turn the robot
     *
     * @param targetAbsoluteHeading Absolute gyro heading to seek on [-180, 180]
     * @param instructions Nested instructions to be executed each iteration of the turning loop. An instruction which
     *                     returns "abort" will forcibly end the turn
     * @return Whether or not the turn was completed without aborting
     * @throws HeartbeatException If heartbeat protocol fails
     * @throws AbortException If a nested instruction finds reason to abort the calling instruction
     */
    private boolean turnPid(double targetAbsoluteHeading, NestedInstruction... instructions) throws HeartbeatException, AbortException {
        updateOrientation();
        telemetry(String.format(Locale.getDefault(), "Turning to gyro %.2f°", targetAbsoluteHeading),
                String.format(Locale.getDefault(), "Currently at gyro %.2f°", orientation.firstAngle));

        currentInstruction = Instruction.TURN;

        final double stableVelocityThreshold = 0.25;
        double headingError;
        boolean stableVelocity = false;
        boolean turnAborted = false;

        for (DcMotorEx motor : drivetrain) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        do {
            heartbeat();

            // Heading correction
            headingError = OpModeUtil.angleDev(orientation.firstAngle, targetAbsoluteHeading);
            final double headingCorrection = pidContTurn.getCorrection(headingError);

            drvFrontLeft.setPower(drvFrontLeft.getPower() - headingCorrection);
            drvFrontRight.setPower(drvFrontRight.getPower() + headingCorrection);
            drvBackLeft.setPower(drvBackLeft.getPower() - headingCorrection);
            drvBackRight.setPower(drvBackRight.getPower() + headingCorrection);

            // Ensure the robot isn't moving too fast before exiting the loop
            final double velocity = OpModeUtil.angularToLinear(drvFrontLeft.getVelocity(AngleUnit.RADIANS), DRIVETRAIN_WHEEL_DIAMETER / 2);
            stableVelocity = (Math.abs(velocity) <= stableVelocityThreshold);

            // Perform nested instructions
            for (NestedInstruction instruction : instructions)
                if (instruction.run().equals("abort"))
                    turnAborted = true;

            if (turnAborted) {
                telemetry("Aborting", "Nested instruction stopped the turn");
                break;
            }

        } while (Math.abs(headingError) > ORIENTATION_EQU_THRESHOLD || !stableVelocity);

        halt();

        currentFieldHeading = OpModeUtil.wrapAngleStandard((turnAborted ? orientation.firstAngle : targetAbsoluteHeading) - gyroFieldHeadingDifference);

        telemetry("Turn complete", String.format(Locale.getDefault(), "At gyro %.2f°, field %.2f°", orientation.firstAngle, currentFieldHeading));

        return (!turnAborted);
    }



    /**
     * Back the robot up until the rear range sensor passes a threshold
     *
     * @param proximity Proximity threshold
     * @throws HeartbeatException If heartbeat protocol fails
     * @throws AbortException If a scheduled concurrent instruction finds reason to abort the calling instruction
     */
    private void retreatToRange(double velocity, double proximity) throws HeartbeatException, AbortException {
        setDrivetrainVelocity(velocity, 270);

        final int confirmationsMax = 3;
        int confirmations = 0;

        while (confirmations < confirmationsMax) {
            heartbeat();

            final double reading = snsRangeBack.getDistance(DistanceUnit.INCH);

            if (Double.isNaN(reading))
                continue;
            else if (reading < proximity)
                confirmations++;
            else
                confirmations = 0;
        }

        halt();
    }


    /**
     * Accurately leave the balancing stone. The drivetrain is stopped as soon as the gyro confirms it's on level ground
     *
     * @param velocity Dismount velocity
     * @param reorient Whether or not to force the robot up against the edge of the plate after dismounting
     * @param halt Whether or not to halt the robot after dismounting, or to preserve velocity between this instruction and the next
     * @throws HeartbeatException If heartbeat protocol fails
     * @throws AbortException If a scheduled concurrent instruction finds reason to abort the calling instruction
     */
    private void dismountPlate(double velocity, boolean reorient, boolean halt) throws HeartbeatException, AbortException {
        // Leave the plate
        final double tiltedThreshold = 5, levelThreshold = 2;
        boolean passedThreshold = false;
        int confirmations = 0, confirmationsMax = 3;

        setDrivetrainVelocity(velocity, (teamColor == TeamColor.RED ? 270 : 90));

        while (confirmations < confirmationsMax) {
            heartbeat();

            if (Math.abs(orientation.secondAngle) > tiltedThreshold && !passedThreshold)
                passedThreshold = true;

            if (passedThreshold && Math.abs(orientation.secondAngle) < levelThreshold)
                confirmations++;
            else
                confirmations = 0;
        }

        if (halt)
            halt();

        // Forcibly reorient up against the plate
        if (reorient && halt) {
            pause(0.5);

            pidGyroEnabled = false;
            preemptivePidDriveBreaking = false;

            driveRelative((teamColor == TeamColor.RED ? 90 : 270), 2, VEL_NORM_DRIVE);

            pidGyroEnabled = true;
            preemptivePidDriveBreaking = true;

            robotState.set(1, 0, origin.Y + arenaMap.STONE_WIDTH / 2 + arenaMap.ROBOT_WIDTH / 2);
        }
    }



    /**
     * Align with the cryptobox using whichever method has been specified
     *
     * @throws HeartbeatException If heartbeat protocol fails
     * @throws AbortException If a scheduled concurrent instruction finds reason to abort the calling instruction
     */
    private void alignWithCryptobox() throws HeartbeatException, AbortException {
        if (alignMethod == AlignMethod.JEWEL)
            alignWithCryptoboxByJewelArm(true);
        else if (alignMethod == AlignMethod.LATCH)
            alignWithCryptoboxByLatch();
    }



    /**
     * Align with a cryptobox column using the jewel arm. Assumes that the robot is already positioned with its back parallel to the
     * cryptobox, and with the jewel arm past the right divider of the target column
     *
     * @param ram Whether or not to forcibly orient the robot against the cryptobox
     * @throws HeartbeatException If heartbeat protocol fails
     * @throws AbortException If a scheduled concurrent instruction finds reason to abort the calling instruction
     */
    private void alignWithCryptoboxByJewelArm(boolean ram) throws HeartbeatException, AbortException {
        telemetry("Alignment by jewel", "Underway");
        srvArmSwivel.setPosition(SWIVEL_POSITION_CRYPTOBOX);

        // Force the robot up against the cryptobox and reorient
        if (ram) {
            driveRelative(270, estimateDistanceFromCryptobox() + 1, VEL_NORM_DRIVE);
            cryptoboxHardReorient(!cryptoboxMap.isColumnEmpty(targetColumn));
        }

        // Create a buffer between the robot and the dividers
        srvArmShoulder.setPosition(ARM_POSITION_CRYPTOBOX);
        driveRelative(90, (cryptoboxMap.isColumnEmpty(targetColumn) ? 3 : 2), VEL_NORM_DRIVE);

        // Check for latch undershoot
        final double proximityThreshold = 2.5;
        int confirmations = 0, confirmationsMax = 1, failures = 0;
        double reading;

        if (snsRangeJewel.getDistance(DistanceUnit.INCH) < proximityThreshold)
            driveRelative(180, 2, VEL_NORM_STRAFE);

        // Strafe to proximity
        setDrivetrainVelocity(VEL_SCAN_STRAFE, 0);

        while (confirmations < confirmationsMax) {
            heartbeat();

            reading = snsRangeJewel.getDistance(DistanceUnit.INCH);

            if (reading < proximityThreshold) {
                confirmations++;
                telemetry("Alignment by jewel", String.format(Locale.getDefault(), "Confirmed proximity %.2fin", reading));
            } else {
                confirmations = 0;
                failures++;
            }
        }

        halt();
        telemetry("Alignment by jewel", "Completed with " + failures + " failures");
        srvArmShoulder.setPosition(ARM_POSITION_UP);
        driveRelative(90, 4, VEL_NORM_DRIVE);
    }



    /**
     * Align with a cryptobox column using the latch arm. Assumes that the robot is already positioned with its back parallel to the
     * cryptobox, and with the latch past the right divider of the target column
     *
     * @throws HeartbeatException If heartbeat protocol fails
     * @throws AbortException If a scheduled concurrent instruction finds reason to abort the calling instruction
     */
    private void alignWithCryptoboxByLatch() throws HeartbeatException, AbortException {
        telemetry("Alignment by latch", "Underway");
        srvArmSwivel.setPosition(SWIVEL_POSITION_CENTER);
        srvArmShoulder.setPosition(ARM_POSITION_UP);

        if (srvLatch.getPosition() != LATCH_POSITION_EXTENDED)
            waitForServo(srvLatch, LATCH_POSITION_EXTENDED);

        // Back up to wall
        final double distancePastInitialReading = 1.8;
        final double wallProximityFailsafe = 2.5;
        final int collisionConfirmationsBeforeBreak = 2;
        int initialReadingEncoderPosition = -1, collisionConfirmations = 0;
        boolean done = false, hitDivider = false;

        setDrivetrainVelocity(VEL_CREEP_DRIVE, 270);

        while (!done) {
            heartbeat();

            // Record the position of the initial reading
            if (!Double.isNaN(snsRangeLatchDivider.getDistance(DistanceUnit.INCH)) && initialReadingEncoderPosition == -1) {
                initialReadingEncoderPosition = drvFrontLeft.getCurrentPosition();
                telemetry("Alignment by latch", "Initial reading at position " + initialReadingEncoderPosition);
            }

            // Check for an arm collision with the divider
            if (snsRangeLatchDivider.getDistance(DistanceUnit.INCH) < wallProximityFailsafe && !Double.isNaN(snsRangeLatchWall.getDistance(DistanceUnit.INCH)))
                collisionConfirmations++;
            else
                collisionConfirmations = 0;

            if (collisionConfirmations >= collisionConfirmationsBeforeBreak) {
                done = true;
                hitDivider = true;
            }

            // Calculate distance since initial reading
            if (initialReadingEncoderPosition != -1) {
                final int currentPosition = drvFrontLeft.getCurrentPosition();
                final double distance = Math.abs(currentPosition - initialReadingEncoderPosition) * TICKS_TO_LINEAR;

                if (distance >= distancePastInitialReading || snsRangeLatchWall.getDistance(DistanceUnit.INCH) < wallProximityFailsafe) {
                    telemetry("Alignment by latch", "Backup terminated at position " + currentPosition);
                    done = true;
                }
            }
        }

        halt();

        // If the sensor readings point towards a collision between the arm and the divider, strafe to prevent it and retry
        // the alignment
        if (PREDICT_AND_CORRECT_DIVIDER_COLLISIONS && hitDivider) {
            telemetry("Alignment by latch", "Arm hit the divider; retrying");
            driveRelative(180, 3, VEL_NORM_STRAFE);
            alignWithCryptoboxByLatch();
            return;
        }

        // Align with divider
        setDrivetrainVelocity(VEL_SCAN_STRAFE, 0);

        final double dividerProximityThreshold = 4;
        int confirmations = 0, confirmationsMax = 1;

        while (confirmations < confirmationsMax) {
            heartbeat();

            final double reading = snsRangeLatchDivider.getDistance(DistanceUnit.INCH);

            if (reading < dividerProximityThreshold)
                confirmations++;
            else
                confirmations = 0;
        }

        telemetry("Alignment by latch", "Done");
        srvLatch.setPosition(LATCH_POSITION_TUCKED);
        halt();
    }



    /**
     * Reorient the robot as if it were pushed up against its cryptobox
     *
     * @param glyph Whether or not the column has a glyph in it
     */
    private void cryptoboxHardReorient(boolean glyph) {
        if (plate == Plate.BACK) {
            robotState.set(0, 0, arenaMap.getColumnLatchPosition(targetColumn).X - arenaMap.LATCH_PADDING);
            robotState.set(1, 0, arenaMap.ARENA_HEIGHT - (glyph ? arenaMap.GLYPH_WIDTH : arenaMap.COLUMN_DEPTH) - arenaMap.ROBOT_HEIGHT / 2);
        } else {
            robotState.set(0, 0, teamColor == TeamColor.RED ? arenaMap.ARENA_WIDTH - (glyph ? arenaMap.GLYPH_WIDTH : arenaMap.COLUMN_DEPTH) - arenaMap.ROBOT_WIDTH / 2 : (glyph ? arenaMap.GLYPH_WIDTH : arenaMap.COLUMN_DEPTH)+ arenaMap.ROBOT_WIDTH / 2);
            robotState.set(1, 0, arenaMap.getColumnLatchPosition(targetColumn).Y + arenaMap.LATCH_PADDING * (teamColor == TeamColor.RED ? 1 : -1));
        }
    }



    /**
     * Score held glyphs in the cryptobox. If only one glyph is held, the ramp will pause briefly halfway through raising
     * to prevent the glyph from falling out
     *
     * @param release Whether or not to pull the ramp back down after dunking
     * @throws HeartbeatException If heartbeat protocol fails
     * @throws AbortException If a scheduled concurrent instruction finds reason to abort the calling instruction
     */
    private void deliverPayload(boolean release) throws HeartbeatException, AbortException {
        int payloadSize = getPayloadSize();

        telemetry("Status", "Delivering " + payloadSize + " glyph(s)");
        cryptoboxMap.deliver(targetColumn, payloadSize);
        setIntakePower(IDEAL_INTAKE_POWER);

        if (payloadSize == 1) {
            waitForServo(srvRamp, RAMP_POSITION_FLAT);
            pause(0.4);
        }

        waitForServo(srvRamp, RAMP_POSITION_UP);

        if (release)
            srvRamp.setPosition(RAMP_POSITION_DOWN);

        setIntakePower(0);

        payload = new GlyphColor[] { GlyphColor.NONE, GlyphColor.NONE };
    }


    /**
     * Shove the payload into the cryptobox
     *
     * @throws HeartbeatException If heartbeat protocol fails
     * @throws AbortException If a scheduled concurrent instruction finds reason to abort the calling instruction
     */
    private void shovePayload() throws HeartbeatException, AbortException {
        preemptivePidDriveBreaking = false;

        driveRelative(90, 1, VEL_FAST_DRIVE);
        driveRelative(270, arenaMap.GLYPH_WIDTH, VEL_FAST_DRIVE);
        srvRamp.setPosition(RAMP_POSITION_DOWN);
        cryptoboxHardReorient(true);

        preemptivePidDriveBreaking = true;
    }



    /**
     * @param power Power on [-1.0, 1.0]
     */
    private void setIntakePower(double power) {
        drvIntakeLeft.setPower(power);
        drvIntakeRight.setPower(power);
    }



    /**
     * Schedule an instruction to be executed in parallel with heartbeat() some time from now
     *
     * @param nested Instruction
     * @param inTime How many seconds from now to execute
     */
    private void scheduleConcurrent(NestedInstruction nested, double inTime) {
        concurrentInstructions.add(new ConcurrentInstruction(nested, runtime.time() + inTime));
    }



    /**
     * Persistent glyph gathering. Assumes the robot is positioned directly in front of and facing the pit
     *
     * @throws HeartbeatException If heartbeat protocol fails
     * @throws AbortException If a nested instruction finds reason to abort the calling instruction
     */
    private void fishForGlyphs() throws HeartbeatException, AbortException {
        preemptivePidDriveBreaking = false;

        try {
            // The robot makes several "harpoon throws" into the pit
            final double harpoonInitialThrow = 5;
            final double harpoonThrow = 4;
            final double harpoonReel = 1;
            final int harpoonThrowCountMax = 3;

            // Aborts the fishing process if it succeeds or if time is running out
            final NestedInstruction aborter = new NestedInstruction() {
                @Override
                public String run() throws AbortException {
                    if (getPayloadSize() > 0)
                        payloadConfirmations++;
                    else
                        payloadConfirmations = 0;

                    // Payload is full
                    if (RAMP_SENSORS_ABORT_MULTIGLYPH && payloadConfirmations >= payloadConfirmationsMax) {
                        fishingStatus = FishingStatus.SUCCESS;

                        throw new AbortException();
                    // Time emergency
                    } else if (runtime.time() >= FISHING_ABORT_THRESHOLD) {
                        if (getPayloadSize() >= MULTIGLYPHS_BEFORE_ABORT)
                            fishingStatus = FishingStatus.SUCCESS;
                        else
                            fishingStatus = FishingStatus.OUT_OF_TIME;

                        throw new AbortException();
                    }

                    return "";
                }
            };

            // Make throws
            int harpoonThrows = 0;

            while (harpoonThrows < harpoonThrowCountMax) {
                // Drive a bit further on the initial throw
                if (harpoonThrows == 0) {
                    if (SCATTER_GLYPHS_ON_PILE_ENTRY)
                        setIntakePower(-IDEAL_INTAKE_POWER);

                    driveRelative(90, harpoonInitialThrow, VEL_FISH, aborter);
                }

                // Oscillate in and out
                setIntakePower(IDEAL_INTAKE_POWER);
                driveRelative(90, harpoonThrow, VEL_FISH, aborter);
                driveRelative(270, harpoonReel, VEL_FISH, aborter);

                harpoonThrows++;

                telemetry("Fishing", "Throw " + harpoonThrows);
            }

            // All throws were made before time was called. Do one last intake check
            fishingStatus = (getPayloadSize() >= MULTIGLYPHS_BEFORE_ABORT ? FishingStatus.SUCCESS : FishingStatus.OUT_OF_TIME);

        } catch (AbortException e) {
            telemetry("Fishing", "Aborting");
        }

        preemptivePidDriveBreaking = true;
        payloadConfirmations = 0;

        halt();
    }



    /**
     * Move the jewel arm to some orientation
     *
     * @param config Configuration
     * @param wait Pause between joint movements?
     * @throws HeartbeatException If heartbeat protocol fails
     * @throws AbortException If a scheduled concurrent instruction finds reason to abort the calling instruction
     */
    private void setArmConfig(ArmConfig config, boolean wait) throws HeartbeatException, AbortException {
        final Servo[] servos;
        final double[] positions;

        switch (config) {
            case UPRIGHT:
                servos = new Servo[] { srvArmShoulder, srvArmSwivel };
                positions = new double[] { ARM_POSITION_UP, SWIVEL_POSITION_CENTER };
            break;

            case CONSTRAINED:
                servos = new Servo[] { srvArmSwivel, srvArmShoulder };
                positions = new double[] { SWIVEL_POSITION_CRYPTOBOX, ARM_POSITION_CONSTRAINED };
            break;

            case UPRIGHT_FROM_DETECT:
                servos = new Servo[] { srvArmShoulder, srvArmSwivel, srvArmShoulder };
                positions = new double[] { 0.5, SWIVEL_POSITION_CENTER, ARM_POSITION_UP };
            break;

            case DETECT_JEWEL:
                servos = new Servo[] { srvArmSwivel, srvArmShoulder };
                positions = new double[] {SWIVEL_POSITION_CENTER, ARM_POSITION_DOWN };
            break;

            case DETECT_CRYPTOBOX:
                servos = new Servo[] { srvArmSwivel, srvArmShoulder };
                positions = new double[] { SWIVEL_POSITION_CRYPTOBOX, ARM_POSITION_CRYPTOBOX };
            break;

            default:
                servos = new Servo[0];
                positions = new double[0];
            break;
        }

        for (int i = 0; i < servos.length; i++)
            if (wait) {
                if (servos[i].getPosition() != positions[i])
                    waitForServo(servos[i], positions[i]);
            } else
                servos[i].setPosition(positions[i]);
    }



    /**
     * @return The number of glyphs currently on the ramp
     */
    private int getPayloadSize() {
        updatePayload();

        if (payload[0] != GlyphColor.NONE ^ payload[1] != GlyphColor.NONE)
            return 1;
        else if (payload[0] != GlyphColor.NONE && payload[1] != GlyphColor.NONE)
            return 2;
        else
            return 0;
    }



    /**
     * Update payload contents via inlaid ramp sensors
     */
    private void updatePayload() {
        final double bottomReading = snsRangeRampBottom.getDistance(DistanceUnit.INCH);
        final double topReading = snsRangeRampTop.getDistance(DistanceUnit.INCH);

        payload[0] = (bottomReading > 3 || Double.isNaN(bottomReading) ? GlyphColor.NONE : GlyphColor.AMBIGUOUS);
        payload[1] = (topReading > 3 || Double.isNaN(topReading) ? GlyphColor.NONE : GlyphColor.AMBIGUOUS);
    }



    /**
     * Micro-correct the orientation
     *
     * @throws HeartbeatException If heartbeat protocol fails
     * @throws AbortException If a scheduled concurrent instruction finds reason to abort the calling instruction
     */
    private void correctOrientation() throws HeartbeatException, AbortException {
        updateOrientation();

        final double error = pidTargetHeading - orientation.firstAngle;

        if (Math.abs(error) > SECONDARY_ORIENTATION_EQU_THRESHOLD) {
            telemetry("Correcting Orientation", String.format(Locale.getDefault(), "Error: %.2f°", error));
            turnPid(pidTargetHeading);
        }
    }



    /**
     * Send a servo to some position and suspend activity until it gets there
     *
     * @param servo Servo
     * @param position Position
     * @param instructions Nested instructions to be executed during the pause
     * @throws HeartbeatException If heartbeat protocol fails
     * @throws AbortException If a nested instruction finds reason to abort the calling instruction
     */
    private void waitForServo(Servo servo, double position, NestedInstruction... instructions) throws HeartbeatException, AbortException {
        final double sweepDistance = Math.abs(servo.getPosition() - position);
        final double timeToSweep = sweepDistance / 1.0;

        servo.setPosition(position);

        pause(timeToSweep, instructions);
    }



    /**
     * @return Estimated distance from the back of the robot's cryptobox
     */
    private double estimateDistanceFromCryptobox() {
        if (plate == Plate.BACK)
            return arenaMap.ARENA_HEIGHT - robotState.get(1, 0) - arenaMap.COLUMN_DEPTH - arenaMap.ROBOT_WIDTH / 2;
        else
            return (teamColor == TeamColor.RED ? arenaMap.ARENA_WIDTH - robotState.get(0, 0) : robotState.get(0, 0)) - arenaMap.COLUMN_DEPTH - arenaMap.ROBOT_WIDTH / 2;
    }



    /**
     * If no column is being targeted, column selection will be delegated to Vuforia. Otherwise, the cryptobox map is prompted for
     * the next column to deliver glyphs in
     *
     * @throws HeartbeatException If heartbeat protocol fails
     */
    private void updateTargetColumn() throws HeartbeatException {
        relicTrackables.activate();

        // Autonomous has just started; the robot has no particular column in mind
        if (targetColumn == RelicRecoveryVuMark.UNKNOWN) {
            final double timeStart = runtime.time();
            final double timeLimit = 6;
            final double timeBeforeSwivel = 0.2;
            final double servoIncrement = 0.01;
            final double servoIncrementBuffer = 0.1;
            final double[] servoBounds = { PHONE_POSITION_PICTOGRAM - 0.05, PHONE_POSITION_PICTOGRAM + 0.05 };
            double lastServoSwivel = timeStart;
            int servoSwivelDirection = -1;

            while (targetColumn == RelicRecoveryVuMark.UNKNOWN && runtime.time() - timeStart < timeLimit) {
                emptyHeartbeat();

                targetColumn = RelicRecoveryVuMark.from(relicTemplate);

                // Begin swiveling the camera if no pictogram is seen within timeBeforeSwivel seconds
                if (runtime.time() - timeStart >= timeBeforeSwivel && runtime.time() - lastServoSwivel >= servoIncrementBuffer) {
                    srvPhone.setPosition(srvPhone.getPosition() + servoIncrement * servoSwivelDirection);

                    lastServoSwivel = runtime.time();

                    if (srvPhone.getPosition() >= servoBounds[1] && servoSwivelDirection == 1)
                        servoSwivelDirection = -1;
                    else if (srvPhone.getPosition() <= servoBounds[0] && servoSwivelDirection == -1)
                        servoSwivelDirection = 1;
                }
            }

        // Glyphs have already been delivered, so choose the next delivery's destination
        } else
            targetColumn = cryptoboxMap.getNextColumn(targetColumn);

        // Vuforia failed; pick something
        if (targetColumn == RelicRecoveryVuMark.UNKNOWN) {
            telemetry("Vuforia", "Failed");
            targetColumn = (plate == Plate.FRONT ? RelicRecoveryVuMark.CENTER : (teamColor == TeamColor.RED ? RelicRecoveryVuMark.RIGHT : RelicRecoveryVuMark.LEFT));
        }

        telemetry("Target column updated", targetColumn + " @ latch position " + arenaMap.getColumnLatchPosition(targetColumn).toString());
    }



    /**
     * Give the drivetrain hard power. Note: the robot becomes unlinked from its state when using this method
     *
     * @param power Power on [-1.0, 1.0]
     * @param relativeDirection Relative direction of movement
     */
    private void setDrivetrainPower(double power, int relativeDirection) {
        currentInstruction = Instruction.DRIVE;
        final int[] signs = OpModeUtil.getMecanumWheelSigns(relativeDirection);

        updateOrientation();
        pidTargetHeading = orientation.firstAngle;

        for (int i = 0; i < drivetrain.length; i++)
            drivetrain[i].setPower(power * signs[i]);
    }



    /**
     * Set the drivetrain's target velocity. Note: the robot becomes unlinked from its state when using this method.
     * If velocity PID is not enabled and repeated calls to heartbeat() are not made, nothing will happen
     *
     * @param linearVelocity Target velocity (unsigned, inches/sec)
     * @param relativeDirection Relative direction
     */
    private void setDrivetrainVelocity(double linearVelocity, int relativeDirection) {
        currentInstruction = Instruction.DRIVE;
        currentRelativeDriveHeading = relativeDirection;
        pidTargetHeading = OpModeUtil.wrapAngleGyro(currentFieldHeading + gyroFieldHeadingDifference);
        pidTargetVelocity = linearVelocity;
    }



    /**
     * Perform high-frequency actions like PID corrections and DS/RC communication verification, usually
     * within the context of state-locked loops
     *
     * @throws HeartbeatException If the op mode is no longer active
     * @throws AbortException If a scheduled concurrent instruction finds reason to abort the calling instruction
     */
    private void heartbeat() throws HeartbeatException, AbortException {
        updateOrientation();

        // Activity check
        if (!client.opModeIsActive())
            throw new HeartbeatException();

        // Handle concurrent instructions
        for (int i = concurrentInstructions.size() - 1; i >= 0; i--) {
            ConcurrentInstruction inst = concurrentInstructions.get(i);

            if (runtime.time() >= inst.TIME) {
                telemetry("Concurrent Instruction", "Running");
                concurrentInstructions.remove(i).NESTED.run();
            }
        }

        // PID driving corrections
        if (currentInstruction == Instruction.DRIVE && runtime.time() - lastPidTime >= PID_INTERVAL) {
            // Heading correction
            if (pidGyroEnabled) {
                final PIDController controller = (currentRelativeDriveHeading == 180 || currentRelativeDriveHeading == 0 ? pidContGyroStrafe : pidContGyroDrive);
                final double headingError = orientation.firstAngle - pidTargetHeading;
                final double headingCorrection = controller.getCorrection(headingError);

                switch ((int)currentRelativeDriveHeading) {
                    case 0:
                        drvFrontLeft.setPower(drvFrontLeft.getPower() + headingCorrection);
                        drvFrontRight.setPower(drvFrontRight.getPower() - headingCorrection);
                        drvBackLeft.setPower(drvBackLeft.getPower() + headingCorrection);
                        drvBackRight.setPower(drvBackRight.getPower() - headingCorrection);
                    break;

                    case 90:
                        drvFrontLeft.setPower(drvFrontLeft.getPower() + headingCorrection);
                        drvFrontRight.setPower(drvFrontRight.getPower() - headingCorrection);
                        drvBackLeft.setPower(drvBackLeft.getPower() + headingCorrection);
                        drvBackRight.setPower(drvBackRight.getPower() - headingCorrection);
                    break;

                    case 180:
                        drvFrontLeft.setPower(drvFrontLeft.getPower() + headingCorrection);
                        drvFrontRight.setPower(drvFrontRight.getPower() - headingCorrection);
                        drvBackLeft.setPower(drvBackLeft.getPower() + headingCorrection);
                        drvBackRight.setPower(drvBackRight.getPower() - headingCorrection);
                    break;

                    case 270:
                        drvFrontLeft.setPower(drvFrontLeft.getPower() + headingCorrection);
                        drvFrontRight.setPower(drvFrontRight.getPower() - headingCorrection);
                        drvBackLeft.setPower(drvBackLeft.getPower() + headingCorrection);
                        drvBackRight.setPower(drvBackRight.getPower() - headingCorrection);
                    break;
                }
            }

            // Velocity correction
            if (pidVelocityEnabled) {
                double avgDrivetrainVelocity = 0;

                for (DcMotorEx motor : drivetrain)
                    avgDrivetrainVelocity += OpModeUtil.angularToLinear(Math.abs(motor.getVelocity(AngleUnit.RADIANS)), DRIVETRAIN_WHEEL_DIAMETER / 2);

                avgDrivetrainVelocity /= 4 * (currentAbsoluteDriveHeading == 180 || currentRelativeDriveHeading == 0 ? DRIVE_TO_STRAFE_RATIO : 1);
                final PIDController controller = (currentRelativeDriveHeading == 180 || currentRelativeDriveHeading == 0 ? pidContVelocityStrafe : pidContVelocityDrive);
                final double velocityError = pidTargetVelocity - avgDrivetrainVelocity;
                final double velocityCorrection = controller.getCorrection(velocityError);
                final int[] wheelSigns = OpModeUtil.getMecanumWheelSigns((int) currentRelativeDriveHeading);

                for (int i = 0; i < drivetrain.length; i++)
                    drivetrain[i].setPower(drivetrain[i].getPower() + velocityCorrection * wheelSigns[i]);
            }

            lastPidTime = runtime.time();
        }
    }



    /**
     * Heartbeat protocol without the fluff
     *
     * @throws HeartbeatException If the op mode is no longer active
     */
    private void emptyHeartbeat() throws HeartbeatException {
        if (!client.opModeIsActive())
            throw new HeartbeatException();
    }



    /**
     * Update orientation data via gyro
     */
    private void updateOrientation() {
        orientation = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }



    /**
     * Suspend activity temporarily
     *
     * @param duration Duration of pause (unsigned, seconds)
     * @param instructions Nested instructions to be executed each iteration of the pause
     * @throws HeartbeatException If heartbeat protocol fails
     * @throws AbortException If a nested instruction finds reason to abort the calling instruction
     */
    private void pause(double duration, NestedInstruction... instructions) throws HeartbeatException, AbortException {
        double startTime = runtime.time();

        while (runtime.time() - startTime < duration) {
            heartbeat();

            for (NestedInstruction instruction : instructions)
                instruction.run();
        }
    }



    /**
     * Suspend activity temporarily without heartbeat
     *
     * @param duration Duration of pause (unsigned, seconds)
     */
    private void uncheckedPause(double duration) {
        double startTime = runtime.time();

        while (runtime.time() - startTime < duration) {}
    }



    /**
     * Stop the robot
     *
     * @throws HeartbeatException If heartbeat protocol fails
     */
    private void halt() throws HeartbeatException {
        for (DcMotorEx motor : drivetrain)
            motor.setPower(0);

        robotState.set(2, 0, 0);
        robotState.set(3, 0, 0);

        for (PIDController cont : pids)
            cont.reset();

        currentInstruction = Instruction.NONE;
        lastPidTime = -1;
        pidTargetVelocity = 0;

        if (SUSPEND_HALT_WHILE_NONZERO_VELOCITY)
            while (Math.abs(OpModeUtil.angularToLinear(drvFrontLeft.getVelocity(AngleUnit.RADIANS), DRIVETRAIN_WHEEL_DIAMETER / 2)) > DRIVETRAIN_VELOCITY_ZERO_THRESHOLD) {
                emptyHeartbeat();
            }
    }



    /**
     * Stop the op mode completely
     */
    private void kill() {
        for (DcMotorEx motor : drivetrain)
            motor.setPower(0);

        drvIntakeLeft.setPower(0);
        drvIntakeRight.setPower(0);
        client.stop();
    }



    /**
     * Shorthand for printing timestamped telemetry
     *
     * @param caption Text preceding the divider
     * @param format Text after the divider
     */
    private void telemetry(String caption, String format) {
        telemetry.addData(String.format(Locale.getDefault(), "[%05.2fs] ", runtime.time()) + caption, format);
        telemetry.update();
    }
}
