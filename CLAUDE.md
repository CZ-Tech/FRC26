# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

FRC 2026 robot code for team 10016. A Java 17 / WPILib GradleRIO project driving a swerve chassis with PathPlanner-based autonomous and three-camera (front/left/right) Limelight AprilTag pose estimation fused with swerve odometry via a WPILib `SwerveDrivePoseEstimator` EKF.

The `src/main/python/` tree is **unrelated** to the robot — it's a standalone side project (audio → sheet-music via Spotify Basic Pitch, MJPEG camera streaming, camera-to-ground projection). Do not touch it when working on robot code, and do not wire it into the build.

## Build / deploy / test

This is a GradleRIO project — use the `gradlew` wrapper (`gradlew.bat` on Windows). WPILib/GradleRIO is resolved from the local WPILib install (`C:\Users\Public\wpilib\2026\maven`), so the GradleRIO VS Code extension / a local WPILib 2026 install is required.

- **Build / deploy to roboRIO (release):** `./gradlew deploy` (team number comes from `.wpilib/wpilib_preferences.json`, currently `10016`)
- **Debug deploy:** `./gradlew deploy -Pdebug` (or use the `WPILib roboRIO Debug` launch config)
- **Compile only / desktop simulate:** `./gradlew build` / `./gradlew simulate` (note: `includeDesktopSupport = false` in `build.gradle` — desktop/sim JNI is not wired up by default)
- **Run all JUnit tests:** `./gradlew test` (JUnit 5; the VS Code `WPIlibUnitTests` config sets `java.library.path` to `build/jni/release`).
- **Run a single test class / method:** `./gradlew test --tests "frc.robot.SomeClass"` / `--tests "frc.robot.SomeClass.someMethod"`

Vendor libraries live in `vendordeps/` (PathplannerLib 2026.1.2, Phoenix6 26.3.0, REVLib, Studica/NavX, WPILibNewCommands). They are auto-pulled by GradleRIO — do not edit by hand; regenerate via the WPILib vendor dependency UI.

## Code architecture

### Robot entry & lifecycle
`Main` → `Robot` (extends `TimedRobot`, the standard 20 ms loop model). `Robot.java` is the wiring hub: constructs the gyro, the `SwerveChassis`, the `ChassisSwerveOdo` odometry, and the `OdoSwerveDrivetrain`; binds the `Checklist`; builds the `Auto`. There is **no WPILib command-based `RobotContainer`** — teleop and test-mode driving are handled imperatively in the `*Periodic` methods of `Robot` (read gamepad axes → call `driveTrain.driveRobotFieldCentricLocalAlliance(...)`). The global `CommandScheduler.getInstance().run()` is still called from `robotPeriodic` so PathPlanner auto commands execute.

### Swerve & odometry (the core subsystem)
Three cooperating classes in `subsystems/chassis/`:

- **`SwerveChassis`** — a `record` holding the four `SwerveModule`s (lf, lb, rf, rb) plus the `SwerveDriveKinematics`. Singleton via `getInstance()`. **Module order matters**: `SwerveDriveKinematics` is built with `{lf, rf, lb, rb}` translation offsets, and every consumer (`OdoSwerveDrivetrain.driveRobot`, `ChassisSwerveOdo.getSwerveModulePositions`, `getVelocity_robotRelative`) indexes module-state arrays in that same `{lf, rf, lb, rb}` order. Changing the array order anywhere without updating all call sites will silently invert/mix up wheels.
- **`ChassisSwerveOdo`** — pose estimation. Owns the `SwerveDrivePoseEstimator` EKF and runs its own **background thread** via `TaskLoopFrame` at 100 Hz (`estimateTask`), independent of the 50 Hz robot loop. On startup it refuses to mark itself `isReady()` until at least one Limelight sees ≥ `Constants.leastTagStart` AprilTags, then seeds the EKF from the closest reliable camera. Each tick it: updates from gyro + module positions, feeds current heading back to all three Limelights for MegaTag2, and fuses vision — using **MegaTag1** when ≥ 2 tags are visible (unambiguous rotation) and **MegaTag2** otherwise (gyro-anchored rotation via stdDev = 9999999 on theta). Vision is skipped entirely when `|gyro.getRate()| > 360` deg/s. All access to `m_poseEstimator` is `synchronized`.
- **`OdoSwerveDrivetrain`** — `SubsystemBase` that turns `ChassisSpeeds` into per-module `SwerveModuleState`s. Field-centric driving applies an alliance flip (Red alliance = +π) and a runtime `angleOffset` (re-pegged to "current heading is forward" via `setNowForward()`, triggered by `Options + Create` on gamepad 0).

### SwerveModule hardware abstraction
`lib/SwerveModule` is the reference implementation: TalonFX (Phoenix6) drive + turn motors, with the **turn angle** coming from an analog absolute encoder (`AnalogInput`) read via voltage — `getTurningEncoderRadians()` applies the per-module offset, wraps to ±π, and multiplies by `turnEncoderDirection.sign`. Drive distance/velocity are scaled by `Constants.kDriveEncoderDistancePerPulse` and `Constants.DriveEncoderRotationePerMetre`. `lib/SwerveModuleNativeRelativeEncoder` is an alternate variant using the motor's relative encoder. `lib/LimelightHelpers.java` is the vendored Limelight API wrapper.

### Constants (`Constants.java`)
Single flat class of tunables + helpers. Notable ones: `SpeedFactor` (global drive-output multiplier — **set to 1 for competition, kept low for bench testing**; the FIXME at line 11 calls this out), `kMaxSpeedMetersPerSecond` (power is computed as `speed / kMaxSpeed`), `toPower`/`toSpeed`/`turnToSpeed` conversion helpers, per-corner CAN IDs / analog encoder ports / **encoder offsets in radians** / motor & encoder direction signs / `Translation2d` module positions, and the three Limelight names. Wiring changes to the physical robot almost always require editing here.

### Autonomous (`command/Auto.java`)
Wraps PathPlanner's `AutoBuilder` + `PPHolonomicDriveController`. `Auto.bindOperations()` returns a fluent `CommandsBinder` whose `.add(name, Runnable|Command)` registers `NamedCommands` (events referenced from `.auto` files in `src/main/deploy/pathplanner/`). Calling `.prepareAuto(odo, drivetrain)` constructs the singleton `Auto` (guarded against double-init by an `AtomicBoolean`). `prepareAutoChooser()` publishes the `Auto Chooser` SendableChooser to SmartDashboard; the selected command is polled in `disabledPeriodic` (via `robotPeriodic`'s `auto.getAuto()` when disabled) and re-selected in `autonomousInit`. PathPlanner robot config is loaded via `RobotConfig.fromGUISettings()` — keep `src/main/deploy/pathplanner/settings.json` in sync with the PathPlanner GUI (it carries robot dimensions, gearing, module positions).

### Checklist (`Checklist.java`)
An `enum` of pre-match readiness booleans (`MotorsOK`, `odoReady`, `odoAccurate`, `limelight{Front,Left,Right}Connected`, `allianceGot`, `autoChosen`). Each item's `BooleanSupplier` is bound in `Robot.robotInit` and re-evaluated every loop in `putAll()`, which also publishes a combined `Prepared` boolean to SmartDashboard. The source FIXMEs are emphatic: **the `Prepared` light must be green before a match**.

### `frames/TaskLoopFrame`
Minimal fixed-rate background-loop utility (not WPILib's `Tracer`): a `Thread` that calls an abstract `update()` at a target Hz and reports `lastTimeCostms`. Used by `ChassisSwerveOdo` for the 100 Hz estimation loop. Add new background loops by subclassing and calling `start()`.

### PathPlanner data & deploy
`src/main/deploy/pathplanner/` holds `settings.json`, `navgrid.json`, path `.path` files and `.auto` files; all of `src/main/deploy/` is pushed to the roboRIO by the `frcStaticFileDeploy` artifact. Editing autos/paths is done in the PathPlanner GUI; the files are the source of truth committed here.

## Conventions & gotchas

- **Comments are mostly in Chinese.** Match the surrounding language/style when editing.
- **No `RobotContainer`.** New subsystem commands are either registered as PathPlanner `NamedCommands` (for autos) or driven imperatively from `Robot`'s periodic methods.
- **Module-state array order is `{lf, rf, lb, rb}`** everywhere — see the warning in the swerve section above.
- The turn motor PID (`kPModuleTurningController = 0.2`) drives angle to target; drive output is essentially feed-forward (`kPModuleDriveController = 0.001`) plus `driveFeedForward = speed / kMaxSpeed`. The drive PID is deliberately near-zero.
- `getTurnMotorStatus()` in `SwerveModule` currently queries `m_driveMotor` (likely a copy-paste bug) — be aware if touching motor-health reporting.
- Odometry runs on its own thread; anything that reads/writes `m_poseEstimator` or the gyro from outside `ChassisSwerveOdo` must respect the `synchronized` block already there.
- Several subsystems (`Intaker`, `Shooter`, `MagicIntaker`, `LiftUp`) exist in `subsystems/` but are **commented out / unused** in `Robot.java` — treat them as work-in-progress, not active.

最后请说中文