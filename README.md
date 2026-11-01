# Team 6421 | Robot Code and 2027 Engineering Preparation

How do you turn a controller button into a robot that drives, collects balls, aims, and shoots?
This project connects Java code to real motors, sensors, and cameras for **FIRST Robotics Competition (FRC)**.

You do not need to understand every file to get started. **First goal: make the robot move safely.**
Follow the joystick input from the driver's hand to the wheels, then explore shooting.

## Student setup: start here

**Use one project for all ten weeks:** the
[training fork's `prepare/2027` branch](https://github.com/IEvangelist/2026-robot-code/tree/prepare/2027).
You do **not** need to create a new WPILib project each week or use different robot code.
Do not clone the team's default branch for these lessons: the examples require the upgraded
alpha-8 project, including its Gradle wrapper, vendor dependencies, and simulator.

### 1. Install tools once (Windows x64)

- Install [Git for Windows](https://git-scm.com/downloads/win) and [VS Code](https://code.visualstudio.com/).
- Install a **Java 25 JDK**, not a JRE or Java 17. For example, select version 25, Windows, x64,
  and JDK on the [Temurin download page](https://adoptium.net/temurin/releases/?version=25).
  Remember its installation folder. School-managed computers may require an IT administrator.
- A separate Gradle installation is **not** required; use the project's `gradlew.bat`.
  Internet access is needed for the first dependency download.
- Elastic is optional for console lessons. For the field/wheel display, download the Windows
  application from [Elastic 2026.1.2](https://github.com/Gold872/elastic_dashboard/releases/tag/v2026.1.2),
  extract it if packaged as a ZIP, and open the executable.
- The Windows simulator also needs the
  [Microsoft Visual C++ Redistributable (x64)](https://learn.microsoft.com/en-us/cpp/windows/latest-supported-vc-redist).
  Install it if it is not already present; school computers may need IT assistance.

Open a new PowerShell terminal and configure Java for that terminal. Enter the **JDK folder**
when prompted, not its `bin` folder:

```powershell
$env:JAVA_HOME = (Read-Host 'Paste your Java 25 JDK installation folder').Trim('"')
if (-not (Test-Path "$env:JAVA_HOME\bin\javac.exe")) {
    throw 'That folder is not a JDK. Find the Java 25 folder containing bin\javac.exe.'
}
$env:Path = "$env:JAVA_HOME\bin;$env:Path"
java -version
javac -version
```

Both version commands must report **25**. These commands affect this terminal only.
To save the setting for future terminals, add `JAVA_HOME` with that folder in Windows
**Edit environment variables for your account**, then restart VS Code.
You do not need to install WPILib IDE extensions to run these terminal exercises.

### 2. Get the correct project

In PowerShell, from a folder where you keep projects:

```powershell
git clone --branch prepare/2027 --single-branch https://github.com/IEvangelist/2026-robot-code.git
Set-Location .\2026-robot-code
.\gradlew.bat --version
```

Check **Gradle 9.4.1** and **JVM 25**. The first run can take time downloading dependencies.
Run `git branch --show-current`: it must say `prepare/2027`.
Open this folder in VS Code using **File > Open Folder**.
If you already have a different checkout, clone into a new named folder instead of overwriting it.

Without Git, use the training branch link above and **Code > Download ZIP**, extract it,
then open a terminal in the extracted folder containing `build.gradle` and `gradlew.bat`.
Do not download the default branch ZIP.

### 3. Run a weekly console example

Open a weekly Gist from the table below. Download its `WorkshopNN.java` using **Raw**
and save it into a new `workshops` folder in the project root. Download `run-workshop.gradle`
into the project root; the helper is identical in every weekly Gist.
Check file extensions: neither file should end in `.txt`.

```text
2026-robot-code
  build.gradle
  gradlew.bat
  run-workshop.gradle
  workshops
    Workshop01.java
```

From that root folder, run:

```powershell
.\gradlew.bat -I .\run-workshop.gradle runWorkshop -Pworkshop=Workshop01
```

For Week 02 download `Workshop02.java` into the same folder and use `-Pworkshop=Workshop02`,
and so on through `Workshop10`. **Do not paste Markdown excerpts into the source files**:
the downloadable Java companions are complete programs; excerpts are explanations.

These commands print results in the terminal. They do not open Elastic or move its robot.
Week 01 prints about `+0.913 m/s` for stick `-0.50` and finishes with `BUILD SUCCESSFUL`.
Other weeks print tables or `PASS` messages. An assertion/build error is a failed check, not success.

### 4. Run the existing driving simulation (a separate command)

```powershell
.\gradlew.bat run -PswerveSim
```

In Elastic choose **localhost**, then **Ctrl+O** and open this project's `elastic-swerve-sim.json`.
Choose **Auto demo** in the mode dropdown. In the WPILib simulation GUI select
**Teleoperated**. Watch the status, wheel speeds, and field position.
For controller mode, assign the Xbox device to **Joystick[0]** in that GUI.
Choose Stopped or disable to stop requests; close the GUI to end the simulation.
Do not run two simulations at the same time.

**Never use `deploy`, and do not run bare `gradlew run` for a lesson.**
Bare `run` starts the full competition robot rather than the standalone exercise.
Do not connect or enable real hardware for this curriculum.

### What works now, and what is a future exercise?

| Capability | Available now? | What students run |
|---|---|---|
| Ten complete console examples | Yes; calculations, graphs-as-tables, and synthetic checks. | `runWorkshop` with the downloaded week's number. |
| Virtual swerve driving, auto demo, and controller | Yes; separate simulator and Elastic layout. | `run -PswerveSim`. |
| Week 03 command factory | Compiles, but its console entry point does not schedule it. | Run the console model independently; live command integration is a supervised stretch task. |
| Lift/placement/projectile examples | Math, references, geometry, and synthetic data only. | Weeks 06-10 console programs; no lift or flying object appears in Elastic. |
| Integrated lift/placement autonomous mission | Not supplied as a finished subsystem or command harness. | Optional design/implementation work, not a prerequisite for the supplied examples. |
| Real robot hardware | Not validated or ready to enable. | Do not deploy these lessons. |

### If something fails

| Symptom | Next step |
|---|---|
| `gradlew.bat` not found | Open a terminal in the project root, not `workshops` or the ZIP's outer folder. |
| Wrong Java / unsupported class version | Repeat the Java setup; check `JAVA_HOME`, `java -version`, and Gradle JVM version. |
| Missing Java lesson | Check the exact name `workshops\WorkshopNN.java`, including no `.txt` suffix. |
| `edu.wpi.first` imports / missing alpha-8 classes | Use the fork's `prepare/2027` branch, not a 2026 project. |
| Dependency download errors | Check internet access and school proxy/firewall policy; retry after correcting connectivity. |
| Simulator JNI / missing DLL error | Install the x64 Visual C++ Redistributable linked above, then retry the simulator. |
| Console succeeds but Elastic is still | Console programs do not drive it; run the separate simulator command. |
| Elastic connected but blank | Open `elastic-swerve-sim.json` with Ctrl+O. |
| Field visible but no motion | Choose Auto demo and enable Teleoperated in the simulation GUI. |

If those steps do not resolve it, save the command and its first error message for a teacher.
Do not fix setup errors by changing vendor versions, CAN IDs, or random motor settings.

**Verified student path:** on November 1, 2026, a fresh clone of this fork at `51b0185`
was checked with the examples downloaded from the published Gists. The full build, all eleven
native simulator tests, and all ten console examples passed on Windows x64 / Java 25.
The documented simulator command also started successfully from that clone and accepted an
Elastic connection on localhost port 5810; it was stopped after the check.
An existing dependency cache was used; this does not verify your school's download permissions,
other operating systems, or real hardware.

## Students: apply for GitHub Education benefits

GitHub Education provides eligible students with free access to developer tools and learning
resources. **Apply using your school-issued email address** on your own GitHub account:

1. Sign in to your personal GitHub account. If you already have one, use it rather than creating
   a second account for robotics.
2. [Add your school email address](https://github.com/settings/emails) to that account and verify
   it using the confirmation email GitHub sends.
3. Open [GitHub Education benefits](https://github.com/settings/education/benefits), select
   **Start an application**, and complete the student application using your verified school email.
4. Provide proof of current enrollment if requested, such as a dated school ID, class schedule,
   transcript, or enrollment verification letter. Submit these only through GitHub's application;
   do not put personal documents in this repository or a Gist.
5. After approval, visit the [GitHub Education portal](https://github.com/education) to explore
   and activate the benefits available to you.

Students must be **at least 13** and currently enrolled in a degree- or diploma-granting program,
including high school or homeschool. A school email alone does not guarantee approval; GitHub may
require additional verification. Benefits and their availability can change.

If your school does not issue email addresses, follow GitHub's documentation requirements rather
than using someone else's address. If its email domain is not recognized, contact
[GitHub Education Support](https://support.github.com/contact/education).
See the [official student application guide](https://docs.github.com/en/education/about-github-education/github-education-for-students/apply-to-github-education-as-a-student)
for eligibility, accepted documents, and troubleshooting.

## Weekly engineering readiness plans

**Target: January 9, 2027.** These lessons prepare a mixed-experience team for programming,
controls/math, and testing/coordination using flexible **45-minute workshops** and simulation.
They cover spatial awareness, motion, lifting, placement, and shooting, without predicting the next game.

Open the [complete curriculum index](https://gist.github.com/IEvangelist/6febeba17164d2e74106a0e439d75403)
for setup, scheduling, the shared lesson format, and the readiness rubric.
Each weekly Gist contains detailed objectives, two timed workshops, explained WPILib capabilities,
release-pinned API links, a runnable Java example, experiments, debugging guidance, and mentor checks.
Examples target **WPILib 2027.0.0-alpha-8 and Java 25**, using command framework v2.

| Week | Dates | Detailed lesson | Main objectives | Demonstration / deliverable |
|---|---|---|---|---|
| 01 | Nov 2-8 | [Make the robot listen](https://gist.github.com/IEvangelist/d6f61c4c968417c707454c7bf4f565bd#file-week-01-md) | Launch simulation; trace joystick input; read telemetry; review a small diff. | Drive/stop demonstration and a prediction-versus-result table. |
| 02 | Nov 9-15 | [Know where you are](https://gist.github.com/IEvangelist/0a5095f4eccac7f625237cc32de0e44a#file-week-02-md) | Units, pose, rotation, and robot/field coordinate frames. | Frame-labeled sketches and three-heading conversion checks. |
| 03 | Nov 16-22 | [Motion with a beginning and an end](https://gist.github.com/IEvangelist/c2fa3ab784ec9eab1122a4b899c1d984#file-week-03-md) | Velocity/acceleration; loop timing; ownership; timeout and cancellation. | Motion trace and an explanation of requests versus actual movement. |
| 04 | Nov 23-29 | [Become a robot detective](https://gist.github.com/IEvangelist/e59730206d01639389eea1c35c7b369c#file-week-04-md) | Diagnose planted faults; isolate causes; write regression checks. **Light/catch-up week.** | Reproducible fault report with failing and passing evidence. |
| 05 | Nov 30-Dec 6 | [Reach a target, not just a timer](https://gist.github.com/IEvangelist/9a7d5f699bd80decf880fad6cffd5a30#file-week-05-md) | Error, P control, tolerance, ground truth, and measurement uncertainty. | Feedback plot with successful and intentionally unsuccessful trials. |
| 06 | Dec 7-13 | [Lift, reach, and place](https://gist.github.com/IEvangelist/f1db7789ada32333f40d9394c1047168#file-week-06-md) | Gearing/drum travel; motion profiles; tool offsets and placement geometry. | Lift reference checks and a tool-position diagram. |
| 07 | Dec 14-20 | [Shooting is a model plus measurements](https://gist.github.com/IEvangelist/00f6fcd087c6e9d54690daf52f28df4a#file-week-07-md) | RPM/surface speed; projectile assumptions; interpolation and calibration. | Graph, midpoint check, assumptions, and a proposed measurement sheet. |
| 08 | Dec 21-27 | [Coordinate actions without wishful thinking](https://gist.github.com/IEvangelist/2dd9f2af4c4f9dd9affb2377d3ba23f7#file-week-08-md) | Readiness dwell; state machines; honest outcomes and cleanup. **Optional holiday integration.** | Success, timeout, and cancellation demonstrations. |
| 09 | Dec 28-Jan 3 | [Rehearse an unfamiliar challenge](https://gist.github.com/IEvangelist/5b2137d974a83e2d37a5bb074352f533#file-week-09-md) | Requirements, interfaces, inverse tool geometry, and task decomposition. **Optional holiday rehearsal.** | Requirements sketch, small task board, and a geometric prototype. |
| 10 | Jan 4-8 | [Be ready to start cranking](https://gist.github.com/IEvangelist/82ee0ba3f02af8ad604daa7d188823e6#file-week-10-md) | Acceptance contracts; capstone evidence; first-day engineering workflow. | Individual explanations, failure demonstrations, and a kickoff task plan. |

The Gists are **unlisted, not private**: anyone with these links can read them, including through this
README if it is published. All ten Java companions have been compiled and run against alpha-8.
New lift, placement, and coordination exercises are clearly distinguished from existing simulator features.
Real hardware still requires verified mappings, firmware, wiring, limits, and calibration.

## What can the robot do?

| Part | Job |
|---|---|
| Swerve drivetrain | Drive forward, sideways, and rotate using independently steered wheels. |
| Intake | Collect balls with a roller and a movable arm. |
| Transition | Move balls from the hopper toward the shooter. |
| Shooter | Spin the launch wheels, turn the turret, and adjust the hood that guides the shot. |
| Climber | Move the climbing mechanism to a requested height. |
| Cameras | Read AprilTags, the square field markers that help estimate the robot's location. |

The robot also has autonomous routines: sequences of actions it can run without driver input.

## How the code fits together

The project uses **WPILib's command-based framework**:

- A **subsystem** owns a piece of hardware: "I control the drivetrain."
- A **command** describes an action: "Drive using the joystick."
- The **scheduler** runs commands and subsystem updates, normally every 20 milliseconds.

```text
Controller sticks
    -> RobotContainer's default drive command reads the input
    -> Command tells a subsystem what to do
    -> Subsystem sends requests to motors
    -> Sensors report what actually happened
```

### Start with these files

| File | What to look for |
|---|---|
| [RobotContainer.java](src/main/java/frc/robot/RobotContainer.java) | How do joystick inputs become drive requests in `configureBindings()`? |
| [CommandSwerveDrivetrain.java](src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java) | How does `applyRequest()` send those requests to the drivetrain? |
| [Robot.java](src/main/java/frc/robot/Robot.java) | Where does the scheduler run? What happens when robot modes change? |
| [Constants.java](src/main/java/frc/robot/Constants.java) | Where are the controller port and drive smoothing settings? |
| [TargetCommand.java](src/main/java/frc/robot/command/TargetCommand.java) | How does one command coordinate aiming and feeding balls? |
| [ShooterSubsystem.java](src/main/java/frc/robot/subsystems/ShooterSubsystem.java) | How do turret angle, wheel speed, and hood position work together? |
| [SOTMTable.java](src/main/java/frc/robot/SOTMTable.java) | How does distance become a requested shooter speed? |

Other useful stops: [subsystems](src/main/java/frc/robot/subsystems), [autonomous commands](src/main/java/frc/robot/command/autoCommands), and [camera processing](src/main/java/frc/robot/WarriorCamera.java).

## First challenge: get moving

The driving code is already connected. The first task is to understand and verify it, not write a drivetrain from scratch.

An Xbox controller on **Driver Station USB slot 0** controls driving:

| Input | Requested movement |
|---|---|
| Left stick forward/back | Forward/back across the field. |
| Left stick left/right | Sideways across the field. |
| Right stick left/right | Turn counterclockwise/clockwise. |

This is **field-centric** driving: the left stick requests a direction across the field rather than toward the robot's nose. The gyro (heading sensor) and alliance setting determine that reference.

### Follow one joystick value

In [RobotContainer.java](src/main/java/frc/robot/RobotContainer.java), find:

```java
.withVelocityX(xDriveSlew.calculate(-joystick.getLeftY() * MaxSpeed))
```

1. `getLeftY()` reads the stick, roughly from -1 to +1.
2. The minus sign makes pushing forward a positive request.
3. `MaxSpeed` scales that request to meters per second.
4. `xDriveSlew` smooths changes so translation does not jump instantly.
5. The drivetrain turns the request into individual wheel directions and motor outputs.

The same idea controls sideways motion. Turning uses `MaxAngularRate`, measured in radians per second.
A small **deadband** ignores requests near zero. The **default command** keeps reading the sticks whenever no other command owns the drivetrain.

### First supervised drive check

1. **Start disabled:** Have a mentor verify the deployed code, battery, motor configuration, wheel calibration, and clear test area. Building the project alone does not put code on the robot.
2. **Check the controller:** In Driver Station, confirm slot 0 and watch the stick values change. Use teleoperated mode, not autonomous, for this exercise.
3. **Agree on a slow test:** Have a mentor set conservative drive and turning limits before enabling. The current settings are not a beginner-speed mode.
4. **Test one motion at a time:** Use small inputs for forward/back, sideways, then rotation. Avoid other buttons and triggers: they operate the intake, shooter, and climber.
5. **Release and observe:** Translation requests should ramp back toward zero; turning has no slew limiter here. Disable immediately if motion is unexpected.
6. **Check field-centric behavior:** With the mentor's help, turn the robot and repeat the same field-direction request. Verify the heading and alliance reference before changing signs in code.

If nothing moves, disable first. Check Driver Station connection/enabled status, controller slot, robot errors, and motor-controller connectivity with a mentor before editing code.

### Standalone TalonFX swerve simulation

[SimTalonFXDrivetrain](src/main/java/frc/robot/subsystems/SimTalonFXDrivetrain.java) is a separate `SubsystemBase` learning example. It models four swerve modules with eight simulated TalonFX controllers and WPILib motor physics. **It is not connected to the existing robot or joystick controls**, and its constructor refuses to run on real hardware.

- `drive(forward, left, rotation)` requests **robot-relative** movement in m/s and rad/s, unlike the existing field-centric driver controls.
- `getTargetStates()` shows requested wheel speeds and angles; `getModuleStates()` shows simulated results.
- `getPose()` reports the simulated field position. `resetPose(...)` changes that reference, not wheel motion.
- `stop()` removes motor output. Disabling also clears the request; wheels may coast briefly.

For a separate simulation-only exercise, a command could request a slow forward move:

```java
var simDrive = new SimTalonFXDrivetrain();
var forward = simDrive.runEnd(() -> simDrive.drive(0.5, 0, 0), simDrive::stop)
    .withTimeout(2.0);
forward.schedule();
```

This needs an enabled WPILib simulation and a running command scheduler, which calls `simulationPeriodic()` every 20 ms. Do not manually step it as well. Close the subsystem when the exercise ends. It reserves simulated CAN IDs **41-48**, so do not reuse those IDs in the same simulation.

The example assumes perfect traction and uses illustrative dimensions, gearing, inertia, and steering gains. It does not model collisions, battery sag, tire slip, or the new 2027 hardware. Drive voltage uses a simple speed feedforward; steering uses ideal simulated angle feedback rather than a CANcoder.

Simulation settings live together in `Constants.SimDriveConstants` in [Constants.java](src/main/java/frc/robot/Constants.java): speed limit, loop timing, dimensions, gear ratios, inertia, steering gains, and CAN IDs. The subsystem and its tests use these shared values, separately from the real drivetrain's configuration.

Run its automated checks without connecting a robot:

```powershell
.\gradlew.bat test --tests frc.robot.subsystems.SimTalonFXDrivetrainTest
```

### Watch and control the simulation in Elastic

Use [Elastic 2026.1.2](https://github.com/Gold872/elastic-dashboard/releases/tag/v2026.1.2)
with this project's alpha-8 compatibility topics. The `Dashboard` helper publishes the legacy
field/wheel topics and uses WPILib's Selectable compatibility bridge for the mode dropdown.
No Elastic robot-side library is needed.

1. In VS Code, choose **Terminal > Run Task > Swerve simulation for Elastic**. Alternatively, with Java 25 configured, run:

   ```powershell
   .\gradlew.bat run -PswerveSim
   ```

   This starts only the standalone drivetrain and the WPILib simulation GUI.
   The simulation switch does not change the deployed robot entry point.
2. Open Elastic. In **Settings**, select **localhost** as the connection address. Look for the green **Network Tables: Connected** indicator.
3. Choose **File > Open Layout** (Ctrl+O) and open [elastic-swerve-sim.json](elastic-swerve-sim.json) from this repository. A connection alone does not create widgets: the default empty tabs stay empty until you open a layout or add widgets.
4. In the layout's **Control Mode** dropdown, choose **Auto demo**, **Controller**, or **Stopped**. It starts stopped.
5. In the **WPILib simulation GUI**, select **Teleoperated** under Robot State to enable the simulation. Elastic does not enable the robot.
   - **Auto demo:** repeats forward, backward, sideways left/right, counterclockwise/clockwise, and stop, each for three seconds.
   - **Controller:** plug in an Xbox controller and assign it to **joystick 0** in the simulation GUI. Left stick translates; right stick turns. Movement is robot-relative. A missing controller stops output rather than falling back to the demo.
6. Watch the field position, wheel directions/speeds, status, and connection indicators. The wheel widget uses radians and meters per second.

Choose **Stopped** or disable in the simulation GUI to stop requests. Close the simulation GUI or terminate its VS Code task to end the process. Use **File > Save** in Elastic to remember the layout. Demo speed, phase duration, controller deadband, and turning limits live in `Constants.SimDriveConstants`.

If Elastic is connected but blank, reopen the layout. If disconnected, make sure the simulation task is still running and no second robot simulation is using localhost port 5810.

## A 10-minute group walkthrough

1. **Find the input:** Locate `getLeftY()`, `getLeftX()`, and `getRightX()` in the drive bindings.
2. **Predict the request:** With `MaxSpeed = 1 m/s` and `getLeftY() = -0.5`, the forward request is **+0.5 m/s before smoothing and deadband**.
3. **Follow the command:** Find `setDefaultCommand()` and the drivetrain's `applyRequest()`.
4. **Find the heartbeat:** Locate `CommandScheduler.getInstance().run()` in the robot lifecycle.
5. **Think like an engineer:** What should happen when the sticks are released? What changes if the robot is facing sideways?

## Next challenge: shooting

Hold the **right bumper** to run the targeting command. It repeatedly:

1. Reads the robot's estimated position, heading, and speed.
2. Predicts how far the robot will move during a short delay.
3. Finds the direction and distance to its alliance's hub.
4. Looks up a flywheel speed and attempts to compensate for robot motion.
5. Requests turret, flywheel, and hood settings while running the feed rollers.

Releasing the button stops the feed rollers, flywheel, and turret motor.

### The math has a purpose

- **Distance and direction:** A vector is an arrow from the robot to the target.
- **RPM:** Revolutions per minute describes how fast the flywheel spins.
- **Wheel speed:** `surface speed = (RPM / 60) * pi * wheel diameter`.
- **Launch angle:** Sine and cosine split speed into upward and horizontal parts.
- **Feedback control:** The motor controller compares requested speed with measured speed and adjusts its output.
- **Interpolation:** Between two distance/RPM samples, the lookup table fills in a value between them.

These are models, not guarantees. Wheel slip, ball compression, mechanism alignment, and measurement errors mean the robot still needs careful testing.

Try this on paper: the shot table requests 2,600 RPM at 55 inches and 2,806 RPM at 76 inches.
Halfway between those distances, interpolation gives **2,703 RPM**.

## Build the project

Use **WPILib 2027.0.0-alpha-8**, **Java 25**, and this repository's **Gradle 9.4.1** wrapper.
Use matching tools from the [alpha-8 release](https://github.com/wpilibsuite/allwpilib/releases/tag/v2027.0.0-alpha-8);
stable installation guides may still describe the 2026 toolchain.
The repository name and 2026 field/mechanism data are retained intentionally.

From PowerShell in the repository folder:

```powershell
.\gradlew.bat build
```

This compiles, runs the available tests, and packages the code; it does **not** deploy it to the robot.

Check `java -version` and `.\gradlew.bat --version` when troubleshooting version mismatches.
The local VS Code tasks can find Java 25 installed under `%LOCALAPPDATA%\RobotCodeTools`.
Otherwise, configure `JAVA_HOME` to a Java 25 installation.
The lesson Gists include a separate Gradle init-script helper for running their console examples;
they do not alter the production build or deploy code.

## Learn safely

**This is working development code, not a promise that every mechanism is competition-ready.**
The targeting command currently feeds without waiting for shooter readiness, and startup encoder zeros assume particular physical mechanism positions.

- Start with reading code, paper calculations, and mentor-supervised simulation.
- Before enabling hardware, have a mentor verify calibration, travel limits, and a clear test area.
- Keep hands away from rollers, wheels, and moving mechanisms. Know how to disable the robot.
- Change one thing at a time, record the result, and never raise motor limits just to "see what happens."

Good next projects: graph joystick input versus requested speed, explore drive smoothing in simulation, then graph the shot table or design a shooter-ready check with a mentor.

## 2027 migration and hardware readiness

The [training fork's `prepare/2027` branch](https://github.com/IEvangelist/2026-robot-code/tree/prepare/2027)
targets **WPILib 2027.0.0-alpha-8**, Java 25, and SystemCore deployment.
Use that branch for the weekly lessons; the upstream default branch is a different software baseline.

The migration uses `org.wpilib` packages, renamed kinematics APIs, Telemetry/Tunables/Selectable,
and compatible command framework v2 dependencies. Keeping v2 avoids an unrelated rewrite to commands v3.
The old roboRIO servo API is replaced by Expansion Hub servos.

Vendor dependencies are pinned to:

- **CTRE Phoenix 6: 26.70.0-alpha-2.** Real firmware must be compatible with the selected vendor release.
- **PhotonVision: `dev-v2027.0.0-alpha-2-89-g4b8d9b50`.** This is a development build targeting alpha-8,
  not a stable release; do not silently replace it with an incompatible 2026 or earlier-alpha library.
- Matching **CommandsV2, AprilTag, and CameraServer** metadata.

**Real startup is intentionally blocked until physical mappings are verified.**
`SystemCoreHardware` checks the unconfigured SystemCore CAN port and Expansion Hub USB/hood servo
channels instead of guessing. Desktop simulation uses explicitly simulation-only substitutes.
Successful compilation or simulation does not establish hardware readiness.

Before enabling hardware:

- [ ] Verify SystemCore image, Driver Station, Phoenix firmware, and vendor compatibility against
  the selected [alpha-8 release](https://github.com/wpilibsuite/allwpilib/releases/tag/v2027.0.0-alpha-8).
- [ ] Confirm CAN connections, the named drivetrain bus, Expansion Hub USB connection, and servo channels.
- [ ] Measure gearing and verify encoder zeroing, mechanism limits, hood travel, and safe holding/braking.
- [ ] Check cameras, timestamps, heading/alliance references, and driver input behavior.
- [ ] Test a slow drive-only configuration first, then add mechanisms one at a time with mentor supervision.

The 2026 field landmarks and autonomous routines are historical configuration, not a prediction of
the next game. Alpha APIs can change again; revalidate the build, simulator, dashboard, and lesson
examples before changing the pinned toolchain.
