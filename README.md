# Javabot-2025 - FRC Robot Code

This is the Java-based robot code refactored from the 2025 FRC season. This project uses WPILib's Command-Based framework with CTRE Phoenix 6 for drivetrain control and Limelight cameras for vision-based localization.


## Quick Start

### Prerequisites
- [WPILib 2025](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)
- [FRC VS Code](https://github.com/wpilibsuite/allwpilib/releases)

### Building and Deploying
1. Open the project in VS Code with WPILib extensions installed
2. Press `Ctrl+Shift+P` and run `WPILib: Build Robot Code`
3. Connect to the robot via USB or WiFi
4. Press `Ctrl+Shift+P` and run `WPILib: Deploy Robot Code`

## Project Overview

This robot features:
- **Swerve Drive**: 4-wheel swerve drivetrain using CTRE Phoenix 6
- **Vision System**: Multiple Limelight cameras for AprilTag-based localization
- **Mechanisms**: Arm, elevator, and wheel intake systems
- **Autonomous**: PathPlanner integration for trajectory-based autonomous routines
- **LED Control**: Custom LED patterns for robot status indication

## Folder Structure

### Root Directory

```
Javabot-2025/
├── src/                          # Source code
├── build/                        # Compiled output (auto-generated)
├── gradle/                       # Gradle wrapper files
├── vendordeps/                   # Vendor library dependencies (CTRE, REV, PathPlanner, etc.)
├── ButtonBoxArduinoCode/         # Arduino code for custom button boxes
├── build.gradle                  # Gradle build configuration
├── settings.gradle               # Gradle settings
└── README.md                     # This file
```

### Source Code (`src/main/`)

#### `src/main/java/frc/robot/`
The main robot code package containing all Java source files.

**Core Files:**
- **`Main.java`** - Entry point for the robot program. You rarely need to modify this.
- **`Robot.java`** - Main robot class that manages the robot lifecycle (init, periodic, autonomous, teleop). Handles mode transitions and scheduler execution.
- **`RobotContainer.java`** - The "heart" of the robot. Declares all subsystems, commands, button bindings, and autonomous routines. This is where you wire everything together.
- **`Constants.java`** - Contains all robot-wide constants (motor IDs, PID values, physical dimensions, etc.). Organized into nested classes by subsystem.
- **`Telemetry.java`** - Handles logging and telemetry data for the swerve drive subsystem (Don't worry about this file)

#### `src/main/java/frc/robot/commands/`
Command classes that define robot behaviors. Commands are the "verbs" of your robot (drive, rotate, shoot, etc.).

**Structure:**
```
commands/
├── drive/                        # Drivetrain-specific commands
│   ├── CenterToTargetCommand.java
│   └── [other drive commands]
├── JoystickCommandsFactory.java  # Creates commands for joystick control
├── RobotCommandsFactory.java     # Factory for common robot commands
└── TrajectoryCommandsFactory.java # Creates autonomous trajectory commands
```

**What's Here:**
- **Drive Commands**: Commands that control the drivetrain (align to target, drive to position, etc.)
- **Factory Classes**: Centralized creation of reusable commands to avoid duplication

#### `src/main/java/frc/robot/subsystems/`
Subsystem classes representing physical robot systems. Subsystems are the "nouns" of your robot.

**Structure:**
```
subsystems/
├── drive/                        # Legacy swerve implementation (if present)
│   ├── DriveSubsystem.java
│   └── SwerveModule.java
├── phoenixDrive/                 # CTRE Phoenix 6 swerve implementation
│   └── [Phoenix 6 drive code]
├── mechanisms/                   # Game-specific mechanism abstractions
│   ├── Arm/                      # Arm mechanism implementation
│   ├── Elevator/                 # Elevator mechanism implementation
│   ├── Wheel/                    # Wheel intake mechanism
│   └── Mechanism.java            # Base class for mechanisms
├── ArmSubsystem.java             # Arm subsystem (rotational joint)
├── ElevatorSubsystem.java        # Elevator subsystem (linear extension)
├── LEDSubsystem.java             # LED control (older version)
├── LEDSubsystemP6.java           # LED control using Phoenix 6
└── LimelightVisionSubsystem.java # Manages Limelight cameras for vision localization
```

**Mechanisms:**  
 For simplicity, we put the 3 most common mechanisms in the 'mechanism' folder, allowing them to be easily reused, and keeping the detailed implementation of how to 'talk' to the motor controller away from the more advance robot control logic.  

Each mechanism is programmed using inheritance. The top layer of the inheritance is the `Mechanism.java.`. This defines characteristics that _all_ mechanisms share. Then comes the 3 main mechanisms - `ArmMechanism.java`, `ElevatorMechanism.java`, and `WheelMechanism.java`. Each of these defines how to interact with an elevator, an arm, and a wheel (wheel used for shooter wheels, kickup wheels, intake wheels, etc). Finally, the last layer of the inheritance is the specific motor controller implementation, either CTRE or SparkMax (i.e. `WheelMechanismCTRE.java` and `WheelMechanismSparkMax.java`)

The subsystems that use these mechanisms, I.E. `ArmSubsystem.java`, simply initialize the mechanism, and add any commands that are specific to this year's robot. For instance, `ArmMechanism.java` provides commands to move the arm to a specific point, in Radians. `ArmSubsystem.java` would have robot-specific commands that move the arm to specific robot positions, I.E. score, intake, etc. 

### Key Subsystems:
- **DriveSubsystem** (phoenixDrive): Controls the swerve drive using CTRE's Phoenix 6 API
- **LimelightVisionSubsystem**: Manages multiple Limelight cameras, handles AprilTag filtering, and provides pose estimates to the drivetrain
- **ArmSubsystem**: Controls the arm mechanism (angular position)
- **ElevatorSubsystem**: Controls the elevator mechanism (linear extension)
- **LEDSubsystemP6**: Controls addressable LEDs for robot status indication (P6 for Pheonix 6 library)
  
### Other Robot Code Files:

#### `src/main/java/frc/robot/vision/`
Vision system classes for cameras and image processing.

```
vision/
├── Limelight.java                # Wrapper class for Limelight camera
└── PiCamera.java                 # Wrapper for Raspberry Pi camera (if used)
```

**What's Here:**
- **Limelight.java**: Provides a clean interface to Limelight features (AprilTags, LED control, pipeline switching, pose estimation). Uses `LimelightHelpers` internally.

#### `src/main/java/frc/robot/utils/`
Utility classes and helper functions used throughout the codebase.

```
utils/
├── LimelightHelpers.java         # Official Limelight helper library (v1.11)
├── MathUtils.java                # Mathematical utility functions
├── SparkMaxUtils.java            # REV SparkMax motor controller utilities
├── TalonUtils.java               # CTRE Talon motor controller utilities
├── Types.java                    # Custom type definitions and data classes
└── Elastic.java                  # Elastic dashboard integration
```

**What's Here:**
- **LimelightHelpers**: Official Limelight library with comprehensive API for all Limelight features
- **Types**: Defines reusable types like `PidConstants`, `FeedForwardConstants`, `MotionProfileConstants`, `LEDState`, etc.
- **Utility Classes**: Helper methods for motors, math operations, and framework integrations

#### `src/main/java/frc/robot/field/`
Field-related constants and utilities.

```
field/
└── ScoringPositions.java         # Defines scoring positions on the field
```

**What's Here:**
- Field geometry and coordinate system definitions
- Scoring positions for autonomous and assisted teleop
- AprilTag locations and field-specific constants

#### `src/main/deploy/`
Files that get deployed to the roboRIO alongside the robot code.

```
deploy/
├── pathplanner/                  # PathPlanner trajectory files
│   ├── autos/                    # Autonomous routine configurations
│   ├── paths/                    # Individual path files (.path)
│   ├── navgrid.json              # Navigation grid for pathfinding
│   └── settings.json             # PathPlanner settings
└── example.txt                   # Example deployment file
```

**What's Here:**
- **PathPlanner Files**: Autonomous paths and routines created with the PathPlanner GUI
- **Configuration Files**: Any files the robot needs at runtime (configs, calibration data, etc.)



## Development Resources

### FRC Documentation
- [WPILib Documentation](https://docs.wpilib.org/en/stable/)
- [Command-Based Programming](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html)
- [CTRE Phoenix 6 Documentation](https://v6.docs.ctr-electronics.com/en/stable/)
- [Limelight Documentation](https://docs.limelightvision.io/en/latest/)
- [PathPlanner Documentation](https://pathplanner.dev/home.html)

### Key Concepts

#### Command-Based Programming
This robot uses WPILib's command-based framework:
- **Subsystems**: Represent hardware systems (drivetrain, arm, etc.)
- **Commands**: Define actions/behaviors that use subsystems
- **Triggers**: Bind commands to controller buttons or conditions
- **Scheduler**: Runs commands automatically based on requirements and triggers


The drivetrain uses odometry (wheel encoders + gyro + vision) to track robot position on the field.

#### Vision Pipeline
1. Limelight cameras detect AprilTags
2. Tags are filtered based on alliance and field position
3. MegaTag algorithms calculate robot pose
4. Pose estimates are fused with drivetrain odometry
5. Result: Accurate field-relative localization

