# Pipelines

A Pipeline is a series of movements and actions that the robot sequentially executes while an autonomous opmode is running. Like Config, Pipelines can be created using the Pipeline Builder.

## Builder

The Pipeline Builder follows the same format as the Config Builder, enabling programmers to add an arbitrary number of actions and movements to their programs.

```
// Example Pipeline
//
// This pipeline showcases all of the options currently available to the builder

Pipeline pipeline = new Pipeline.builder(manager)
    .addContinuousAction(new LiftPositionAction(manager)
    .addAction(new ToggleClawAction(manager))
    .addAction(new SetLiftHeightAction(manager, 3000)
    .addLinearPath(
        PrecisionMode.HIGH,
        new TrapezoidalMotionProfile(250, 1000),
        new Position(-590, 100, 0),
        new Position(-590, 1360, Math.PI/3)
    )
    .addAction(new ToggleClawAction(manager))
    .addAction(new SetLiftHeightAction(manager, 0)
    .addCurvedPath(
        new Position(-590, 1360, 0),
        new Position(120, 1400, 0),
        new Position(130, 2100, 0),
        new Position(-510, 1900, 0)
    )
    .build();

pipeline.execute();
```



