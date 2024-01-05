package frc.robot.constants.enums;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.LedConfs.LedSections;
import frc.robot.subsystems.LedController;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class AutonomousAction {
  private final String name;
  private final CommandBase underlying;

  private CommandBase command;

  private static final List<AutonomousAction> actionList = new ArrayList<AutonomousAction>();
  private static final Map<String, Command> eventMap = new HashMap<String, Command>();

  AutonomousAction(String name, CommandBase underlying) {
    this.name = name;
    this.underlying = underlying;
  }

  private static void register(AutonomousAction action) {
    actionList.add(action);
    eventMap.put(action.name, action.command);
  }

  public static void registerAutonomousAction(String name, double waitSeconds) {
    AutonomousAction action = new AutonomousAction(name, new WaitCommand(waitSeconds));
    action.command = action.underlying;
    register(action);
  }

  public static void registerAutonomousAction(
      String name, LedController controller, LedColors color
  ) {
    AutonomousAction action = new AutonomousAction(
        name, new InstantCommand(() -> controller.setSolidColor(color, LedSections.ALL))
    );
    action.command = action.underlying;
    register(action);
  }

  public static void registerAutonomousAction(String name, String message) {
    AutonomousAction action = new AutonomousAction(name, new PrintCommand(message));
    // unclear if we can just skip this step or not
    action.command = new InstantCommand(() -> action.underlying.schedule());
    register(action);
  }

  public static void registerAutonomousAction(String name, CommandBase command) {
    AutonomousAction action = new AutonomousAction(name, command);
    action.command = new InstantCommand(() -> action.underlying.schedule());
    register(action);
  }

  public static Map<String, Command> getEventMap() {
    return eventMap;
  }
}
