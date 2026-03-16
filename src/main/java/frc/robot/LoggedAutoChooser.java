package frc.robot;

import static edu.wpi.first.wpilibj.Alert.AlertType.*;

import choreo.auto.AutoRoutine;
import choreo.util.ChoreoAlert;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayEntry;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;

public class LoggedAutoChooser extends LoggedNetworkInput {
	public static final String NONE_NAME = "__Nothing__";
	static final Alert selectedNonexistentAuto = ChoreoAlert.alert("Selected an auto that isn't an option", kError);

	public String key;
	public HashMap<String, Supplier<Command>> autoRoutines = new HashMap<>(Map.of(NONE_NAME, Commands::none));

	public StringEntry selected;
	public StringEntry active;
	public StringArrayEntry options;

	public String lastCommandName = NONE_NAME;
	public Command lastCommand = Commands.none();

	public LoggableInputs inputs = new LoggableInputs() {
		public void toLog(LogTable table) {
			table.put(key, lastCommandName);
		}

		public void fromLog(LogTable table) {
			lastCommandName = table.get(key, lastCommandName);
		}
	};

	public LoggedAutoChooser(String tableName) {
		this(tableName, NetworkTableInstance.getDefault());
	}

	public LoggedAutoChooser() {
		this("");
	}

	LoggedAutoChooser(String tableName, NetworkTableInstance ntInstance) {
		Logger.registerDashboardInput(this);
		if (tableName == null) {
			tableName = "";
		}
		key = tableName;
		String path = tableName.isEmpty() ? "" : NetworkTable.normalizeKey(tableName, true);
		NetworkTable table = ntInstance.getTable(path + "/AutoChooser");

		selected = table.getStringTopic("selected").getEntry("");
		selected.set(NONE_NAME);

		table.getStringTopic(".type").publish().set("String Chooser");
		table.getStringTopic("default").publish().set(NONE_NAME);

		active = table.getStringTopic("active").getEntry(NONE_NAME);
		active.set(NONE_NAME);

		var defaultOptions = autoRoutines.keySet().toArray(new String[0]);
		options = table.getStringArrayTopic("options").getEntry(defaultOptions);
		options.set(defaultOptions);
		periodic();
	}

	public void periodic() {
		if (!Logger.hasReplaySource()) {
			if (DriverStation.isDisabled()
					&& DriverStation.isDSAttached()
					&& DriverStation.getAlliance().isPresent()) {
				String selectStr = selected.get();
				if (selectStr.equals(lastCommandName)) {
					Logger.processInputs(prefix + " " + key, inputs);
					return;
				}
				if (!autoRoutines.containsKey(selectStr) && !selectStr.equals(NONE_NAME)) {
					selected.set(NONE_NAME);
					selectStr = NONE_NAME;
					selectedNonexistentAuto.set(true);
				} else {
					selectedNonexistentAuto.set(false);
				}
				lastCommandName = selectStr;
			}
		}
		lastCommand = autoRoutines.get(lastCommandName).get().withName(lastCommandName);
		active.set(lastCommandName);
		Logger.processInputs(prefix + " " + key, inputs);
	}

	public void addRoutine(String name, Supplier<AutoRoutine> generator) {
		autoRoutines.put(name, () -> generator.get().cmd());
		options.set(autoRoutines.keySet().toArray(new String[0]));
	}

	public void addCmd(String name, Supplier<Command> generator) {
		autoRoutines.put(name, generator);
		options.set(autoRoutines.keySet().toArray(new String[0]));
	}

	public Command selectedCommandScheduler() {
		return Commands.defer(() -> lastCommand.asProxy(), Set.of());
	}

	public Command selectedCommand() {
		return lastCommand;
	}
}
