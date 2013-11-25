package robots.beep;

import java.util.HashSet;

import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;

import model.bool.Variable.Operator;
import smachGenerator.ISmachableSensor;

@XmlAccessorType(XmlAccessType.FIELD)
public class BeepSensorTimer implements ISmachableSensor {

	private final String name;

	public BeepSensorTimer(String name) {
		this.name = name;
	}
	
	public BeepSensorTimer(){
		this.name = null;
	}

	@Override
	public String getName() {
		return name;
	}

	@Override
	public HashSet<String> getImports() {
		HashSet<String> imports = new HashSet<>();
		imports.add("import time");
		return imports;
	}

	@Override
	public String getCallback() {
		return "";
	}

	@Override
	public String getSubscriberSetup() {
		return "";
	}

	@Override
	public String getValueIdentifier() {
		return "t_" + name;
	}

	@Override
	/**
	 * Hacked: 
	 * Two statements (second: 2 tabs)
	 * Second statement resets timer!
	 */
	public String getGlobalIdentifier() {
		return getValueIdentifier() + "\n\t\t" + getIdentifierInit();
	}

	@Override
	public String getIdentifierInit() {
		return getValueIdentifier() + " = time.time()";
	}

	@Override
	public String getTopicType() {
		return std_msgs.Float32._TYPE;
	}

	@Override
	public String getTransitionCondition(String op, int compVal) {
		return "time.time()-" + getValueIdentifier() + op + (float)compVal / 1000;
	}

	@Override
	public String[] onShutDown() {
		return new String[0];
	}

}
