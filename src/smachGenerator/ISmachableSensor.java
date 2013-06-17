package smachGenerator;

import javax.xml.bind.annotation.XmlSeeAlso;

import robots.beep.BeepColorSensor;
import robots.beep.BeepIRSensor;

import model.bool.Variable.Operator;

@XmlSeeAlso({BeepIRSensor.class, BeepColorSensor.class})
public interface ISmachableSensor {
	
	public abstract String getTopic();
	public abstract String getName();
	public abstract String getObejctInMessage();
	public abstract String getTopicType();
	public abstract String getTopicPackage();
	public abstract String getTransitionCondition(Operator op, int compVal);
	
}
