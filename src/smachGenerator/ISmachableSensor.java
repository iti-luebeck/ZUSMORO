package smachGenerator;

import javax.xml.bind.annotation.XmlSeeAlso;

import robots.beep.BeepColorSensor;
import robots.beep.BeepIRSensor;

import model.bool.Variable.Operator;

@XmlSeeAlso({ BeepIRSensor.class, BeepColorSensor.class })
public interface ISmachableSensor {

	public abstract String getTopic();

	public abstract String getName();

	public abstract String getObejctInMessage();

	/**
	 * Returns the type of the topic in ros conventions.
	 * <p>
	 * e.g. If Int23 are transmitted at this topic the return value of this
	 * function would be <code>sdt_msgs.Int32._Type</code>. Equaling the String <code>"sdt_msgs/Int32"</code> (June, 2013)
	 * 
	 * @return Type of the topic
	 */
	public abstract String getTopicType();

	public abstract String getTransitionCondition(Operator op, int compVal);

}
