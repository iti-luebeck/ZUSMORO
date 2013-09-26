package smachGenerator;

import java.util.HashSet;

import javax.xml.bind.annotation.XmlSeeAlso;

import robots.beep.BeepSensorColor;
import robots.beep.BeepSensorIR;

import model.bool.Variable.Operator;

@XmlSeeAlso({ BeepSensorIR.class, BeepSensorColor.class })
public interface ISmachableSensor {

	/**
	 * @return the name of the sensor
	 */
	public abstract String getName();

	/**
	 * 
	 * @return a HashSet of all Imports that are needed for this
	 *         {@link ISmachableSensor}.
	 */
	public abstract HashSet<String> getImports();

	/**
	 * 
	 * Creates the callback function for this sensor. All calculations (if
	 * necessary) to be able to compare values for later transitions have to be
	 * done here and the result has to be stored in the
	 * <code>ValueIdentifier</code>. This <code>ValueIdentifier</code> has to be
	 * the same that is initialized by <code>getIdentifierInit()</code> and
	 * <code>getGlobalIdentifier</code>.
	 * 
	 * @return the callback for this sensor.
	 */
	public abstract String getCallback();

	/**
	 * Creates the Subscriber Setup statement. The callback for this Subscriber
	 * will be the one that is created by <code>getCallback()</code>.
	 * 
	 * @return Subscriber initialization
	 */
	public abstract String getSubscriberSetup();

	/**
	 * @return the identifier that stores the current value of this sensor.
	 */
	public abstract String getValueIdentifier();

	/**
	 * Returns the identifier of the globally defined variable for this sensor.
	 * <p>
	 * NOTE: this means this might return the name of an Array or anything else.
	 * To receive the identifier that stores the current value of this sensor,
	 * use <code>getValueIdentifier()</code>!</p>
	 * <p><i>May not be empty!</i></p>
	 * 
	 * @return globally identifier name.
	 */
	public abstract String getGlobalIdentifier();

	/**
	 * <p><i>May not be empty!</i></p>
	 * 
	 * @return statements to define and initialize the value the callback stores
	 *         values in.
	 */
	public abstract String getIdentifierInit();

	/**
	 * Returns the type of the topic in ros conventions.
	 * <p>
	 * e.g. If Int23 are transmitted at this topic the return value of this
	 * function would be <code>sdt_msgs.Int32._Type</code>. Equaling the String
	 * <code>"sdt_msgs/Int32"</code> (June, 2013)
	 * 
	 * <p><i>May not be empty!</i></p>
	 * 
	 * @return Type of the topic
	 */
	public abstract String getTopicType();

	/**
	 * Returns the condition comparing the current sensor value with the compare
	 * value.
	 * 
	 * @param op
	 *            Operator to compare the values.
	 * @param compVal
	 *            value to compare the current sensor value with.
	 * @return a condition representing the comparison of the sensor value with
	 *         the compare value.
	 */
	public abstract String getTransitionCondition(Operator op, int compVal);

	/**
	 * Returns a number of commands that shall be executed before the {@link SmachAutomat} is shut down.
	 * Mainly this will be publishing some last messages for the sensor to deactivate etc.
	 * @return Some commands to shutdown this sensor
	 */
	public abstract String[] onShutDown();
	
}
