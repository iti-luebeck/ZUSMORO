package smachGenerator;

import java.util.LinkedList;

public interface ISmachableGuard{

	/**
	 * Returns the name of all sensors a value shall be compared with. Returns
	 * an empty List, if there is no condition for this transition.
	 * 
	 * @return List of Sensors
	 */
	public abstract LinkedList<String> getSensorNames();

	/**
	 * Returns a List of {@link Operator}s. Each {@link Operator} belongs to the
	 * corresponding sensor in the getSensorNames list.
	 * 
	 * @return List of Strings, representing the relation between
	 *         sensor and compare value. Choose one of <, <=, ==, >=, >, !=
	 */
	public abstract LinkedList<String> getOperators();

	/**
	 * Returns a List of {@link Integer}s. Each of these compare values shall be
	 * used to be compared with the sensor from the getSensorNames List and the
	 * Operator from the getOperators List
	 * 
	 * @return List of values to compare with
	 */
	public abstract LinkedList<Integer> getCompValues();

}
