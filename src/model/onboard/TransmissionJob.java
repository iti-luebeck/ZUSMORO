package model.onboard;

public interface TransmissionJob {

	/**
	 * @param lastMsgRec
	 *            Last Message received from epuck
	 * @param lastMsgSent
	 *            Last Message sent from program
	 * @return unit recieved equals unit sent
	 * 
	 *         Method checking if the last information unit (e.g. a state
	 *         including transitions, name, actions etc.) was transmitted
	 *         correctly. If not, a point of restart is set.
	 */
	boolean check(String lastMsgRec, String lastMsgSent);

	boolean isComplete();

	double completionStatus();

	void removeFirst();

	String getFirst();

}
