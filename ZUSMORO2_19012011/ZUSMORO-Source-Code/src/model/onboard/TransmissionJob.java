package model.onboard;

public interface TransmissionJob {


	boolean check(String lastMsgRec, String lastMsgSent);
	boolean isComplete();
	double completionStatus();
	void removeFirst();
	String getFirst();

}
