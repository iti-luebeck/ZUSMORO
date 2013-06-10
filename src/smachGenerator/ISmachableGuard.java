package smachGenerator;

import java.util.LinkedList;

import model.bool.Variable.Operator;


public interface ISmachableGuard {
	
	public abstract LinkedList<String> getSensorNames();
	public abstract LinkedList<Operator> getOperators();
	public abstract LinkedList<Integer> getCompValues();

}
