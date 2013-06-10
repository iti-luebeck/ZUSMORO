package model.bool;

import java.util.LinkedList;

import model.bool.Variable.Operator;
import smachGenerator.ISmachableGuard;

public class True extends BooleanExpression implements ISmachableGuard{
	@Override
	public boolean eval() {
		return true;
	}

	@Override
	public String toString() {
		return "true";
	}

	@Override
	public boolean equals(Object obj) {
		return (obj != null && obj instanceof True);
	}

	@Override
	public LinkedList<String> getSensorNames() {
		return null;
	}

	@Override
	public LinkedList<Operator> getOperators() {
		return null;
	}

	@Override
	public LinkedList<Integer> getCompValues() {
		return null;
	}
}
