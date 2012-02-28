package model.bool;

public class True extends BooleanExpression {
	@Override
	public boolean eval() {
		return true;
	}

	@Override
	public String toString() {
		return "<true>";
	}

	@Override
	public boolean equals(Object obj) {
		return (obj != null && obj instanceof True);
	}
}
