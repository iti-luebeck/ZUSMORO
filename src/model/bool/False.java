package model.bool;

public class False extends BooleanExpression {
	@Override
	public boolean eval() {
		return false;
	}

	@Override
	public String toString() {
		return "false";
	}

	@Override
	public boolean equals(Object obj) {
		return (obj != null && obj instanceof False);
	}
}
