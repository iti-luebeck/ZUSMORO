package model.bool;

public class Neg extends BooleanExpression {

	private BooleanExpression arg;

	public Neg(BooleanExpression arg) {
		if (arg == null) {
			throw new IllegalArgumentException("Boolean operants may not be null");
		}
		this.arg = arg;
	}

	@Override
	public boolean eval() {
		return !arg.eval();
	}

	@Override
	public String toString() {
		return "<Neg>\r\n"+arg+"\r\n</Neg>";
	}

	@Override
	public boolean equals(Object obj) {
		return obj != null && obj instanceof Neg && this.arg.equals(((Neg) obj).arg);
	}
}
