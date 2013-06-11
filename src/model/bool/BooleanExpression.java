package model.bool;

import java.io.BufferedReader;

import model.bool.Variable.Operator;

public abstract class BooleanExpression {

	public final static BooleanExpression TRUE = new True();
	public final static BooleanExpression FALSE = new False();
	public abstract boolean eval();
	@Override
	public abstract String toString();
	@Override
	public abstract boolean equals(Object obj);


	public static BooleanExpression parseExpr(BufferedReader reader) throws Exception {
		BooleanExpression expr = null;
		String line = reader.readLine().trim();
		if (line.equalsIgnoreCase("<And>")) {
			And and = new And(BooleanExpression.parseExpr(reader), BooleanExpression.parseExpr(reader));
			reader.readLine();
			expr = and;
		} else if (line.equalsIgnoreCase("<Or>")) {
			Or or = new Or(BooleanExpression.parseExpr(reader), BooleanExpression.parseExpr(reader));
			reader.readLine();
			expr = or;
		} else if (line.equalsIgnoreCase("<Neg>")) {
			Neg neg = new Neg(BooleanExpression.parseExpr(reader));
			reader.readLine();
			expr = neg;
		} else if (line.equalsIgnoreCase("true")) {//INFO Changed from <true> to true
			expr = TRUE;
		} else if (line.equalsIgnoreCase("<false>")) {
			expr = FALSE;
		} else if (line.equalsIgnoreCase("<Variable>")) {
			Variable var = new Variable(reader.readLine(), Operator.valueOf(reader.readLine()), Integer.parseInt(reader.readLine()));
			reader.readLine();
			expr = var;
		} else if (line.equalsIgnoreCase("<HugeAnd>")) {
			HugeAnd hugeAnd = new HugeAnd();
			reader.mark(2000);
			while (!(line = reader.readLine().trim()).equalsIgnoreCase("</HugeAnd>")) {
				reader.reset();
				BooleanExpression nextExpr = BooleanExpression.parseExpr(reader);
				if (nextExpr != null) {
					hugeAnd.addOperand(nextExpr);
				}
				reader.mark(2000);
			}
			expr = hugeAnd;
		}
		return expr;
	}
}
