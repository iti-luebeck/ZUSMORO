package tests;

import static org.junit.Assert.assertTrue;

import java.io.BufferedReader;
import java.io.StringReader;

import model.bool.And;
import model.bool.BooleanExpression;
import model.bool.HugeAnd;
import model.bool.Neg;
import model.bool.Or;
import model.bool.Variable;
import model.bool.Variable.Operator;

import org.junit.Test;

public class BooleanExpressionTest {

	@Test
	public void testParseExpr() {
		Variable var1 = new Variable("IR0", Operator.EQUAL, 1000);
		Variable var2 = new Variable("IR4", Operator.SMALLER, 100);
		Variable var3 = new Variable("IR5", Operator.NOT_EQUAL, 999);
		And and = new And(var1,var2);
		Or or = new Or(var2,var3);
		HugeAnd hAnd = new HugeAnd();
		hAnd.addOperand(and);
		hAnd.addOperand(or);
		hAnd.addOperand(var1);
		hAnd.addOperand(BooleanExpression.TRUE);
		Neg neg = new Neg(hAnd);
		String out = neg.toString();
		System.out.println(out);
		BufferedReader reader = new BufferedReader(new StringReader(out));
		BooleanExpression expr = null;
		try {
			expr = BooleanExpression.parseExpr(reader);
		} catch (Exception e) {
			e.printStackTrace();
		}
		assertTrue(neg.equals(expr));
	}

}
