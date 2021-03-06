/*
 * Copyright (c) 2015, Institute of Computer Engineering, University of L�beck
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * 
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
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
