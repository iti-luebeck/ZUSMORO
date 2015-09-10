/*
 * Copyright (c) 2015, Institute of Computer Engineering, University of Lübeck
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
