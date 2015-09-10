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
package robots.beep;

import java.util.HashSet;

import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;

import model.bool.Variable.Operator;
import smachGenerator.ISmachableSensor;

@XmlAccessorType(XmlAccessType.FIELD)
public class BeepSensorTimer implements ISmachableSensor {

	private final String name;

	public BeepSensorTimer(String name) {
		this.name = name;
	}
	
	public BeepSensorTimer(){
		this.name = null;
	}

	@Override
	public String getName() {
		return name;
	}

	@Override
	public HashSet<String> getImports() {
		HashSet<String> imports = new HashSet<>();
		imports.add("import time");
		return imports;
	}

	@Override
	public String getCallback() {
		return "";
	}

	@Override
	public String getSubscriberSetup() {
		return "";
	}

	@Override
	public String getValueIdentifier() {
		return "t_" + name;
	}

	@Override
	/**
	 * Hacked: 
	 * Two statements (second: 2 tabs)
	 * Second statement resets timer!
	 */
	public String getGlobalIdentifier() {
		return getValueIdentifier() + "\n\t\t" + getIdentifierInit();
	}

	@Override
	public String getIdentifierInit() {
		return getValueIdentifier() + " = time.time()";
	}

	@Override
	public String getTopicType() {
		return std_msgs.Float32._TYPE;
	}

	@Override
	public String getTransitionCondition(String op, int compVal) {
		return "time.time()-" + getValueIdentifier() + op + (float)compVal / 1000;
	}

	@Override
	public String[] onShutDown() {
		return new String[0];
	}

}
