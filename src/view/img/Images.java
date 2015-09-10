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
package view.img;

import javax.swing.ImageIcon;

public class Images {

	public static final ImageIcon STATETOOL = new ImageIcon(Images.class.getResource("stateTool.png"));
	public static final ImageIcon TRANSITIONTOOL = new ImageIcon(Images.class.getResource("transition.png"));
	public static final ImageIcon MOVETOOL = new ImageIcon(Images.class.getResource("move.png"));
	public static final ImageIcon DELETETOOL = new ImageIcon(Images.class.getResource("delete.png"));
	public static final ImageIcon CONNECT = new ImageIcon(Images.class.getResource("connect.png"));
	public static final ImageIcon DISCONNECT = new ImageIcon(Images.class.getResource("disconnect.png"));
	public static final ImageIcon START = new ImageIcon(Images.class.getResource("start.png"));
	public static final ImageIcon PAUSE = new ImageIcon(Images.class.getResource("pause.png"));
	public static final ImageIcon STOP = new ImageIcon(Images.class.getResource("stop.png"));
	public static final ImageIcon TRANSMIT = new ImageIcon(Images.class.getResource("transmit.png"));
	public static final ImageIcon DEBUG = new ImageIcon(Images.class.getResource("debug.png"));
}
