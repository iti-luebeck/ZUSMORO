#include "zusmoro.h"

#define VERSION "Vzusmoro1.1.0\r"

static char in[30];
static char out[50];


char c;
int z=0; //Zustandspointer
int Z=-1; //Anzahl Zustände
int t=0; //Transitionspointer
int T=-1; //Anzahl Transitionen
int b=0;//Bedingungspointer
int B=-1; //Anzahl Bedingungen

int timerms=0;
int timer30s=0;

int sound=0;
int debug=1;

int start=-1;
state* states=0;

guard* guards=0;

transition* transitions =0;


int j;//irgendein Zähler


void sendState(int y) {
	int z=y;
	if(y<0)
		z=-y-1;
	sprintf(out,"z%d,%d\r",z,states[z].numberOfTransitions);
	uart_send_text(out);
	if(y>=0){
	sprintf(out,"r%d,%d,%d,%d,%d,%d\r",
	        states[z].motorRoffset,
	        states[z].motorRmin,
	        states[z].motorRmax,
	        states[z].motorRKpr,
	        states[z].motorRvar,
	        states[z].motorRvalue
	       );
	uart_send_text(out);
	sprintf(out,"l%d,%d,%d,%d,%d,%d\r",
	        states[z].motorLoffset,
	        states[z].motorLmin,
	        states[z].motorLmax,
	        states[z].motorLKpr,
	        states[z].motorLvar,
	        states[z].motorLvalue
	       );
	uart_send_text(out);
	}
	sprintf(out,"L%d\r",states[z].leds);
	uart_send_text(out);
	if(y>=0)
	for (j=0;j<states[y].numberOfTransitions;j++) {
		sprintf(out,"G%d\r",states[y].transitions[j]);
		uart_send_text(out);
	}

};

void sendTransition(int y) {
	sprintf(out,"t%d,%d,%d\r",y,transitions[y].numberOfGuards,transitions[y].followState);
	uart_send_text(out);
	for (j=0;j<transitions[y].numberOfGuards;j++) {
		sprintf(out,"g%d\r",transitions[y].guards[j]);
		uart_send_text(out);
	}
};

void sendMallocError() {
	uart_send_static_text("Malloc Error!\r\n");
}

void reset() {
	e_end_agendas_processing();
	setLeds(0);
	e_set_speed_left(0);
	e_set_speed_right(0);
	timerms=0;
	timer30s=0;
	if (states != 0) {
		for (z=0;z<Z;z++)
			if (states[z].transitions !=0) {
				free(states[z].transitions);
				states[z].transitions=0;
			}
		free(states);
		states=0;
	}

	if (transitions != 0) {
		for (t=0;t<T;t++)
			if (transitions[t].guards !=0) {
				free(transitions[t].guards);
				transitions[t].guards=0;
			}
		free(transitions);
		transitions=0;
	}


	if (guards != 0) {
		free(guards);
		guards=0;
	}
}


void readInput() {
	switch (in[0]) {
	case 'Z': {// Zustandsliste prolog
			reset();
			sscanf(in,"Z%d\r",&Z);
			states=malloc(Z*sizeof(state));
			if (!states)
				sendMallocError();
			break;
		}

	case 'z': { // Zustand prolog
			int anzahlTrans=0;
			sscanf(in,"z%d,%d\r",&z,&anzahlTrans);
			states[z].numberOfTransitions=anzahlTrans;
			t=0;
			states[z].leds=0;

			states[z].motorLoffset=0;
			states[z].motorLmin=0;
			states[z].motorLmax=0;
			states[z].motorLKpr=0;
			states[z].motorLvar=0;
			states[z].motorLvalue=0;

			states[z].motorRoffset=0;
			states[z].motorRmin=0;
			states[z].motorRmax=0;
			states[z].motorRKpr=0;
			states[z].motorRvar=0;
			states[z].motorRvalue=0;


			states[z].transitions=malloc(anzahlTrans*sizeof(int));
			if (!states[z].transitions)
				sendMallocError();
			break;
		}

	case 'L': { // Zustand LEDs

			sscanf(in,"L%d\r",&states[z].leds);
			break;
		}
	case 'l': { // Zustand Motor links

			sscanf(in,"l%d,%d,%d,%d,%d,%d\r",
			       &states[z].motorLoffset,
			       &states[z].motorLmin,
			       &states[z].motorLmax,
			       &states[z].motorLKpr,
			       &states[z].motorLvar,
			       &states[z].motorLvalue
			      );
			break;
		}
	case 'r': { // Zustand Motor rechts

			sscanf(in,"r%d,%d,%d,%d,%d,%d\r",
			       &states[z].motorRoffset,
			       &states[z].motorRmin,
			       &states[z].motorRmax,
			       &states[z].motorRKpr,
			       &states[z].motorRvar,
			       &states[z].motorRvalue
			      );
			break;
		}
	case 'G': { // Transition eintragen
			sscanf(in,"G%d\r",&(states[z].transitions[t++]));
			//sendState(z);
			//TODO: Abfangen wenn zuviele Transitionen gesendet werden
			break;
		}

	case 'y': { // Zustand epilog
			sendState(z);
			uart_send_static_text("\n");
			break;
		}
	case 'Y': { // Zustandsliste epilog
			sprintf(out,"Z%d\r",Z);
			uart_send_text(out);
			for (z=0;z<Z;z++)
				sendState(z);
			uart_send_static_text("\n");
			break;
		}
	case 'B': { // Bedingungsliste prolog
			sscanf(in,"B%d\r",&B);
			guards=malloc(B*sizeof(guard));
			if (!guards)
				sendMallocError();
			break;
		}
	case 'b': { // Bedingung
			int op=0;
			int var=0;
			int k=0;
			sscanf(in,"b%d,%d,%d,%d\r",&b,&op,&var,&k);
			guards[b].op=op;
			guards[b].variable=var;
			guards[b].compValue=k;
			break;
		}


	case 'C': { // Bedingungsliste epilog
			sprintf(out,"B%d\r",B);
			uart_send_text(out);
			for (b=0;b<B;b++) {
				sprintf(out,"b%d,%d,%d,%d\r",b,guards[b].op,guards[b].variable,guards[b].compValue);
				uart_send_text(out);
			}
			uart_send_static_text("\n");
			break;
		}
	case 'A': { // Startzustand setzen
			sscanf(in,"A%d\r",&start);
			break;
		}
	case 'a': { //Startzustand abfragen
			sprintf(out,"A%d\r\n",start);
			uart_send_text(out);
			break;
		}

	case 'T': { // Transitionsliste prolog
			sscanf(in,"T%d\r",&T);
			transitions=malloc(T*sizeof(transition));
			if (!transitions)
				sendMallocError();
			break;
		}
	case 't': { // Transition prolog
			int follow=-1;
			int noG=0;

			sscanf(in,"t%d,%d,%d\r",&t,&noG,&follow);

			transitions[t].numberOfGuards=noG;
			transitions[t].followState=follow;
			b=0;
			transitions[t].guards=malloc(noG*(sizeof(int)));
			if (!transitions[t].guards)
				sendMallocError();
			break;
		}
	case 'g': { // Transition Bedingszuordnung
			sscanf(in,"g%d\r",&(transitions[t].guards[b++]));
			//TODO: Abfangen wenn zuviele Transitionen gesendet werden
			break;
		}

	case 'u': { // Transition epilog
			sendTransition(t);
			uart_send_static_text("\n");
			break;
		}
	case 'U': { // Transition epilog
			sprintf(out,"T%d\r",T);
			uart_send_text(out);
			for (t=0;t<T;t++)
				sendTransition(t);
			uart_send_static_text("\n");
			break;
		}

	case 'S': { //Start
			z=start;
			e_start_agendas_processing();
			enterState();

			break;
		}
	case 's': { //Stop

			e_end_agendas_processing();
			setLeds(0);
			e_set_speed_left(0);
			e_set_speed_right(0);
			break;
		}
	case 'd': {//Debug Mode
			sscanf(in,"d%d\r",&debug);
			break;
		}
	case 'v': {//Version abfragen
			uart_send_static_text(VERSION);
			uart_send_static_text("\n");

			break;
		}

	default:

		sprintf(out,"Unknown:%s\r",in);
		if (strlen(in)>1)
			uart_send_text(out);
		break;
	}
};



int getSelector() {
	return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;

}

void setLeds(int n) {
	int l=n;
	for (j=0;j<=7;j++) {
		e_set_led(j,l&1);
		l>>=1;
	}
	e_set_front_led(l&1);
	l>>=1;
	e_set_body_led(l&1);
	l>>=1;
	setSound(l&1);
};

void setSound(int s) {
	if (s==1) {
		if (sound==0) {
			e_init_sound();
			sound=1;
			e_play_sound(7294,3732);
		}
	} else {
		if (sound==1) {
			e_close_sound();
			sound=0;
		}
	}
};

void enterState() {
	if (z>=0) {
		/*
		e_set_speed_left(states[z].motorL);
		e_set_speed_right(states[z].motorR);
		*/
		if (!states[z].motorRKpr)
			e_set_speed_right(states[z].motorRoffset);

		if (!states[z].motorLKpr)
			e_set_speed_left(states[z].motorLoffset);


		setLeds(states[z].leds);
		timerms=0;
		timer30s=0;
		if (debug&1) {
			sendState(-z-1);
			uart_send_static_text("\n");
		}
	} else
		reset();
};

void incTimer(void) {
	if (timerms>30000) {
		timer30s++;
		timerms-=30000;
	}
	timerms++;

};

int getVar(int i) {
	int ret=-1;
	if (i>=0) {
		switch (i) {
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7: {
				ret=e_get_calibrated_prox(i);
				break;
			}
		case 8:
		case 9:
		case 10: {
				e_i2cp_enable();
				char hiByte = e_i2cp_read(0xC0,(i-8)*2);
				char loByte = e_i2cp_read(0xC0,(i-8)*2+1);
				e_i2cp_disable();
				ret=(unsigned int)loByte|((unsigned int)hiByte<<8);

				break;
			}
		case 11: {
				ret=timerms;
				break;
			}
		case 12: {
				ret=timer30s;
				break;
			}
		default:
			break;
		}
		return ret;
	} else
		//getVar(a)-getVar(b):=getVar(-100*b-a)
		return getVar((-i)%100)-getVar(-i/100);

}

int evalGuard(int n) {
	int var=getVar(guards[n].variable);
	int ret=1;
	switch (guards[n].op) {
	case 5: {
			if (var>guards[n].compValue)
				ret=0;
			break;
		}
	case 4: {
			if (var>=guards[n].compValue)
				ret=0;
			break;
		}
	case 3: {
			if (var==guards[n].compValue)
				ret=0;
			break;
		}
	case 2: {
			if (var!=guards[n].compValue)
				ret=0;
			break;
		}
	case 1: {
			if (var<=guards[n].compValue)
				ret=0;
			break;
		}
	case 0: {
			if (var<guards[n].compValue)
				ret=0;
			break;
		}
	default:
		break;
	}
	return ret;
}


int evalTrans(int n) {

	int i;
	for (i=0;i<transitions[n].numberOfGuards;i++) {
		if (evalGuard(transitions[n].guards[i]))
			return -1;
	}
	return transitions[n].followState;
}

void sendSensorData() {

	uart_send_static_text("D");
	int i;
	for (i=0;i<12;i++) {
		sprintf(out,"%d,\r",getVar(i));
		uart_send_text(out);
	}
	sprintf(out,"%d\r",getVar(12));
	uart_send_text(out);

	uart_send_static_text("\n");

}

void setControlledMotor() {
	int motorR=states[z].motorRoffset;
	int motorL=states[z].motorLoffset;

	if (states[z].motorRKpr) {
		motorR=controller
		       (states[z].motorRoffset, states[z].motorRmin, states[z].motorRmax,
		        states[z].motorRKpr, states[z].motorRvar, states[z].motorRvalue);
		e_set_speed_right(motorR);
	}
	if (states[z].motorLKpr) {
		motorL=controller
		       (states[z].motorLoffset, states[z].motorLmin, states[z].motorLmax,
		        states[z].motorLKpr, states[z].motorLvar, states[z].motorLvalue);
		e_set_speed_left(motorL);

	}

	if (debug&2) {
		sprintf(out,"M%d,%d\r\n",motorR,motorL);
		uart_send_text(out);
	}
}



void evalAllTrans(void) {

	for (j=0;j<states[z].numberOfTransitions;j++) {
		int ret=evalTrans(states[z].transitions[j]);
		if (ret!=-1) {
			z=ret;
			enterState();
		}
	}

	setControlledMotor();


	if (debug&4)
		sendSensorData();
};

int controller(int off, int min, int max,
        int kpr, int var, int value) {
	int varval=getVar(var);
	int diff=value-varval;
	long tempLong=(long)kpr*(long)diff;
	tempLong/=1000;
	if (tempLong>max)
		tempLong=max;
	else if (tempLong<min)
		tempLong=min;

	int res=tempLong+off;

	if (res>1024)
		return 1024;
	else if (res<-1024)
		return -1024;
	else
		return res;
};


int main(void) {

	if (RCONbits.POR) {	// reset if power on (some problem for few robots)
		RCONbits.POR=0;
		RESET();
	}
	e_init_port();   //Configure port pins
	e_init_uart1();   //Initialize UART to 115200Kbaud
	//e_calibrate_ir();
	e_init_motors();
	e_i2cp_init();
	e_init_ad_scan(ALL_ADC);
	//e_init_uart2(); //Initialize UART to 115200Kbaud
	e_activate_agenda(incTimer,10);
	e_activate_agenda(evalAllTrans,1000);

	while (1) {
		if (getSelector()==0) {
			int i=0;

			do if (e_getchar_uart1(&c))
					in[i++]=c;
			while (c!='\r');

			in[i++]='\n';
			in[i++]='\0';
			readInput();
			sprintf(out,"%s\r",in);
			if (strlen(in)>1)
				uart_send_text(out);

		}
	}


}
