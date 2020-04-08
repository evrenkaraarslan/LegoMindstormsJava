package basicCode;
import lejos.nxt.*;

public class TaskOne {
// defining static variables that will be necessary in all parts of the code
	static boolean flagInit = false; //Initial section flag
	static boolean flagLine = false; //line following section flag
	static boolean flagAfterLine = false; //wall alignment section flag
	static boolean flagInter = false; //wall following section flag
	static boolean flagObst = false; //obstacle seeking section flag
	static boolean obsDet = false; //obstacle circumnavigating section flag
	static long odoY = 0; //Y axis odometry
	static int odoTh = 0; //Directional odometry
	static int wallOffset = 25; //reference distance for wall following
	static int wKp = 2000; //wall following controller proportional gain
	static int wKi = 50; //wall following controller integral gain
	static int wInteg = 0; //wall following controller integrator
	static int offset = 47; //reference intensity for line following
	static int Kp = 8000; //line following controller proportional gain
	static int Ki = 300; //line following controller integral gain
	static int Kd = 0; //line following controller derivative gain
	static int Tp = 300; //line following controller feedforward speed value
	static int Dt = 100; //controller sampling time gain
	static int integral = 0; //line following controller integrator
	static int prevSens = 0; //previous light sensor value
	static int prevError = 0; //line following controller previous error value
	static int limit = 400; //line following controller limit value
	static int filterSize = 4; //sensor mean filter size
	static int posA = 0; //right motor position
	static int posB = 0; //left motor position

public static void main(String args[]) throws InterruptedException {
	LightSensor lightSensor = new LightSensor(SensorPort.S2);  
	UltrasonicSensor sonic = new UltrasonicSensor(SensorPort.S1); 
	while (! Button.ESCAPE.isDown()) { 
		lightSensor.setFloodlight(true);
		int sensorValue = lightSensor.readNormalizedValue()/10; //light value
		if(sensorValue > 60) sensorValue = 20; //discard sensor saturation
		Thread.sleep(40);
		if(!flagInit)
		{ //initial section
			LCD.drawString("Sensor value: " + sensorValue, 0, 6);
			LCD.drawString("initial section", 0, 1);
			LCD.drawString("is executing..", 0, 2);
			LCD.refresh();
			Motor.A.forward();
			Motor.A.setSpeed(300); //Initial speed for right motor
			Motor.B.forward();
			Motor.B.setSpeed(300); // Initial speed for left motor
			if ((prevSens>48)&&(prevSens - sensorValue)>2) flagInit = true; 
// The light sensor should detect a fall from line to ground to end initial section
			prevSens = sensorValue;  // update previous sensor value
		}
		else if (!flagLine)
		{ //line following section
			int dist = sonic.getDistance(); //distance from ultrasonic sensor
			LCD.drawString("Sensor value: " + sensorValue, 0, 6);
			LCD.drawString("distance:   " + dist, 0, 3);
			LCD.refresh(); 
			if (dist<20) flagLine = true; 
                        // An object should be detected to end the line following section
			LCD.drawString("line f section   ", 0, 1);
			LCD.refresh();
			LCD.drawString("Sensor value: " + sensorValue, 0, 6);   
			int error = sensorValue - offset; // to find light error in each cycle
			int difError = prevError - error;  // error differentiator
                        prevError = error; // update previous error value
			integral = integral + error; // integral action
			LCD.drawString("Error: " + error, 0, 4);
			LCD.drawString("Integral: " + integral, 0, 5);
			LCD.refresh(); 
			int turn = (Kp * error + Ki * integral + Kd * difError)/Dt; // PID
			int powerA = Tp + turn; // determine right motor power
			int powerB = Tp - turn; // determine left motor power
			if(powerA>limit) powerA = limit; // saturate right motor
			else if(powerA<(-1*limit)) powerA = -1 * limit;
			if(powerB>limit) powerB = limit; // saturate left motor
			else if(powerB<(-1*limit)) powerB = -1 * limit;
			if(powerA<0) Motor.A.backward(); //define right motor direction
			else Motor.A.forward();
			Motor.A.setSpeed(powerA); 
			if(powerB<0) Motor.B.backward(); // define left motor direction
			else Motor.B.forward();
			Motor.B.setSpeed(powerB);
		}
		else if(!flagAfterLine)
		{ //wall alignment section
			LCD.drawString("line f section   ", 0, 1);
			LCD.drawString("is finished..   ", 0, 2);
			LCD.drawString("Sensor value: " + sensorValue, 0, 6);
			LCD.refresh();
			Motor.A.setSpeed(200); // back speed for right motor
                        Motor.B.setSpeed(200); // back speed for left motor
			Motor.A.backward();
			Motor.B.backward();
			Thread.sleep(1500); // go back for 1.5 secs
			Motor.A.setSpeed(100); // slow down for right motor
			Motor.B.setSpeed(100); // slow down for left motor
			Motor.A.stop(); // stop right motor
			Motor.B.stop(); // stop left motor
			int positionB = Motor.B.getPosition(); // get left motor position
			Motor.C.rotateTo(-90); // turn Ultrasonic sensor to 90 degree left
			Motor.A.setSpeed(400); // set speed for right motor
			Motor.B.setSpeed(400); // set speed for left motor
			Motor.B.rotateTo(positionB+400); // turn to right
			Thread.sleep(500); // wait for 0.5 secs
			Motor.A.setSpeed(300); // set speed for right motor
			Motor.B.setSpeed(300); // set speed for left motor
			Motor.A.forward(); 
			Motor.B.forward();
			Thread.sleep(500); // move forward for 0.5 secs
			prevSens = 0; // set down previous light sensor value
			flagAfterLine = true; // end of section
		}
        else if(!flagInter)
	{ //wall following section
				LCD.drawString("looking for   ", 0, 1);
				LCD.drawString("obstacle area ", 0, 2);
				LCD.refresh();
				int cumSensor = 0;
				for (int cnt=0;cnt<filterSize;cnt++)
				{
					int white = sonic.getDistance(); //read white value
					cumSensor = cumSensor + white;
				} // filters distance reading values
				int obsDist = cumSensor / filterSize;
				if (obsDist > 30) obsDist = 30; // saturates distance
				LCD.drawString("distance:   " + obsDist,0, 3);
				LCD.refresh();
				int error = obsDist - wallOffset; // wall following controller error
				wInteg = wInteg + error; // wall following controller integrator
				LCD.drawString("Error: " + error, 0, 4);
				LCD.drawString("Integral: " + wInteg, 0, 5);
				LCD.refresh();
				int turn = (wKp * error + wKi * wInteg)/Dt; // PI controller
				int powerA = 250 + turn; // determine right motor power
				int powerB = 250 - turn; // determine left motor power
				if (powerA < 100) powerA = 100; // down limit for right motor
				if (powerB < 100) powerB = 100; // down limit for left motor
				Motor.A.setSpeed(powerA);
				Motor.B.setSpeed(powerB);
				if((prevSens>48)&&(prevSens - sensorValue> 3) // obstacle area
                                {
					wInteg = 0; // setting down integrator
					Motor.A.setSpeed(100); // slow down for right motor
					Motor.B.setSpeed(100); // slow down for left motor
					Motor.C.rotateTo(0); // turn Ultrasonic sensor forward
					flagInter = true; // end of section
					LCD.clear(); 
			        }
prevSens = sensorValue;
posA = Motor.A.getPosition();//getting initial position from motor A		posB = Motor.B.getPosition();//getting initial position from motor B
}
        else if(!flagObst)
	{ //obstacle seeking section
				LCD.drawString("look for     ", 0, 1);
				LCD.drawString("an obstacle ", 0, 2);
				int dist = sonic.getDistance(); // get distance value from Ultrasonic sensor
				LCD.drawString("distance: " + dist, 0, 3);
				LCD.refresh();				
				Motor.A.setSpeed(200); // set speed for right motor
				Motor.B.setSpeed(200); // set speed for left motor
				Motor.A.forward(); // go forward
				Motor.B.forward();
				odoTh = odoTh + 275 * (Motor.A.getPosition() - posA + posB - Motor.B.getPosition()); // calculate direction odometry
                                LCD.drawString("Theta: " + odoTh, 0, 4);
				double Ds = 27.5 * 3.14 * (Motor.A.getPosition() - posA + Motor.B.getPosition() - posB) / 360; // calculate elongational distance
				odoY = odoY + 1000 * Math.round(Ds); // update Y odometry
                                LCD.drawString("Y:  " + odoY, 0, 6);
				LCD.refresh();
				posA = Motor.A.getPosition(); // get position of right motor
				posB = Motor.B.getPosition(); // get position of left motor
				if(dist<22) //Obstacle detected
                                {   
					Motor.A.stop(); // stop motors
					Motor.B.stop();
                                        Motor.C.rotateTo(-90); // ultrasonic sensor rotates 90 deg towards to obstacle 
					Motor.A.setSpeed(100);
					Motor.B.setSpeed(100);
					Motor.A.backward();
					Motor.B.forward(); 
					LCD.drawString("odo:   " + odoTh, 0, 4);
					LCD.refresh();
					while ((odoTh > -90000)) //robot rotates 90deg. to right
                                        {
						LCD.drawString("odo:   " + odoTh, 0, 4);
						LCD.refresh();
						Thread.sleep(88);
						odoTh = odoTh - 5000; // updates direction odometry
					}
					Motor.A.stop(); // stop motors
					Motor.B.stop();
					Thread.sleep(100);
					Motor.A.setSpeed(200);
					Motor.B.setSpeed(200);
					flagObst = true; // switch to circumnavigate algorithm
					obsDet = false;
					LCD.drawString("Theta: " + odoTh, 0, 4);
					LCD.refresh();
					posA = Motor.A.getPosition(); // get position of right motor
					posB = Motor.B.getPosition(); // get position of left motor
				    }
				if((prevSens>46)&&((prevSens-sensorValue)>2)&&(odoY>0)) 
                                {// reaches to end point
					LCD.clear();
					LCD.drawString("MISSION   ", 0, 1);
					LCD.drawString("COMPLETED  ", 0, 2);
					LCD.refresh();
					Motor.A.stop(); // stop motors
					Motor.B.stop();
					flagObst=true; // end of mission
					obsDet=true;
					Sound.twoBeeps();
                }
				prevSens = sensorValue; // update previous light sensor value
			}
			else if(!obsDet) 
		        { //circumnavigationg the obstacle till absolute 0 angle
				LCD.drawString("circumnavigate", 0, 1);
				LCD.refresh();
				Motor.A.forward(); // go forward
				Motor.B.forward();
				int cumSensor = 0;
				for (int cnt=0;cnt<filterSize;cnt++) 
                                {
					int white = sonic.getDistance(); //read white value
					cumSensor = cumSensor + white;
				} //filtering distance values
				int obsDist = cumSensor / filterSize;
				if (obsDist > 30) obsDist = 30;
				LCD.drawString("distance:   " + obsDist, 0, 3);
				LCD.refresh();
				int error = obsDist - 22; // finding error for PI control
				wInteg = wInteg + error; // obstacle following integrator
				LCD.drawString("Error: " + error, 0, 4);
				LCD.refresh();
				int turn = (2000 * error + 50 * wInteg)/Dt; // PI controller
				int powerA = 200 + turn; //Determining speed of motors
				int powerB = 200 - turn;  //Determining speed of motors
				if (powerA < 50) powerA = 50; // under limit motor powers
				if (powerB < 50) powerB = 50;
				Motor.A.setSpeed(powerA);
				Motor.B.setSpeed(powerB);
				int Dth = 275 * (Motor.A.getPosition() - posA + posB - Motor.B.getPosition());// calculation of delta direction
                                LCD.drawString("Theta: " + odoTh, 0, 4); 
				double Ds = 27.5 * 3.14 * (Motor.A.getPosition() - posA + Motor.B.getPosition() - posB) / 360; 
                                double ThRad = 3.14 * (odoTh/1000 + (Dth / 2000+90))/180;//angle in rad
				long mmY = Math.round(1000*Math.sin(ThRad));
				odoY = odoY + Math.round(Ds * mmY); //updating y odometry
                                odoTh = odoTh + Dth; //updating direction odometry
                                LCD.drawString("Y:  " + odoY, 0, 6);
				posA = Motor.A.getPosition(); // get position of right motor
				posB = Motor.B.getPosition(); // get position of left motor
                if((obsDist > 22)&&(odoTh>0))
                {//when robot reaches the absolute 0 angle and getting far from obstacle
					if (odoTh > 10000)
                                        { // adjust absolute direction to zero
						Motor.A.setSpeed(100);
						Motor.B.setSpeed(100);
						Motor.A.stop();
						Motor.B.forward();
						LCD.drawString("odo:   " + odoTh, 0, 4);
						LCD.refresh();
						while ((odoTh > 0))
                        {
							LCD.drawString("odo:   " + odoTh, 0, 4);
							LCD.refresh();
							Thread.sleep(44);
							odoTh = odoTh - 2500;
						}
					}
					Motor.C.rotateTo(0);
					Motor.A.setSpeed(200);
					Motor.B.setSpeed(200);
					flagObst = false;//set off obstacle following mode
					obsDet = true;//sets obstacle detection mode
				}
				if((prevSens>46)&&((prevSens - sensorValue) > 2)&&(odoY>0)) 
                                {// reaches to end point, code is as same before
					LCD.clear();
					LCD.drawString("MISSION   ", 0, 1);
					LCD.drawString("COMPLETED  ", 0, 2);
					LCD.refresh();
					Motor.A.stop();
					Motor.B.stop();
					flagObst=true; // end of mission
					obsDet=true;
					Sound.twoBeeps();
                }
				prevSens = sensorValue;
			}
		}
		lightSensor.setFloodlight(false);
	}
}


