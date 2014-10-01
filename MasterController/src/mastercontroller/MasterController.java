

/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package mastercontroller;

import java.net.*;
import java.io.*;
//import java.lang.*;
import static java.lang.Thread.sleep;

public class MasterController
{
    // CANbus ID DEFINITIONS
    static final int ID_OFFSET = 1 << 21;
    static final int TIME_MESSAGE_ID = 256 * ID_OFFSET;             // 0x200
    static final int STATE_MESSAGE_ID = 304 * ID_OFFSET;            // 0x260
    static final int TORQUE_MESSAGE_ID = 300 * ID_OFFSET;           // 0x258
    static final int MOTOR_MESSAGE_ID = 296 * ID_OFFSET;            // 0x250
    static final int CONTROL_LEVER_MESSAGE_ID = 641 * ID_OFFSET;    // 0x502
    static final int TENSION_MESSAGE_ID = 448 * ID_OFFSET;          // 0x380
    static final int CABLE_ANGLE_MESSAGE_ID = 464 * ID_OFFSET;      // 0x3a0
    static final int LAUNCH_PARAM_MESSAGE_ID = 327 * ID_OFFSET;     // 0x28e
    static final int PARAM_REQUEST_MESSAGE_ID = 312 * ID_OFFSET;    // 0x270

    static final float TICSPERSECOND = 64;
    static final float SIMULATIONSTEPTIME = ((float) 1.0) / TICSPERSECOND;
    static final float REALTIMEFACTOR = ((float) 1.0);

    static final float STEPTIMEMILLIS = 1000 * SIMULATIONSTEPTIME / REALTIMEFACTOR;
    static final float GRAVITY_ACCELERATION = (float) 9.81;
    static final float ZERO_CABLE_SPEED_TOLERANCE = (float) 0.1;
    //  DRIVE PARAMETERS
    static final float TORQUE_TO_TENSION = 20;
    static final float TENSION_TO_TORQUE = 1 / TORQUE_TO_TENSION;

    // LAUNCH PARAMS
    static final float GROUND_TENSION_FACTOR = (float) 1.0;
    static final float CLIMB_TENSION_FACTOR = (float) 1.3;

    static final float GLIDER_MASS = (float) 600;
    static final float GLIDER_WEIGHT = GLIDER_MASS * GRAVITY_ACCELERATION;

    //  for soft stat taper up
    static final float SOFT_START_TIME = (float) 1.0;
    static final float K1 = (float) Math.PI / (SOFT_START_TIME * TICSPERSECOND);

    //  for rotation taper down 
    static final float PROFILE_TRIGGER_CABLE_SPEED = (float) 20.578; // 40 knots
    static final float MAX_GROUND_CABLE_SPEED = (float) 35;
    static final float K2 = (float) (Math.PI
            / (2 * MAX_GROUND_CABLE_SPEED - PROFILE_TRIGGER_CABLE_SPEED));

    //  for transition to ramp
    static final float PEAK_CABLE_SPEED_DROP = (float) 0.97;

    //  for ramp taper up 
    static final float RAMP_TIME = 6;
    static final float K3 = (float) (Math.PI / (2 * RAMP_TIME * TICSPERSECOND));

//  for end of climb taper down
    static final float TAPERANGLETRIG = (float) 50;  //  Angle to start taper
    static final float TAPERTIME = (float) 3;  //  End of climb taper time
    static final float K4 = (float) (Math.PI / (2 * TAPERTIME * TICSPERSECOND));

    static final float RELEASEDELTA = (float) 5;    //  for detection of release

//    for parachute tension and taper
    static final float MAX_PARACHUTE_TENSION = (float) 3000;    //  newtons
    static final float PARACHUTE_TAPER_SPEED = (float) 25;      //  m/s
    static final float MAX_PARACHUTE_CABLE_SPEED = (float) 35;  //  m/s
    static final float K5 = (float) (Math.PI
            / (2 * MAX_PARACHUTE_CABLE_SPEED - PARACHUTE_TAPER_SPEED));

    private static void sendStateMessage(int newState,
            OutputStreamWriter outstream) throws IOException
    {
        CanCnvt stateMessage = new CanCnvt();
        stateMessage.id = STATE_MESSAGE_ID;

        switch (newState)
        {
            case 0: // prep
                System.out.println("Prep State");
                stateMessage.set_byte(1, 0);
                outstream.write(stateMessage.msg_prep());
                break;
            case 1: // armed
                System.out.println("Armed State");
                stateMessage.set_byte(2, 0);
                outstream.write(stateMessage.msg_prep());
                break;
            case 2: // profile
                System.out.println("Profile State");
                stateMessage.set_byte(3, 0);
                outstream.write(stateMessage.msg_prep());
                break;
            case 3: // profile
                break;                
            case 4: // ramp
                System.out.println("Ramp State");
                stateMessage.set_byte(4, 0);
                outstream.write(stateMessage.msg_prep());
                break;
            case 5: // constant
                System.out.println("Constant State");
                stateMessage.set_byte(5, 0);
                outstream.write(stateMessage.msg_prep());
                break;
            case 6: // recovery
                System.out.println("Recovery State");
                stateMessage.set_byte(6, 0);
                outstream.write(stateMessage.msg_prep());
                break;
        }
        outstream.flush();
    }

    public static void main(String args[]) throws InterruptedException
    {   // CANbus SCALE AND OFFSET DEFINITIONS
        float torqueScale = (float) (1.0 / 32.0);
        float tensionScale = (float) 0.25;
        float tensionOffset = (float) 1024.0;
        float cableAngleScale = (float) 0.5;
        float cableAngleOffset = (float) 40.0;
        float drumRadius = (float) 0.4;
        float motorToDrum = (float) 7.0;    //  speed reduction motor to drum

        byte fracTime;
        int elapsedTics = -1;
        double simulationStartTime;      // is this used?
        double timeMillis;
        double nextStepTime;
        double remainingTimeMillis;


        CanCnvt timeMessage = new CanCnvt();
        timeMessage.id = TIME_MESSAGE_ID;
        timeMessage.set_byte(0, 0); // time
        timeMessage.dlc = 1;

        float tension = (float) 0.0;
        float torque;
        float filt_torque = 0;
        float receivedTension = 0;
        float receivedMotorSpeed;
        float receivedCableSpeed = 0;
        float receivedCableAngle;

        float motorSpeedScale = (float) (1.0 / 128.0);

        CanCnvt torqueMessage = new CanCnvt();
        torqueMessage.id = TORQUE_MESSAGE_ID;
        torqueMessage.set_byte(0, 2); // max cable speed, not implemented

        CanCnvt requestMessage = new CanCnvt();
        requestMessage.id = PARAM_REQUEST_MESSAGE_ID;

        //  for debug use to insert extra messages
        CanCnvt temp = new CanCnvt();
        temp.id = STATE_MESSAGE_ID;
        temp.dlc = 0;

        CanCnvt canIn = new CanCnvt();
        String msg = "";

        // state machine stuff
        int state = 0;
        int speedMessageFlag = 0;
        int tensionMessageFlag = 0;
        int paramReceivedFlag = 0;
        int parametersRequestedFlag = 0;
        int launchResetFlag = 1;
        int startProfileTics = 0;
        int startRampTics = 0;
        float startRampTension = 0;
        float peakCableSpeed = 0;
        int taperFlag = 1;
        float taperTics = 0;
        float minCableSpeed = 0;
        

        // create the form
        ControlPanel cp = new ControlPanel();
        cp.setVisible(true);

        try
        {            
            // hub-server: start PS first when using 
            Socket connection = new Socket("192.168.1.113", 32123);
            
            //  MC local server: start this first when using
            //ServerSocket serverSocket = new ServerSocket(32123);
            //Socket connection = serverSocket.accept();

            connection.setTcpNoDelay(true);

            BufferedReader instream = new BufferedReader(new InputStreamReader(connection.getInputStream()));
            OutputStreamWriter outstream = new OutputStreamWriter(connection.getOutputStream());

            nextStepTime = System.currentTimeMillis();

            while (true)
            {   // endless loop    
                if (launchResetFlag == 1)   // init variables for launch
                {
                
                    state = 0;
                    tensionMessageFlag = speedMessageFlag = 0;
                    parametersRequestedFlag = 0;
                    paramReceivedFlag = 0;
                    launchResetFlag = 0;
                    filt_torque = 0;  //  This should be set to 0 on entry to
                                      //  Prep from Safe in real system
                                             
                    //  the below should not need to be re-initialized
                    //startProfileTics = 0;
                    //startRampTics = 0;
                    //startRampTension = 0;
                    //taperTics = 0;                    
                    //peakCableSpeed = (float) 0.0; 
                    //minCableSpeed = 0;
                    //tension = (float) 0.0;
                    //receivedTension = (float) 0.0;
                    //receivedCableSpeed = (float) 0.0;                 
                }
                
                //  note: this is now structured as a sequence that advances 
                //  on what is expected next and not the endless loop that 
                //  will be employed in the F4 MC where asynchronous events 
                //  can occur that affect both the state machine and the time
                //  messages.  This has been done trying to detemine the 
                //  reason for the slow execution speed of the current 
                //  simulation
                timeMillis = (double) System.currentTimeMillis();

                //System.out.println("System Time (ms), loop entry:  " + (long) timeMillis
                //          + " " + (long) nextStepTime + " " 
                //          + (long) (timeMillis - nextStepTime) 
                //          + " " + STEPTIMEMILLIS);
                while (timeMillis < nextStepTime)
                {
                    timeMillis = (double) System.currentTimeMillis();
                }

                //send time message and advance time one tic
                elapsedTics += 1;
                fracTime = (byte) (elapsedTics % TICSPERSECOND);
                timeMessage.set_byte(fracTime, 0); // time
                if (fracTime == 0)
                {
                    System.out.println("Second count: " + elapsedTics / TICSPERSECOND);
                    timeMessage.set_int((int) (elapsedTics / TICSPERSECOND), 1);
                }
                outstream.write(timeMessage.msg_prep());
                outstream.flush();
                timeMessage.dlc = 1;
                //  next Time message time                   
                nextStepTime += STEPTIMEMILLIS;

                //  read messages until tension and motor messages are received
                //  note: if there were other messages after the last tension 
                //  and motor messages (e.g., cable angle, they will be read 
                //  after the next time message.  This will be remidied by 
                //  threading the read operations in the future
                // System.out.println("System Time before reads (ms): "
                //         + (long) System.currentTimeMillis());
                while ((tensionMessageFlag == 0)
                        || (speedMessageFlag == 0))
                {

                    //  wait for input message    
                    if ((msg = instream.readLine()) == null)
                    {
                        System.out.println("Null message received");
                    } else
                    {
                        //System.out.println("System Time after read (ms):     " 
                        //+ (long) System.currentTimeMillis() + "  "
                        //+ tensionMessageFlag + speedMessageflag);

                        canIn.convert_msgtobin(msg);

                        // System.out.println("CANid: " + canIn.id / ID_OFFSET);
                        switch (canIn.id)
                        {
                            case TENSION_MESSAGE_ID:
                                receivedTension = (canIn.get_short(1) - tensionOffset)
                                        * tensionScale;
                                tensionMessageFlag = 1;

                                // System.out.println("Tension is: " + receivedTension);
                                break;
                            case MOTOR_MESSAGE_ID:
                                receivedMotorSpeed = canIn.get_short(1)
                                        * motorSpeedScale;
                                receivedCableSpeed = (float) (2 * 3.14159
                                        * drumRadius * receivedMotorSpeed
                                        / motorToDrum);
                                // System.out.println("Speeds are: " + receivedMotorSpeed + " " + receivedCableSpeed);
                                speedMessageFlag = 1;
                                break;
                            case CABLE_ANGLE_MESSAGE_ID:
                                receivedCableAngle = (canIn.get_ubyte(1) - cableAngleOffset) * cableAngleScale;
                                if (taperFlag == 0 && receivedCableAngle > TAPERANGLETRIG)
                                {
                                    taperFlag = 1;
                                    taperTics = elapsedTics;
                                }
                                // System.out.println("Cable Angle is: " + receivedCableAngle);
                                break;
                            case LAUNCH_PARAM_MESSAGE_ID:
                                System.out.println("Launch Parameters Message Recieved");
                                paramReceivedFlag = 1;
                                break;
                        }
                    }
                }

                //System.out.println("System Time after reads (ms):  "
                //        + (long) System.currentTimeMillis());
                // Template for state machine (with accomodation for control panel I/O differences)
                switch (state)
                {
                    case 0: // prep                        
                        if (cp.getSlider() < 0.1)
                        {
                            state = 1; // going to armed state
                            cp.setStateLed(1);
                            sendStateMessage(1, outstream);
                            //  System.out.println("Going to state: " + state);
                        }
                        break;
                    case 1: // armed
                        
                        if ((parametersRequestedFlag == 0) 
                                && (cp.getSlider() > 0.9))
                        {
                            // request launch parameters
                            outstream.write(requestMessage.msg_prep());
                            outstream.flush();
                            parametersRequestedFlag = 1;
                        }
                        // when we get the response, start the simulation
                        if (paramReceivedFlag == 1)
                        {
                            simulationStartTime = (double) (System.currentTimeMillis());
                            
                            state = 2;
                            cp.setStateLed(2);
                            startProfileTics = elapsedTics;
                            sendStateMessage(2, outstream);
                            //  System.out.println("Going to state: " + state);
                        }
                        break;
                    case 2: // profile 1
                        if (elapsedTics - startProfileTics
                                >= SOFT_START_TIME * TICSPERSECOND)
                        {
                            state = 3;
                            peakCableSpeed = receivedCableSpeed;
                            cp.setStateLed(3);
                            //  System.out.println("Going to state: " + state);
                        }
                        break;
                    case 3: // profile 2

                        peakCableSpeed = receivedCableSpeed > peakCableSpeed
                                ? receivedCableSpeed : peakCableSpeed;
                        if (receivedCableSpeed < (peakCableSpeed
                                * PEAK_CABLE_SPEED_DROP))
                        {
                            state = 4;
                            startRampTics = elapsedTics;
                            startRampTension = receivedTension;
                            sendStateMessage(4, outstream);
                            cp.setStateLed(4);
                            //  System.out.println("Going to state: " + state);
                        }
                        break;
                    case 4: // ramp
                        if (elapsedTics - startRampTics > RAMP_TIME * TICSPERSECOND)
                        {
                            state = 5;
                            cp.setStateLed(5);
                            sendStateMessage(5, outstream);
                            minCableSpeed = receivedCableSpeed;
                            //  System.out.println("Going to state: " + state);
                            taperFlag = 0;
                        }
                        break;
                    case 5: // constant
                        //                       System.out.println(receivedCableSpeed);
                        if (receivedCableSpeed < minCableSpeed)
                        {
                            minCableSpeed = receivedCableSpeed;
                        }
                        if (receivedCableSpeed > minCableSpeed + RELEASEDELTA)
                        {
                            state = 6;
                            cp.setStateLed(6);
                            sendStateMessage(6, outstream);
                            //  System.out.println("Going to state: " + state);
                        }
                        break;
                    case 6: // recovery
//                        System.out.println(receivedCableSpeed);
                        if (receivedCableSpeed < ZERO_CABLE_SPEED_TOLERANCE)
                        {
                            state = 0;
                            cp.setStateLed(0);
                            sendStateMessage(0, outstream);
                            //  System.out.println("Going to state: " + state);
                            launchResetFlag = 1;                            
                        }
                        break;
                    
                          

                }
                //System.out.println("System Time after State (ms):  " 
                //        + (long) System.currentTimeMillis() + "  "
                //        + tensionMessageFlag + speedMessageflag);

                //  Template for Desired Tension and Control Law        
                switch (state)
                {
                    case 0: // prep
                        tension = 0;
                        break;

                    case 1: // armed
                        tension = 0;
                        break;
                    case 2: // profile 1
                        tension = (float) (GROUND_TENSION_FACTOR * GLIDER_WEIGHT * 0.5
                                * (1 - Math.cos(K1 * (elapsedTics - startProfileTics))));
                        break;

                    case 3: // profile 2
                        // System.out.println(receivedCableSpeed +  PROFILE_TRIGGER_CABLE_SPEED);
                        if (receivedCableSpeed < PROFILE_TRIGGER_CABLE_SPEED)
                        {
                            tension = GROUND_TENSION_FACTOR * GLIDER_WEIGHT;
                            //                            System.out.println(tension);
                        } else
                        {
                            tension = (float) (GROUND_TENSION_FACTOR * GLIDER_WEIGHT
                                    * Math.cos(K2 * (receivedCableSpeed - PROFILE_TRIGGER_CABLE_SPEED)));
                            //                            System.out.println(tension);
                        }
                        break;
                    case 4: // ramp
                        tension = (float) ((startRampTension
                                + (CLIMB_TENSION_FACTOR * GLIDER_WEIGHT
                                - startRampTension)
                                * Math.sin(K3 * (elapsedTics - startRampTics))));
                        //  System.out.println(tension);
                        break;
                    case 5: // constant
                        tension = (float) (CLIMB_TENSION_FACTOR * GLIDER_WEIGHT);
                        if (taperFlag == 1)
                        {
                            tension *= 0.4 + 0.6 * 0.5
                                    * (1 + Math.cos(K4 * (elapsedTics - taperTics)));
                        }
                        break;
                    case 6: // recovery
                        tension = MAX_PARACHUTE_TENSION;
                        if (receivedCableSpeed > PROFILE_TRIGGER_CABLE_SPEED)
                        {
                            tension *= Math.cos(K5 * (receivedCableSpeed - PARACHUTE_TAPER_SPEED));
                            //                            System.out.println(tension);
                        }
                        break;
                    
                }
                tension *= cp.getSlider(); // scale by slider 
                torque = tension * TENSION_TO_TORQUE;
                //  filter the torque with about 1 Hz bandwidth
                filt_torque += (torque - filt_torque) / 8;

                torqueMessage.set_short((short) (filt_torque / torqueScale), 0); // torque
                outstream.write(torqueMessage.msg_prep());
                outstream.flush();
                tensionMessageFlag = speedMessageFlag = 0;
                //System.out.println("System Time after Torque (ms): " 
                // + (long) System.currentTimeMillis() + "  "
                // + tensionMessageFlag + speedMessageflag);

                //  go to sleep until time for the next time message
                //  this should not be needed when the reads are separately
                //  threaded
                //timeMillis = (double) System.currentTimeMillis();
                //remainingTimeMillis = nextStepTime - timeMillis;
                //if (remainingTimeMillis > 0)
                //{
                //  ceiling the sleep period
                //Thread.sleep((int) (remainingTimeMillis + 1));
                //}
            }

        } catch (IOException e)
        {
            e.printStackTrace();
        }

    }
}
