// SIM-C2829 with data data262.txt - 14Apr17
//   - serial_output every 200ms
//   - at row 282, input data set for HOME
// ---------------------------------------------------
// #include <conio.h>
#include <stdio.h>
#include <time.h>
// #include <curses.h>
#include <stdlib.h>
#include <unistd.h>
// #include <delays.h>
#include <wiringPi.h>

#define ENABLE      8   // B8 enable outputs
#define	DATAout		5   // B5
#define CLOCKout	7   // B7
#define STROBE   	2   // B3
#define	CLOCKin		3   // B2
#define LOAD		4   // B1
#define DATAin		0   // B4
#define ENshift     6   // EN enable level shifter

int         b, i, r, w, x, y, z, mode, tt, read2=0, state, interval, code, min, temp, temp2, temp_time, force=0, iface=0;
int         nextfree, speed, nexttrain, line, signal, newroute, dmu, demo, welwyn=0, enable=0;
int         lock_flag=0, block_flag=0, m6503, rdy_time=0, tt_time, out_flag, clock_flag=0, lever21=0;
int         lever_status[74][18], indicator_status[40][17], TC_status[24][7];
int         ind_addr[49][2], TC_addr[21][2], annun_status[3][4];
int         block_input[16][5], block_output[28][4], block_locks[5][12], IBS_flag[4];
int         input_data[129], output_data[129], input_test[129], output_test[41];
int         bell_status[5][28], bellcode[20][14], bell_flags[5][8];
int         route[6][9][10], trainqueue[7][5], trains[19][10][6], timetable[100][5], trainqueue2[50][5];
int         GPIOinput[97];
float       fx,fy;
double      start_time;

void GPIO_setup(void);
int menu();
void serial_input();
int process_block();
int block_write();
int process_bell();
int bell_decode();
int bell_ring();
int check_cancel();
int process_lever();
int process_indicator();
int IBS_bells();
int serial_output();
int calc_locks();
int calc_block_locks();
int create_train();
int move_train();
int calc_TCoff();
int reduce_speed();
int change_route();
int annunciator();
int file_read();
int random2(int max);
void delay0 (void);
void delay1 (void);
int getch(void);
int kbhit (void);
void set_getch_mode();
int serial_input_test(void);
int serial_output_test(void);
double clock0();

int main(void)
{
    GPIO_setup();
    set_getch_mode();
start:
    if( menu() ) return 0;

    if(file_read()) return 0;  // read from file: lever_status and indicator_status

    if( mode == 200 ) { serial_input_test(); goto start; }
    if( mode == 201 ) { serial_output_test(); goto start; }

    printf("\r\nPress ENTER to start\r\n");
loop4:
    x = getch();
    usleep(1000);  // 1ms delay
    if(x != 13) goto loop4;  // wait for ENTER ( = 13 )
    printf("\r\nSTARTED\r\n\r\n");
    temp = 0;
    temp2 = clock0();    // use for serial_output every 2 seconds
    temp_time = clock0();
    tt_time = clock0();  // timetable start time offset (if applicable)

loop2:

    temp = temp + 1;
    usleep(1000);  // 1ms delay

    serial_input();

    process_block();

    block_write();

    process_bell();  // record incoming, set outgoing bell signals, and ring bell

    process_lever();

    process_indicator();

    IBS_bells();

    serial_output();

    calc_locks();

    calc_block_locks();

    create_train();

    move_train();

    if( demo == 0 ) annunciator();  // not in demo mode

    if( temp == 100 )
    {
        if( clock0() - temp_time > 320 ) printf("\r\nTime for 100 cycles: %3.0lf\r\n",clock0() - temp_time);
        temp = 0;
        temp_time = clock0();
        // force lever read after first 500 cycles ??
        if( force == 0 )
        {
            force = 1;
            // also set serial output flag to force populate output and enable output for first time
            out_flag = 1;
            // also set block_lock flags if Welwyn winder not at 'N' at start-up
            x = 1;  // line 1
            while( x < 4 )
            {
              if( block_input[19 - x * 4][0] == 0 )  // Welwyn N=0 (HS 15, HF 7)
              {
                block_locks[x][0] = 1;  // LC locked
                block_locks[x][6] = 1;  // set N=0
                block_locks[x][7] = 1;  // set R=1
                block_locks[x][9] = 1;  // set R=0
                // block will release when Welwyn returned to 'N' (ie. N=1)
              }
              x = x + 2;  // line 3
            }
        }
    }

    if( ( temp2 + 200 ) < clock0() )
    {
        out_flag = 3;
        serial_output();
        temp2 = clock0();
    }

    if( ! kbhit() ) goto loop2;
    getch();
loop3a:
    printf("\r\nProgram paused\r\n\r\nPress Enter to continue, or Q to quit\r\n\r\n");
loop3:
    x=getch();
    // Sleep(1);  // 1ms delay
    if(x!=13 && x!=113 && x!=114 && x!=81 && x!=119 && x!=27 && x!=3) goto loop3;    // Enter is 10, q is 113, Q is 81, w is 119, ESC is 27, CTL-C is 3
    if(x==13)  // continue
    {
        printf("Program resumed\r\n");
        temp = 0;               // reset cycles timer
        temp_time = clock0();
        goto loop2;
    }
    if(x==119)  // reset Welwyn winders
    {
        printf("Welwyn reset - block lock status\r\n   US - ");
        for(y = 0; y < 10; y++) printf("%d ",block_locks[1][y]); printf("\r\n   UF - ");
        for(y = 0; y < 10; y++) printf("%d ",block_locks[3][y]); printf("\r\n\r\n");
        for(x = 1; x < 3; x++) for(y = 6; y < 10; y++) block_locks[2*x-1][y] = 1;  // set Welwyn flags; calculate will release
        block_locks[1][0] = 1; block_locks[3][0] = 1;  // force LC locked
        block_flag = 1;  // set block_locks calculate flag
        goto loop2;  // continue
    }
    if(x==114) serial_input_test();
    if(x==81 || x==113 || x==27 || x==3)
    {
        printf("Finish - are you sure (y/n)?\r\n");
        x=getch();
        if( x!=121 && x!=89 ) goto loop3a;  // y is 121, Y is 89
    }

    goto start;  // exit to menu

    return 0;
}

void GPIO_setup (void)
{
  usleep(1000);  // 1ms delay
  if (geteuid () != 0)
  {
    fprintf (stderr, "Need to be root to run (sudo?)\r\n") ;
    exit (0) ;
  }

  if (wiringPiSetup () == -1)
    exit (1) ;

  printf ("GPIO setup ... ") ; fflush (stdout) ;

  // set all GPIO to outputs, and write zero
  for(x=0;x<9;x++) { pinMode(x,OUTPUT); digitalWrite(x,0); }
  usleep(1000);  // 1ms delay

  pinMode(DATAin,INPUT);     // DATA is input
  digitalWrite(ENshift,1);   // enable level shifter
  digitalWrite(LOAD,1);      // LOAD high normal
  // digitalWrite(LOAD,0);
  // digitalWrite(CLOCKin,1);   // CLOCK high normal
  digitalWrite(CLOCKin,0);

  printf ("OK\r\n") ;
}

int menu()
{
    clock_flag = 0;  // start from zero next time clock0() used
loopt4:
    printf("\r\nSimulator - main menu\r\n\r\n1. Demo - Up slow\r\n2. Demo - Up fast\r\n3. Full operation");
    printf("\r\n4. Passive\r\n5. Timetable menu\r\n7. Serial output test\r\n8. Serial input test\r\n9. Exit\r\n");
    demo = 0;  // clear demo flag
loopt1:
    x=getch();
    // Sleep(1);  // 1ms delay
    if( x==57 )                   // 9 = getch 57
    {
      digitalWrite( ENABLE, 0 );  // turn off outputs
      return 1;
    }
    else if( x==49 ) mode = 13;   // 1 = getch 49 - Demo - Up slow
    else if( x==50 ) mode = 14;   // 2 = getch 50 - Demo - Up fast
    else if( x==51 ) mode = 11;   // 3 = getch 51 - full operation
    else if( x==52 ) mode = 15;   // 4 = getch 52 - Passive
    else if( x==53 ) goto loopt2; // 5 = getch 53 - Timetable menu (mode 12)
    else if( x==55 ) mode = 201;  // 7 = getch 55 - test
    else if( x==56 ) mode = 200;  // 8 = getch 56 - test
    else goto loopt1;
    if( mode == 13 || mode == 14 ) demo = 1;  // set demo flag
    return 0;
loopt2:
    mode = 12;  // timetables
    printf("\r\nTimetable menu\r\n\r\n1. Fast lines 20 minute demo\r\n2. Fast lines 20 minute demo\r\n3. All lines 20 minute demo\r\n4. Other routes sample\r\n5. WTT from 0654\r\n6. Slow lines - training\r\n7. 4-Track (3-1 and 4)\r\n9. Exit\r\n");
loopt3:
    x=getch();
    // Sleep(1);  // 1ms delay
    if( x==57 ) goto loopt4;         // 9 = getch 57
    else if( x==49 ) tt = 1;   // 1 = getch 49 - TT1
    else if( x==50 ) tt = 2;   // 2 = getch 50 - TT2
    else if( x==51 ) tt = 3;   // 3 = getch 51 - TT3
    else if( x==52 ) tt = 4;   // 4 = getch 52 - TT4
    else if( x==53 ) tt = 5;   // 5 = getch 53 - TT5
    else if( x==54 ) tt = 6;   // 6 = getch 54 - TT6
    else if( x==55 ) tt = 7;   // 7 = getch 55 - TT7
    else goto loopt3;
    return 0;
}

void serial_input()
{
    // input from 12 shift registers
    for(x=1;x<97;x++) GPIOinput[x] = 0;		    // zeroise input data
    digitalWrite (LOAD, 0);				        // LOAD on
    delay0(); delay0();	delay1();					            // delay
    digitalWrite (LOAD, 1);				        // LOAD off
    delay1();						            // delay
    for( x=1; x<97; x++ )					    // 12 registers
    {
	  delay1();					                // delay
	  GPIOinput[x] = digitalRead (DATAin);   	// store data
	  delay1(); delay0();					    // delay
	  digitalWrite (CLOCKin, 0);			    // CLOCK low
	  delay1();	delay0();				        // delay
	  digitalWrite (CLOCKin, 1);		    	// CLOCK high, shifts on rising edge
	  delay1();	delay0();				        // delay
    }
    /*
    //  --- FOR SIGNAL BOX ---
    // extract data into input_data; 12 x 8 shift registers
    for(x=0; x<6; x++) for(y=1; y<9; y++)
      {
          // see spreadsheet for derivation of formula for index
          input_data[ x*16 + 2*y -1 ] = GPIOinput[ x*8 +(x>2)*24 +y ];
          input_data[ x*16 + 2*y ] = GPIOinput[ x*8 +(x>2)*24 +29 +(y>4)*8 -y ];
      }
    */

    //  --- FOR HOME ---
    for( x=1; x<97; x++ ) input_data[x] = GPIOinput[x];

    // levers - populate lever_status
    for(x=0; x<74; x++) lever_status[x][1] = input_data[x+1];
    // block - popualte block_input
    for(x=0; x<16; x++)
    {
        w = block_input[x][4];   // location in input_data
        block_input[x][1] = input_data[w];
    }
}

int process_block()
{
    // update outputs
    for( x = 0; x < 16; x++ )
    {
        // normal
        if( block_input[x][0] != block_input[x][1] )
        {
            // set change for peggers
            if( block_input[x][2] > 0 && block_input[x][2] < 100 )
            {
                if( clock0() < ( block_input[x][3] + 50 ) && block_input[x][1] == 0 ) goto skips1;  // accept change to 'off' after 50ms minimum
                block_output[block_input[x][2]][1] = block_input[x][1];
                // set 'Comm TOL' flag in block_locks, and set block_locks calculate flag
                if( ( mode == 11 || mode == 12 || mode == 15 ) && block_input[x][1] == 1 )
                {
                    if( x == 1 && block_locks[1][0] == 1 ) { block_locks[1][4] = 1; block_flag = 1; }  // HS
                    else if( x == 11 && block_locks[2][0] == 1 ) { block_locks[2][4] = 1; block_flag = 1; }  // NS
                    else if( x == 9 && block_locks[3][0] == 1 ) { block_locks[3][4] = 1; block_flag = 1; }  // HF
                    else if( x == 3 && block_locks[4][0] == 1 ) { block_locks[4][4] = 1; block_flag = 1; }  // NF
                }
                else if( demo == 1 && ( x == 3 || x == 11 ) )
                {
                    // allows Rotary TOL release in Demo mode (as for Passive)
                    block_locks[ ( 19 - x ) / 4 ][4] = 1;  // set 'Comm TOL' flag in block_locks
                    block_flag = 1;  // set block_locks calculate flag
                }
            }
            // set change for bells
            else if( block_input[x][2] > 100 && block_input[x][2] < 200 ) bell_status[block_input[x][2] - 100][1] = block_input[x][1];
            // Welwyn winder
            else if( block_input[x][2] > 200 && demo == 0 && welwyn == 1 )  // not in demo modes or if switched off
            {
                if( clock0() < ( block_input[x][3] + 100 ) ) goto skips1;  // accept change after 50ms minimum
                w = block_input[x][2] - 200;
                if( w < 3 ) z = 1;  // Up slow
                else z = 3;  // Up fast
                state = block_input[x][1];  // new state is 0 off or 1 on
                if( ( w % 2 ) == 1 )  // Normal switch (N)
                {
                    if( state == 0 && block_locks[z][6] != 1 ) { block_locks[z][6] = -1; bell_flags[z][5] = clock0(); }  // N=0 set wait flag and time (for spurious override)
                    else if( state == 1 && block_locks[z][6] == 1 && block_locks[z][7] == 1 ) block_locks[z][8] = 1;  // N=1
                }
                else block_locks[z][9 - state * 2] = 1;  // Operated switch (R)
                block_flag = 1;  // set block_locks calculate flag
                printf("Line %d Welwyn status: ",z);
                for( y = 0; y < 10; y++ ) printf("%d ",block_locks[z][y]);
                printf("\r\n");
            }
            block_input[x][0] = block_input[x][1];
            block_input[x][3] = clock0();  // used to accept next read after 50ms (not for bells here)
            printf("%03.1f Block in %d Status %d\r\n",(fx=clock0())/1000,x,block_input[x][1]);
          skips1:
            asm("nop");  // loop to next x value
        }
    }
    // accept Welwyn N=0 as genuine after 3 seconds
    x = 1; while( x < 4 )  // US x = 1, UF x = 3
    {
        if( block_locks[x][6] == -1 && clock0() > ( bell_flags[x][5] + 3000 ) )  // wait flag set and time reached
        {
            if( block_input[19 - 4 * x][0] == 0 )  // N=0 is still set (US 15, UF 7)
            {
                block_locks[x][0] = 1;  // lock LC while Welwyn operated
                block_locks[x][6] = 1;  // set Welwyn N=0
            }
            else block_locks[x][6] = 0;  // N=0 not set, so ignore and reset flag
            printf("Line %d Welwyn status: ",x);
            for( y = 0; y < 10; y++ ) printf("%d ",block_locks[x][y]);
            printf("\r\n");
        }
        x = x + 2;
    }
    return 0;
}

int block_write()
{
    w = 0;  // change flag
    for( x = 0; x < 28; x++ )
    {
        // check for Rotary TOL release
        if( x == 21 && block_output[x][0] == 1 )  // Down slow TOL release on
            if( clock0() > bell_flags[2][4] + 350 ) { block_output[x][1] = 0; bell_flags[2][4] = 0; }  // turn TOL release off and reset time
        if( x == 10 && block_output[x][0] == 1 )  // Down fast TOL release on
            if( clock0() > bell_flags[4][4] + 350 ) { block_output[x][1] = 0; bell_flags[4][4] = 0; }  // turn TOL release off and reset time
        // check for change
        if( block_output[x][0] != block_output[x][1] )
        {
            // check for LC (not for Demo)
            //    Locked - change not allowed
            //    Free - set lock flag
            //    If 'commutator off' followed by 'commutator on' within 1 second (ie. contact bounce or looseness)
            //    then allow LC to be re-pegged
            if( mode == 11 || mode == 12 || mode == 15 )
            {
                z = 0;  // flag for LC commutator
                if( x == 5 ) { line = 1; b = 1; z = 1; }  // UP SLOW - check lever 2N (b for indicator)
                else if( x == 22 ) { line = 2; b = 22; z = 1; }  // DOWN SLOW - check lever 39N
                else if( x == 18 ) { line = 3; b = 6; z = 1; }  // UP FAST - check lever 8N
                else if( x == 12 ) { line = 4; b = 29; z = 1; }  // DOWN SLOW - check lever 43N
                if( z == 1 )
                  {
                    // switching off
                    if( block_output[x][1] == 0 ) block_output[x][2] = clock0();  // for checking for immediately on again
                    // switching on
                    //    more than 1 second from last 'off' (ie. genuine 'on', not bounce)
                    //    LC lock not checked for down lines (Rotary) - block_locks[0][0] is always 0
                    else if( clock0() > ( block_output[x][2] + 1000 ) && ( block_locks[line * (line % 2 )][0] == 1 || indicator_status[b][0] == 0 ) ) block_output[x][1] = 0;  // LC not allowed
                    else block_locks[line][0] = 1;  // LC allowed; set LC lock (may already be set if bounce)
                  }
            }
            // set flag for change
            if( block_output[x][0] != block_output[x][1] )
            {
                w = 1;
                block_output[x][0] = block_output[x][1];
            }
            // release starter one-pull locks if LC off (only necessary for mode 11)
            if( block_output[x][1] == 0 )
            {
                if( x == 2 ) block_locks[1][11] = 0;  // NS
                else if( x == 7 ) block_locks[2][11] = 0;  // HS
                else if( x == 15 ) block_locks[3][11] = 0;  // NF
                else if( x == 20 ) block_locks[4][11] = 0;  // HF
            }
        }
    }
    // changes
    if( w == 1 )
    {
        if( mode == 11 || mode == 12 ) lock_flag = 1;  // set flag to recalculate lever locks (full operation only)
        out_flag = 1;  // set output flag for serial-parallel output
    }
    return 0;
}

int process_bell()
{
    for( x = 1; x < 5; x++ )  // process each bell
    {
        // process if status has changed
        if( bell_status[x][0] != bell_status[x][1] )
        {
            // check: changes of status cannot be quicker than 50ms
            if( clock0() < bell_status[x][2] + 50 ) goto skipu1;
            //
            bell_status[x][2] = clock0();
            if( bell_status[x][1] == 0) bell_status[x][0] = 0;  // set and continue if tapper off
            else
            {
                bell_status[x][0] = 1;  // tapper on
                // if idle
                if( bell_status[x][3] == 0 )
                {
                    bell_status[x][3] = 2;  // set to 'receiving'
                    bell_status[x][12] = 1;  // set to 'receive'
                    bell_status[x][16] = 1;  // initialise count
                    bell_status[x][18] = clock0();  // record initial time
                }
                // if start of 'sending' response
                else if( bell_status[x][3] == 1 && (1 - bell_status[x][6] % 2) && bell_status[x][16] == 0 )  // ... count = 0
                {
                    bell_status[x][16] = 1;  // initialise count
                    bell_status[x][18] = clock0();  // record initial time
                }
                // if receiving in progress
                else if( (bell_status[x][3] == 1 && (1 - bell_status[x][6] % 2)) || (bell_status[x][3] == 2 && bell_status[x][12] % 2) )
                {
                    interval = clock0() - bell_status[x][17] - bell_status[x][18];  // ... minus accum, minus initial time (will = clock if count = 0 ie. start of status 3)
                    bell_status[x][18+bell_status[x][16]] = interval;  // record new interval (will be zero if count = 0)
                    if( bell_status[x][16] > 0) bell_status[x][17] = bell_status[x][17] + interval;  // increase accum, only if count > 0
                    bell_status[x][16]++;  // increase count
                }
            }
        }
        // check for end of receiving, and interpret code
        // in 'send' mode (% is modulus)
        if( bell_status[x][3] == 1 && bell_status[x][6] % 2 == 0 && bell_status[x][6] > 0 )
        {
            nexttrain = bell_status[x][4];  // save nexttrain
            // bell code 1 or 2
            if( bell_status[x][6] == 2 )
            {
                // no response after 10 seconds
                if( bell_status[x][16] == 0 && clock0() > bell_status[x][2] + 10000 )  // count = 0
                {
                    if( bell_status[x][7] == 1) for(y=0; y<28; y++) bell_status[x][y] = 0; // bell code 2, clear status
                    else
                    {
                        code = bell_status[x][7];  // save bell code
                        for(y=0; y<28; y++) bell_status[x][y] = 0; // clear status
                        // repeat bell code set up
                        bell_status[x][3] = 1;  // set bell to 'sending'
                        bell_status[x][4] = nexttrain;  // set nexttrain indicator
                        bell_status[x][7] = code;  // set bell code
                        printf("    repeat call attention\r\n");
                    }
                }
                // no input for 1 second, ie. end of receiving (2 seconds for demo)
                else if( bell_status[x][16] > 0 && clock0() > bell_status[x][2] + 1000 + demo * 1000 )
                {
                    bell_decode();
                    printf("%d-",x);
                    for(z=0; z<28; z++) printf("%d ",bell_status[x][z]);
                    printf("\r\n");
                    // repeat if incorrect code received
                    if( (bell_status[x][7] == 1 && bell_status[x][7] != code) || (bell_status[x][7] > 1 && code != 0) )  // expecting bell code 1 or 2
                    {
                        printf("    incorrect code received: %d\r\n",code);
                        code = bell_status[x][7];  // save bell code
                        for(y=0; y<28; y++) bell_status[x][y] = 0; // clear status
                        // repeat bell code set up
                        bell_status[x][3] = 1;  // set bell to 'sending'
                        bell_status[x][4] = nexttrain;  // set nexttrain indicator
                        bell_status[x][7] = code;  // set bell code
                    }
                    // call attention correctly answered: set up train bell code
                    else if( bell_status[x][7] > 1 )
                    {
                        code = bell_status[x][7];
                        bell_status[x][6] = 3;
                        // prepare bell-send data
                        bell_status[x][14] = clock0() + 500;  // set send time 0.5 second
                        bell_status[x][16] = 1;  // set count to 1
                        bell_status[x][17] = 0;  // start as off
                        for(y=18; y<28; y++) bell_status[x][y] = bellcode[code][y-14]; // copy bell intervals
                        printf("%d-",x);
                        for(z=0; z<28; z++) printf("%d ",bell_status[x][z]);
                        printf("\r\n");
                    }
                    // TES correctly answered: clear status
                    else
                    {
                        for(y=0; y<28; y++) bell_status[x][y] = 0;
                        // set 'last used' time for immediate response within 3 seconds
                        bell_flags[x][6] = clock0() + 3000;  // current line
                        if( x % 2 ) bell_flags[4 - x][6] = clock0() + 3000;  // other line, if x = 1 or 3
                        else bell_flags[6 - x][6] = clock0() + 3000;  // other line, if x = 2 or 4
                    }
                }
            }
            // bell code is a train or 2-1
            else
            {
                b = 0;  // repeat flag
                // no response after 5 seconds
                if( bell_status[x][16] == 0 && clock0() > bell_status[x][2] + 5000 )  // count = 0
                {
                    if( bell_status[x][7] == 2 ) b = 1;  // repeat 2-1
                    else if( bell_status[x][6] == 8 )  // repeating train code for LC
                    {
                        // merely clear status, no need for response
                        for(y=0; y<28; y++) bell_status[x][y] = 0; // clear status
                    }
                    else  // refusal
                    {
                        trains[nexttrain][2][0] = 101;  // set status to refused
                        trains[nexttrain][4][0] = clock0() + 45000;  // reset start time (ie. repeat in 45 seconds)
                        bell_flags[x][1] = bell_status[x][7];  // set refusal flag by saving code
                        bell_flags[x][2] = bell_status[x][4];  // ... and save nexttrain
                        for(y=0; y<28; y++) bell_status[x][y] = 0; // clear status
                        printf ("%03.1f code %d %d-%d refused\r\n",(fx=clock0())/1000,bellcode[trains[nexttrain][0][0]][1],trains[nexttrain][1][0],trains[nexttrain][2][0]);
                    }
                }
                // no input for 1 second, ie. end of receiving (2 seconds for demo)
                else if( bell_status[x][16] > 0 && clock0() > bell_status[x][2] + 1000 + demo * 1000 )
                {
                    bell_decode();
                    printf("%d-",x);
                    for(z=0; z<28; z++) printf("%d ",bell_status[x][z]);
                    printf("\r\n");
                    // demo: force correct code so that no repeat
                    if( demo == 1 ) code = bell_status[x][7];
                    // repeat if incorrect code received
                    if( !( bell_status[x][7] == code || ( bell_status[x][7] == 2 && code == 18 ) ) ) b = 1;  // not correct code and not 2-1-1
                    // correct response received
                    else if( bell_status[x][7] == 2 || code == 18 )  // bell code 2-1 (or 2-1-1)
                    {
                        trains[nexttrain][2][0] = 800;  // TOS sent and acknowledged
                        printf ("%03.1f code %d %d-%d\r\n",(fx=clock0())/1000,bellcode[trains[nexttrain][0][0]][1],trains[nexttrain][1][0],trains[nexttrain][2][0]);
                        for(y=0; y<28; y++) bell_status[x][y] = 0; // clear status
                        // set 'last used' time for immediate response within 3 seconds
                        bell_flags[x][6] = clock0() + 3000;  // current line
                        if( x % 2 ) bell_flags[4 - x][6] = clock0() + 3000;  // other line, if x = 1 or 3
                        else bell_flags[6 - x][6] = clock0() + 3000;  // other line, if x = 2 or 4
                        // if 2-1-1, set up bell receive
                        if( code == 18 )
                        {
                            code = 0;  // call attention
                            bell_status[x][3] = 2;  // set to receiving
                            bell_status[x][12] = 2;  // set reply status
                            bell_status[x][14] = clock0() + 500;  // set reply time 0.5 second
                            // prepare bell-send data
                            bell_status[x][16] = 1;  // set count to 1
                            bell_status[x][17] = 0;  // start as off
                            for(y=18; y<28; y++) bell_status[x][y] = bellcode[code][y-14]; // copy bell intervals for code 0
                            printf("%d-",x);
                            for(z=0; z<28; z++) printf("%d ",bell_status[x][z]);
                            printf("\r\n");
                        }
                    }
                    else  // train bell code
                    {
                        trains[nexttrain][2][0] = 100;  // train code sent and acknowledged
                        trains[nexttrain][4][0] = clock0();  // allow time to check for LC
                        printf ("%03.1f code %d %d-%d %03.1fs\r\n",(fx=clock0())/1000,bellcode[trains[nexttrain][0][0]][1],trains[nexttrain][1][0],trains[nexttrain][2][0],(fy=trains[nexttrain][5][0])/1000);
                        for(y=0; y<28; y++) bell_status[x][y] = 0; // clear status
                        // set 'last used' time for immediate response within 3 seconds
                        bell_flags[x][6] = clock0() + 3000;  // current line
                        if( x % 2 ) bell_flags[4 - x][6] = clock0() + 3000;  // other line, if x = 1 or 3
                        else bell_flags[6 - x][6] = clock0() + 3000;  // other line, if x = 2 or 4
                    }
                }
                if( b == 1 )  // repeat code
                {
                    code = bell_status[x][7];
                    bell_status[x][6] = 3;   // set to send
                    bell_status[x][14] = clock0() + 500;  // set send time 0.5 second
                    bell_status[x][16] = 1;  // set count to 1
                    bell_status[x][17] = 0;  // start as off
                    for(y=18; y<28; y++) bell_status[x][y] = bellcode[code][y-14]; // copy bell intervals
                    printf("%d-",x);
                    for(z=0; z<28; z++) printf("%d ",bell_status[x][z]);
                    printf("\r\n");
                }
            }
        }
        // in 'receive' mode (% is modulus)
        else if( (bell_status[x][3] == 2 && bell_status[x][12] % 2 == 1) )
        {
            // no response when expecting train code
            if( bell_status[x][12] == 3 && bell_status[x][16] == 0 )  // count is zero, ie. code not started
            {
                // back to idle after 5 seconds
                if( clock0() > bell_status[x][2] + 5000 )
                {
                    for(y=12; y<28; y++) bell_status[x][y] = 0; // reset bell intervals etc.
                    bell_status[x][3] = 0;  // set to idle
                    printf("bell receive time-out: %d\r\n",x);
                }
            }
            // no input for 1 second, ie. end of receiving (2 seconds for demo)
            else if( clock0() > bell_status[x][2] + 1000 + demo * 1000 )
            {
                bell_decode();
                bell_status[x][13] = code;
                printf("%d-",x);
                for(z=0; z<28; z++) printf("%d ",bell_status[x][z]);
                printf("\r\n");
                // expecting bell code 1 or 2 or 6
                if( bell_status[x][12] == 1 )
                {
                    if( code == 1 )  // first check for LC if code is 2; if not, error
                    {
                        if( x == 1 && block_output[7][0] != 1 ) code = -1;
                        else if( x == 2 && block_output[2][0] != 1 ) code = -1;
                        else if( x == 3 && block_output[20][0] != 1 ) code = -1;
                        else if( x == 4 && block_output[15][0] != 1 ) code = -1;
                    }
                    if( code != 0 && code != 1 && code != 17 )  // not 1 or 2 or 6
                    {
                        // check for acceptance of refused train
                        if( demo == 1 && bell_flags[x][1] > 0 ) code = bell_flags[x][1];  // demo: force correct code
                        if( bell_flags[x][1] == code )
                        {
                            b = bell_flags[x][2];  // this is 'nexttrain' for the refused train
                            bell_flags[x][1] = 0;
                            bell_flags[x][2] = 0;  // reset refusal flags
                            trains[b][2][0] = 100;  // train code now accepted
                            trains[b][4][0] = clock0();  // allow time to check for LC
                            printf ("%03.1f code %d %d-%d (was 101) %03.1fs\r\n",(fx=clock0())/1000,bellcode[trains[b][0][0]][1],trains[b][1][0],trains[b][2][0],(fy=trains[b][5][0])/1000);
                            for(y=0; y<28; y++) bell_status[x][y] = 0; // clear status
                        }
                        // error if not these codes
                        else
                        {
                            for(y=12; y<28; y++) bell_status[x][y] = 0; // zeroise record
                            bell_status[x][3] = 3;  // set error status
                            bell_status[x][2] = clock0();
                            printf("%03.1f %d - bell code error\r\n",(fx=clock0())/1000,x);
                            goto skipu1;
                        }
                    }
                    else
                    {
                        bell_status[x][12] = 2;  // set reply status
                        bell_status[x][14] = clock0() + 500 + (clock0() > bell_flags[x][6]) * (2500 + random2(4000));  // set reply time between 3 and 7 seconds
                        // reset if there is a refusal, as call attention now needed to re-send
                        if( bell_flags[x][1] > 0 )
                        {
                            b = bell_flags[x][2];  // this is 'nexttrain' for the refused train
                            bell_flags[x][1] = 0;
                            bell_flags[x][2] = 0;  // reset refusal flags
                            trains[b][2][0] = 0;  // return to start status
                            printf ("%03.1f code %d %d-%d (was 101)\r\n",(fx=clock0())/1000,bellcode[trains[b][0][0]][1],trains[b][1][0],trains[b][2][0]);
                        }
                    }
                }
                // expecting train code or 2-1 or 2-4 or 3-5 (ie. bell_status[x][12] = 3)
                else
                {
                    if( code < 2 || code == 17) code = -1;  // error if bell code 1 or 2 or 6
                    else if( code == 2 || code == 16 )  // if 2-1 or 2-4, check not LC
                    {
                        if( x == 1 && block_output[5][0] == 1 ) code = -1;
                        else if( x == 2 && block_output[22][0] == 1 ) code = -1;
                        else if( x == 3 && block_output[18][0] == 1 ) code = -1;
                        else if( x == 4 && block_output[12][0] == 1 ) code = -1;
                    }
                    else if( code > 2 && code < 16)  // if train code, check for Line Blocked
                    {
                        if( x == 1 && (block_output[6][0] != 0 || block_output[7][0] != 0)) code = -1;
                        else if( x == 2 && (block_output[1][0] != 0 || block_output[2][0] != 0)) code = -1;
                        else if( x == 3 && (block_output[19][0] != 0 || block_output[20][0] != 0)) code = -1;
                        else if( x == 4 && (block_output[14][0] != 0 || block_output[15][0] != 0)) code = -1;
                    }
                    else if( code == 19 )  // code 3-5 cancelling
                    {
                        if( check_cancel() ) code = -1;  // cannot accept cancelling, set error
                        else  // accept cancelling
                        {
                            if( x == 2 )
                            {
                                block_output[1][1] = 0;    // up slow
                                block_output[2][1] = 0;
                            }
                            else if( x == 1 )
                            {
                                block_output[6][1] = 0;    // down slow
                                block_output[7][1] = 0;
                            }
                            else if( x == 4 )
                            {
                                block_output[14][1] = 0;    // up fast
                                block_output[15][1] = 0;
                            }
                            else if( x == 3 )
                            {
                                block_output[19][1] = 0;    // down fast
                                block_output[20][1] = 0;
                            }
                            bell_flags[x][3] = clock0();
                            printf("%03.1f line %d cancelling\r\n",(fx=clock0()/1000),b);
                        }
                    }
                    if( code == -1 )  // ie. error
                    {
                        for(y=12; y<28; y++) bell_status[x][y] = 0; // zeroise record
                        bell_status[x][3] = 3;  // set error status
                        bell_status[x][2] = clock0();
                        printf("%03.1f %d - bell code error\r\n",(fx=clock0())/1000,x);
                        goto skipu1;
                    }
                    else
                    {
                        bell_status[x][12] = 4;  // set reply status
                        bell_status[x][14] = clock0() + 500;  // set reply time 0.5 second
                    }
                }
                // prepare bell-send data
                bell_status[x][16] = 1;  // set count to 1
                bell_status[x][17] = 0;  // start as off
                for(y=18; y<28; y++) bell_status[x][y] = bellcode[code][y-14]; // copy bell intervals
                printf("%d-",x);
                for(z=0; z<28; z++) printf("%d ",bell_status[x][z]);
                printf("\r\n");
            }
        }
        // start send in 'send' mode
        if( bell_status[x][3] == 1 && bell_status[x][6] == 0 && bell_flags[x][0] == 0 )  // send ready, but not started and not 'wait'
        {
            if( bell_status[x][7] == 1 ) code = 1;  // bell code is 2
            else code = 0;  // call attention
            bell_status[x][6] = 1;
            // prepare bell-send data
            bell_status[x][14] = clock0() + 500;  // set send time 0.5 second
            bell_status[x][16] = 1;  // set count to 1
            bell_status[x][17] = 0;  // start as off
            for(y=18; y<28; y++) bell_status[x][y] = bellcode[code][y-14]; // copy bell intervals
            printf("%d-",x);
            for(z=0; z<28; z++) printf("%d ",bell_status[x][z]);
            printf("\r\n");
            if( bell_flags[x][1] > 0 ) bell_flags[x][1] = 0;
            bell_flags[x][2] = 0;  // reset refused flags
        }

        // ring bell, and check for end of bell code
        if( bell_ring() )  // 0 - continue; 1 - finished
        {
            // in 'send' mode (% is modulus)
            if( (bell_status[x][3] == 1 && bell_status[x][6] % 2 == 1) )
            {
                bell_status[x][6] = bell_status[x][6] + 1;  // switch to response
                for(y=16; y<28; y++) bell_status[x][y] = 0; // reset bell intervals etc.
                bell_status[x][2] = clock0();  // so able to check for time-out
            }
            // in 'receive' mode (% is modulus)
            else if( (bell_status[x][3] == 2 && bell_status[x][12] % 2 == 0) )
            {
                code = bell_status[x][13];  // replying to this code received
                if( code == 1 )  // bell code 2 - set TOL, release LC
                {
                    if( x == 1 )
                    {
                        block_output[6][1] = 1;
                        block_output[7][1] = 0;
                    }
                    else if( x == 2 )
                    {
                        block_output[1][1] = 1;
                        block_output[2][1] = 0;
                    }
                    else if( x == 3 )
                    {
                        block_output[19][1] = 1;
                        block_output[20][1] = 0;
                    }
                    else if( x == 4 )
                    {
                        block_output[14][1] = 1;
                        block_output[15][1] = 0;
                    }
                }
                else if( code > 2 && code < 16 )  // bell code is a train - set LC
                {
                    if( x == 1 ) block_output[7][1] = 1;
                    else if( x == 2 ) block_output[2][1] = 1;
                    else if( x == 3 ) block_output[20][1] = 1;
                    else if( x == 4 ) block_output[15][1] = 1;
                }
                for(y=12; y<28; y++) bell_status[x][y] = 0; // reset bell intervals etc.
                if( code == 0 )
                {
                    bell_status[x][12] = 3;  // bell code 1 - set to receive bell code
                    bell_status[x][2] = clock0();  // so able to check for time-out
                }
                else
                {
                    for(z=0; z<28; z++) bell_status[x][z] = 0; // set to idle; clear status
                    // set 'last used' time for immediate response within 6 seconds
                    bell_flags[x][6] = clock0() + 6000;  // current line
                    if( x % 2 ) bell_flags[4 - x][6] = clock0() + 6000;  // other line, if x = 1 or 3
                    else bell_flags[6 - x][6] = clock0() + 6000;  // other line, if x = 2 or 4
                }
            }
        }
        // if error on receiving, or no answer to repeat for no LC
        // ... reset to idle after 5 seconds
        if( ( bell_status[x][3] == 3 || ( bell_status[x][3] == 1 && bell_status[x][6] == 8 ) ) &&
                clock0() > bell_status[x][2] + 5000 )
        {
            for(z=0; z<28; z++) bell_status[x][z] = 0; // set to idle; clear status
            printf("%03.1f %d - bell error reset\r\n",(fx=clock0())/1000,x);
        }

skipu1:
        // set wait flag for bell on other line
        if( x > 2 ) y = x - 2; else y = x + 2;
        if( bell_status[x][3] > 0 && bell_flags[x][0] == 0 ) bell_flags[y][0] = 1; else bell_flags[y][0] = 0;
        // loop to next x value
    }
    return 0;
}

int bell_decode()
{
    if(bell_status[x][16] == 1)   // 'call attention' code 0
    {
        code = 0;
        printf("\r\n%03.1f bell %d code received: 1\r\n",(fx=clock0())/1000,x);
        return 0;
    }
    else if (bell_status[x][16] == 2)   // 'train entering section' code 1
    {
        code = 1;
        printf("\r\n%03.1f bell %d code received: 2\r\n",(fx=clock0())/1000,x);
        return 0;
    }
    else   // find minimum interval; build binary: ratio to min. < 1.5 = short(0); > 1.5 = long(1)
    {
        min = bell_status[x][19] + 1;  // first interval ( + 1 so no danger of divide by 0 )
        for (y=2; y<bell_status[x][16]; ++y) if (bell_status[x][18+y] < min) min = bell_status[x][18+y] + 1;
        code = 0;
        printf("\r\n  min %d\r\n",min);
        for (y=1; y<bell_status[x][16]; ++y)
        {
            fx = bell_status[x][18+y];
            fy = min;
            printf("  ratio %f\r\n",fx/fy);
            code = code * 2 + ((fx/fy) > 1.5);
        }
    }
    if(bell_status[x][16] == 3 && code < 2) code = 2;    // 'train out of section' code 2 (accepts 2-1 or 3)
    else if (bell_status[x][16] == 4) code = code + 10;
    else if (bell_status[x][16] == 5) code = code + 20;
    else if (bell_status[x][16] == 6) code = code + 40;
    else if (bell_status[x][16] == 8) code = code + 50;
    else if (bell_status[x][16] == 10 && code == 80) code = 51;
    else
    {
        printf("invalid code: count %d, code %d\r\n",bell_status[x][16],code);
        code = -1;   // code is invalid
        return 0;
    }
    for (y=0; y<20; ++y)  // convert into ref. from bellcode table
    {
        if (bellcode[y][0] == code)
        {
            code = y;
            printf("\r\n%03.1f bell %d code received: %d\r\n",(fx=clock0())/1000,x,bellcode[y][1]);  // real bell code
            return 0;
        }
    }
    printf("invalid code: count %d, code %d\r\n",bell_status[x][16],code);
    code = -1;   // code is invalid
    return 0;
}

int bell_ring()  // run from within 'process_bell' for bell 'x'
{
    // check if sending (% is modulus)
    if( ((bell_status[x][3] == 1) && (bell_status[x][6] % 2 == 1)) || ((bell_status[x][3] == 2) && (bell_status[x][12] % 2 == 0)) )
    {
        // if pulse is off, and time for next pulse reached
        if( bell_status[x][17] == 0 && clock0() > bell_status[x][14] )
        {
            // turn pulse on
            if( x == 1 ) block_output[3][1] = 1;
            else if( x == 2 ) block_output[0][1] = 1;
            else if( x == 3 ) block_output[16][1] = 1;
            else if( x == 4 ) block_output[13][1] = 1;
            // set status to on
            bell_status[x][17] = 1;
            printf("%03.1f Bell ring: %d   ",(fx=clock0())/1000,x);
        }
        // if pulse is on, and pulse length time reached
        // Harpenden fast 125ms, slow 100ms, Napsbury fast 75ms, slow 100ms
        else if( bell_status[x][17] == 1 && clock0() > ( bell_status[x][14] + 100 + (x/3)*25 - (x/4)*50 ) )
        {
            // turn pulse off
            if( x == 1 ) block_output[3][1] = 0;
            else if( x == 2 ) block_output[0][1] = 0;
            else if( x == 3 ) block_output[16][1] = 0;
            else if( x == 4 ) block_output[13][1] = 0;
            // check for end of bell code
            if( bell_status[x][bell_status[x][16]+17] == 99 ) return 1;  // finished
            else  // more pulses to do
            {
                bell_status[x][17] = 0;  // set status to off
                // set time for next pulse - 20% longer for Harpenden, 30% longer for Napsbury and 60% longer for Demo
                bell_status[x][14] = bell_status[x][14] + ( bell_status[x][bell_status[x][16]+17] * ( 120 + ( (1 - (x % 2)) || demo ) * 10 + demo * 30 ) ) / 100;
                bell_status[x][16] = bell_status[x][16] + 1;  // increase count
            }
        }
    }
    return 0;
}

int check_cancel()  // run from within 'process_bell' for bell 'x'
{
    // opposite line from bell
    if( x % 2 == 1 ) b = x + 1;
    else b = x - 1;
    // check for train with status 600
    y = 0;
    while( y < 14 )
    {
        if( trains[y][0][0] > 0 )
        {
            if( trains[y][3][0] == b && trains[y][2][0] == 600 ) return 1;
        }
        y = y + 1;
    }
    // line 1 (NS)
    if( x == 2 )
    {
        // block needs to be LC or TOL
        if( block_output[1][0] == 0 && block_output[2][0] == 0 ) return 1;
        // starter needs to be at danger
        if( lever_status[18][0] == 0 ) return 1;
    }
    // line 2 (HS)
    else if( x == 1 )
    {
        // block needs to be LC or TOL
        if( block_output[6][0] == 0 && block_output[7][0] == 0 ) return 1;
        // IB needs to be at danger
        if( lever_status[58][0] == 0 ) return 1;
        // TC13 needs to be clear
        if( TC_status[13][0] == 1 ) return 1;
    }
    // line 3 (NF)
    else if( x == 4 )
    {
        // block needs to be LC or TOL
        if( block_output[14][0] == 0 && block_output[15][0] == 0 ) return 1;
        // starter needs to be at danger
        if( lever_status[22][0] == 0 ) return 1;
    }
    // line 4 (HF)
    else if( x == 3 )
    {
        // block needs to be LC or TOL
        if( block_output[19][0] == 0 && block_output[20][0] == 0 ) return 1;
        // IB needs to be at danger
        if( lever_status[66][0] == 0 ) return 1;
        // TC23 needs to be clear
        if( TC_status[23][0] == 1 ) return 1;
    }
    return 0;  // accept cancelling
}

int process_lever()
{
    for(x=0; x<74; x++)
    {
        // lever is in Error
        if( lever_status[x][17] == 9 )
        {
            // error is both N and R; check that status is restored to as before error
            if( lever_status[x][0] == lever_status[x][1] && lever_status[x+1][0] == lever_status[x+1][1] )
            {
                // reset error
                lever_status[x][17] = 0;
                lever_status[x+1][17] = 0;
                annun_status[1][1] = 0;  // turn annunciator off
                printf("lever error reset, lever ref: %d\r\n",x);
            }
            x = x + 1;  // increase loop counter because error'd in pairs
        }
        // check lever status for change
        else if( lever_status[x][0] != lever_status[x][1] )
        {
            // convert lever ref. into actual lever number, for printing
            y = x;
            if( y > 23 ) y = y + 4;  // skip lever 13 & 14
            if( y > 33 ) y = y + 2;  // skip lever 18      (29+4)
            if( y > 41 ) y = y + 2;  // skip lever 22      (35+6)
            if( y > 45 ) y = y + 2;  // skip lever 24      (37+8)
            if( y > 61 ) y = y + 2;  // skip lever 32      (51+10)
            if( y > 67 ) y = y + 2;  // skip lever 35      (55+12)
            z = y % 2;  // modulus: 0 = N, 1 = R
            y = (y + 2) / 2;  // gives lever number

            // check lever locks
            if( lever_status[x][16] == 0 && lever_status[x][1] == 0 )  // lever is locked, and is being moved from N or R
            {
                printf("ERROR - lever %d",y);
                if( z == 0 ) printf("N ");
                else printf("R ");
                printf("is locked. Restore lever to reset.\r\n\a");
                annun_status[1][1] = 1;  // turn annunciator on
                lever_status[x][17] = 9;  // set Error flag
                if( (x % 2) == 0 )
                {
                    lever_status[x+1][17] = 9;    // lever N so set next R also
                    x = x + 1;
                }
                else lever_status[x-1][17] = 9;  // lever R so set previous N also (ie. FPLs)
            }
            // process normally
            else
            {
                lever_status[x][0] = lever_status[x][1];  // set new status of lever
                if( lever_status[x][15] > -1 )  // an indicator is impacted
                {
                    // for Distants, check Homes are 'off'
                    b = 0;
                    if( x == 1 && !( indicator_status[2][0] == 1 && indicator_status[10][0] == 1 ) ) b = 1;  // lever 1, indicators 2 & 10
                    else if( x == 13 && !( indicator_status[7][0] == 1 && indicator_status[12][0] == 1 ) ) b = 1;  // lever 7, indicators 8 & 12
                    else if( x == 65 && !( indicator_status[21][0] == 1 && indicator_status[23][0] == 1 ) ) b = 1;  // lever 40, indicators 38 & 39
                    else if( x == 73 && !( indicator_status[28][0] == 1 && indicator_status[30][0] == 1 ) ) b = 1;  // lever 44, indicators 42 & 43
                    if( b == 1 ) indicator_status[lever_status[x][15]][13] = 1;  // distant is Held 'on'
                    else  // set indicator
                    {
                        indicator_status[lever_status[x][15]][1] = lever_status[x][1];  // set new status in indicator_status
                        indicator_status[lever_status[x][15]][3] = clock0();  // for calculating delay in reaction of indicator
                    }
                }
                if( ( mode == 11 || mode == 12 ) && lever_status[x][1] == 0 )  // starters moved from normal (full operation only)
                {
                    // one pull only for starters; locked in calc_locks
                    if( x == 18 ) block_locks[1][11] = 1;  // lever 10
                    else if( x == 58 ) block_locks[2][11] = 1;  // lever 37
                    else if( x == 22 ) block_locks[3][11] = 1;  // lever 12
                    else if( x == 66 ) block_locks[4][11] = 1;  // lever 41
                }
                printf("Lever %d",y);
                if( z == 0 ) printf("N ");
                else printf("R ");
                if( lever_status[x][0] == 0 ) printf("Off\r\n");
                else printf("On\r\n");
            }
            if( mode == 11 || mode == 12 ) lock_flag = 1;  // set flag to recalculate lever locks (full operation only)
            // special case: flag to start demo
            if( x == 34 && lever_status[x][1] == 0 ) lever21 = 1;  // lever 21N off
        }
        // start demo
        if( lever21 == 1 || input_data[78] == 1 )     // lever 21N off or plunger 21 pressed
        {
            lever21 = 0;
            if( mode == 13 ) trainqueue[0][1] = 101;  // start Up slow demo
            if( mode == 14 ) trainqueue[0][3] = 101;  // start Up fast demo
        }
    }
    return 0;
}

int process_indicator()
{
    // process indicators
    for(x=0; x<34; x++)
    {
        // process dummy for 16G in garden
        if( x == 33 )  // end of regular loop is 32
        {
            x = 39;    // reference for 16G dummy
            indicator_status[39][1] = indicator_status[10][0];   // set status to be same as 10 off
            indicator_status[39][3] = clock();  // for calculating delay in reaction of indicator
        }
        // normal process - has status changed?
        if( indicator_status[x][0] != indicator_status[x][1]  // has status changed?
         || ( indicator_status[x][2] == 6 && indicator_status[x][4] > 0 ) )  // or, N-R in progress
        {
            // indicator is held ON; reset status to 0
            if( indicator_status[x][1] == 1 && indicator_status[x][13] == 1 ) indicator_status[x][1] = 0;
            // normal indicator
            else if( indicator_status[x][2] < 4 )
            {
                if ( clock0() > (indicator_status[x][3] + indicator_status[x][5]) )  // delay time reached?
                {
                    out_flag = 1;   // set flag to trigger serial output
                    indicator_status[x][0] = indicator_status[x][1];  // set new status of indicator
                    if( indicator_status[x][2] == 2 )  // also BR
                    {
                        // only change BR if not: to OFF but held or TC occupied
                        b = ( 26 * x - 42 ) / 10;  // x = 2 is TC 1; x = 7 is TC 14
                        if( !( indicator_status[x][1] == 1 && ( indicator_status[x][13] == 1 || TC_status[b][0] == 1 ) ) )
                        {
                            indicator_status[x+1][1] = indicator_status[x][1];  // copy status to BR
                            indicator_status[x+1][3] = indicator_status[x][3];  // copy clock to BR
                        }
                    }
                    if( indicator_status[x][2] == 3 )  // also Ind 25
                    {
                        indicator_status[x-8][1] = indicator_status[x][1];  // copy status to 25
                        indicator_status[x-8][3] = indicator_status[x][3];  // copy clock to 25
                    }
                    if( indicator_status[x][1] == 0 )  // check for Homes 'On' to reset Distant
                    {
                        // if home or starter no longer 'Off' and distant is 'Off', set distant 'On' and Held
                        if( (x == 2 || x == 10) && indicator_status[0][0] == 1 ) { indicator_status[0][1] = 0; indicator_status[0][3] = clock0(); indicator_status[0][13] = 1; }  // homes 2 & 10, distant 1
                        else if( (x == 7 || x == 12) && indicator_status[5][0] == 1 ) { indicator_status[5][1] = 0; indicator_status[5][3] = clock0(); indicator_status[5][13] = 1; }  // homes 8 & 12, distant 7
                        else if( (x == 21 || x == 23) && indicator_status[24][0] == 1 ) { indicator_status[24][1] = 0; indicator_status[24][3] = clock0(); indicator_status[24][13] = 1; }  // homes 38 & 39, distant 40
                        else if( (x == 28 || x == 30) && indicator_status[31][0] == 1 ) { indicator_status[31][1] = 0; indicator_status[31][3] = clock0(); indicator_status[31][13] = 1; }  // homes 42 & 43, distant 44
                    }
                }
            }
            // colour-light
            else if( indicator_status[x][2] == 4 || indicator_status[x][2] == 5 )
            {
                if( indicator_status[x][4] == 0 ) indicator_status[x][4] = 9 - 4 * indicator_status[x][1];  // set stage to 5 for OFF, 9 for ON
                if( clock0() > (indicator_status[x][3] + indicator_status[x][indicator_status[x][4]]) )  // delay time reached?
                {
                    out_flag = 1;   // set flag to trigger serial output
                    if( indicator_status[x][4] == 5 || indicator_status[x][4] == 9 )
                    {
                        indicator_status[x][4] = indicator_status[x][4] + 2;  // set to 2nd stage
                        indicator_status[x][16] = 0;  // set serial output status to 'Off'
                    }
                    else
                    {
                        indicator_status[x][0] = indicator_status[x][1];    // finished, set new status of indicator, reset stage flag
                        if( indicator_status[x][4] == 7 ) indicator_status[x][16] = 2;  // set serial output status to 'Green'
                        else indicator_status[x][16] = 1;  // set serial output status to 'Red/Yellow'
                        indicator_status[x][4] = 0;
                        if( indicator_status[x][1] == 0 ) indicator_status[x][13] = 1;  // Held ON
                    }
                    if( indicator_status[x][2] == 5 )  // also IB
                    {
                        indicator_status[x+1][1] = indicator_status[x][1];  // copy status to IB
                        indicator_status[x+1][3] = clock0();  // set clock for IB
                    }
                }
            }
            // points N-R
            else if( indicator_status[x][2] == 6 )
            {
                if( indicator_status[x][4] == 0 )  // ready to switch
                {
                  indicator_status[x][12] = indicator_status[x][1];  // save change state for duration of change
                  // switch on: set stage to 5 for N, 7 for R
                  if( indicator_status[x][1] == 1 )
                  {
                    // depends on lever normal or reversed
                    if( x == 4 ) indicator_status[x][4] = 5 * lever_status[6][0] + 7 * lever_status[7][0];  // lever 4
                    else if( x == 16 ) indicator_status[x][4] = 5 * lever_status[46][0] + 7 * lever_status[47][0];  // lever 29
                    else if( x == 17 ) indicator_status[x][4] = 5 * lever_status[52][0] + 7 * lever_status[53][0];  // lever 33
                    // error, no change if stage is not 5 or 7  (lever not properly normal or reversed)
                    if( indicator_status[x][4] != 5 && indicator_status[x][4] != 7 )
                    {
                        indicator_status[x][4] = 0;  // stage to zero in case both 5 and 7 (ie. lever switch is faulty)
                        indicator_status[x][0] = indicator_status[x][1];
                    }
                  }
                  // switch off, according to which is on
                  else
                  {
                    if( indicator_status[x][9] == 1 ) indicator_status[x][4] = 5;  // set stage to 5 for N
                    else if( indicator_status[x][11] == 1 ) indicator_status[x][4] = 7;  // set stage to 7 for R
                    else indicator_status[x][0] = indicator_status[x][1];  // no change required as no light is on
                  }
                }
                else if ( clock0() > (indicator_status[x][3] + indicator_status[x][indicator_status[x][4]]) )  // delay time reached?
                {
                    out_flag = 1;   // set flag to trigger serial output
                    indicator_status[x][0] = indicator_status[x][12];  // set new status of indicator
                    if( indicator_status[x][12] == 1 ) indicator_status[x][indicator_status[x][4]+4] = 1;  // set flag to 'on'
                    else indicator_status[x][indicator_status[x][4]+4] = 0;  // set flag to 'off'
                    indicator_status[x][4] = 0;  // reset stage flag
                }
            }
            // points 17R
            else if( indicator_status[x][2] == 7 )
            {
                // turn on
                if( indicator_status[x][1] == 1 )
                {
                    if( lever_status[29][0] == 0 ) indicator_status[x][0] = indicator_status[x][1];  // 29N - no change required
                    else if( clock0() > (indicator_status[x][3] + indicator_status[x][5]) )
                    {
                        out_flag = 1;   // set flag to trigger serial output
                        indicator_status[x][0] = indicator_status[x][1];  // set new status of indicator
                        indicator_status[x][9] = 1;  // set flag to 'on'
                    }
                }
                // turn off
                else
                {
                    if( indicator_status[x][9] == 0 ) indicator_status[x][0] = indicator_status[x][1];  // indicator off - no change required
                    else if( clock0() > (indicator_status[x][3] + indicator_status[x][5]) )
                    {
                        out_flag = 1;   // set flag to trigger serial output
                        indicator_status[x][0] = indicator_status[x][1];  // set new status of indicator
                        indicator_status[x][9] = 0;  // set flag to 'off'
                    }
                }
            }
        }
        // reset if held
        if( indicator_status[x][13] == 1 )
        {
            // check whether IB distants need to remain held
            b = 0;
            if( x == 19 && TC_status[12][0] == 1 ) b = 1;  // R37 and TC12
            else if( x == 26 && TC_status[22][0] == 1 ) b = 1;  // R41 and TC22
            // general reset, unless b = 1 from above
            if( lever_status[indicator_status[x][14]][0] == 1 && b == 0 ) indicator_status[x][13] = 0;
        }
    }
    // process pseudo indicators needed by 'move_train' to control TCs
    // levers 6 9 11
    for(x=33; x<36; x++)
    {
        // if link status changed, set new status
        if( indicator_status[x][0] != indicator_status[x][1] )
            indicator_status[x][0] = indicator_status[x][1];
    }
    // levers 25 36 39
    if( lever_status[39][0] == 1 && indicator_status[23][0] == 1 ) indicator_status[36][0] = 1;
    else indicator_status[36][0] = 0;
    if( lever_status[57][0] == 1 && indicator_status[23][0] == 1 ) indicator_status[37][0] = 1;
    else indicator_status[37][0] = 0;
    if( lever_status[63][0] == 1 && indicator_status[23][0] == 1 ) indicator_status[38][0] = 1;
    else indicator_status[38][0] = 0;

    // process TCs
    for(x=1; x<24; x++)
    {
        if( TC_status[x][0] != TC_status[x][1] )  // has link status changed?
        {
            // split TCs for IBs
            // ... if TC12 or TC22 ON, do not toggle
            // ... if TC11 or TC21 OFF, do not toggle
            if( ! ( ((x == 12 || x == 22) && TC_status[x][1] == 1) ||
                    ((x == 11 || x == 21) && TC_status[x][1] == 0) ) )
                out_flag = 1;   // set flag to trigger serial output
            TC_status[x][0] = TC_status[x][1];  // set new status
            printf("TC %d",x);
            if( TC_status[x][1] == 0 ) printf(" Off\r\n");
            else printf(" On\r\n");
            // set signals to danger/caution
            if( TC_status[x][1] == 1 && TC_status[x][3] != -1 )
            {
                indicator_status[TC_status[x][3]][1] = 0;  // signals
                indicator_status[TC_status[x][3]][3] = clock0();  // for calculating delay in reaction of indicator
                indicator_status[TC_status[x][3]][13] = 1;  // held ON (needed for distants if they had not been cleared)
                if( TC_status[x][4] > 0 )
                {
                    indicator_status[TC_status[x][4]][1] = 0;  // BRs
                    indicator_status[TC_status[x][4]][3] = clock0();  // for calculating delay in reaction of indicator
                    // indicator_status[TC_status[x][4]][13] = 1;  // held ON
                }
            }
            // set block to TOL (do it anyway)
            if( x == 1 )
            {
                block_output[4][1] = 1;    // Up slow - HS
                block_output[5][1] = 0;
            }
            else if( x == 7 )
            {
                block_output[9][1] = 1;    // Down slow - NS
                block_output[22][1] = 0;
            }
            else if( x == 14 )
            {
                block_output[17][1] = 1;    // Up fast - HF
                block_output[18][1] = 0;
            }
            else if( x == 19 )
            {
                block_output[11][1] = 1;    // Down fast - NF
                block_output[12][1] = 0;
            }
            if( mode == 11 || mode == 12 )  // for full operation only
            {
                lock_flag = 1;  // set flag to recalculate lever locks
                block_flag = 1;  // set flag for block_lock calculation
            }
            // IBS bells
            if( x == 13 && TC_status[x][1] == 1 ) IBS_flag[0] = 1;  // DS bell
            if( x == 23 && TC_status[x][1] == 1 ) IBS_flag[1] = 1;  // DF bell
        }
    }
    return 0;
}

int IBS_bells()
{
    // flags: 0 - DS; 1 - DF
    //        2 - DS timer; 3 - DF timer
    for( x=0;x<2;x++ )
    {
        if( IBS_flag[x] == 1 )                  // flag set for IBS bell
        {
            if( IBS_flag[x+2] == 0 )            // bell not started
            {
                block_output[26+x][1] = 1;      // switch bell on
                IBS_flag[x+2] = clock0() + 500;  // set timer
            }
            else if( IBS_flag[x+2] < clock0() )  // bell started, time reached
            {
                block_output[26+x][1] = 0;      // switch bell off
                IBS_flag[x] = 0;                // reset bell flag
                IBS_flag[x+2] = 0;              // reset timer flag
            }
        }
    }
    return 0;
}

int serial_output()
{
    if( out_flag == 0 ) return 0;  // no need to process if flag not set
    if( out_flag == 2 ) goto skipSO1;  // test mode, do not populate
    // out_flag = 3 is for forced output every 2 seconds (do not print)

    // populate output table
    // clear data before starting
    for(x=0; x<129; x++) output_data[x] = 0;
    // block
    for(x=0; x<28; x++) output_data[block_output[x][3]] = block_output[x][0];  // copy status
    // indicators
    for(x=0; x<33; x++)
    {
        w = indicator_status[x][15];  // address in output_data
        // normal indicator or 17R
        if( indicator_status[x][2] < 4 || indicator_status[x][2] == 7 ) output_data[w] = indicator_status[x][0];  // copy status
        // colour-light
        else if( indicator_status[x][2] == 4 || indicator_status[x][2] == 5 )
        {
            if( indicator_status[x][16] == 1 ) output_data[w] = 1;  // signal is 'On'
            else if( indicator_status[x][16] == 2 ) output_data[w + 1] = 1;  // signal is 'Off'
        }
        // points N-R
        else if( indicator_status[x][2] == 6 )
        {
            output_data[w] = indicator_status[x][9];  // copy status of N
            output_data[w + 1] = indicator_status[x][11];  // copy status of R
        }
    }
    // signal 16 in garden
    //   R-Y  0  R0 Y0 G0
    //        1   1  0  0
    //        2  check status of 16G
    //   G    0   0  0  0
    //        1   0  1  0
    //        2   0  0  1
    for(x=81; x<84; x++) output_data[x] = 0;  // clear R, Y and G to start
    if( indicator_status[13][16] == 1 ) output_data[81] = 1;  // 16 is Red
    else if( indicator_status[13][16] == 2)  // 16 is Off
    {
        //if( indicator_status[39][16] == 1 ) output_data[31] = 1;  // 16 is Yellow
        //else if( indicator_status[39][16] == 2 ) output_data[32] = 1;  // 16 is Green
        if( indicator_status[10][0] == 0 ) output_data[82] = 1;  // 10 is not off; 16 Yellow
        else output_data[83] = 1;                                // 10 is off, 16 Green
    }

    // track circuits
    for(x=1; x<24; x++)
    {
        output_data[TC_status[x][6]] = TC_status[x][0];  // copy status
        if( ( x == 11 || x == 21 ) && TC_status[x][0] == 1 ) x = x + 1;  // split TC already 'On', so skip 2nd part
    }
    // diagram power lights
    output_data[26] = 1; output_data[27] = 1;
skipSO1:
    // print output data
    if( out_flag < 3 )
    {
      printf("SerialOUT ");
      for(x=0; x<15; x++)
      {
        for(y=1; y<9; y++) printf("%d",output_data[x*8+y]);
        if( x == 6 ) printf("\r\n ");
        else printf(" ");
      }
      printf("\r\n");
    }
    // write GPIO output data
    digitalWrite( ENABLE, 1 );
    // send data to shift registers in reverse order
    for( x=120; x>0; x-- )
    {
        digitalWrite( DATAout, output_data[x] );  // write data bit
        delay1();
        digitalWrite( CLOCKout, 1 );              // clock on
        delay1();
        digitalWrite( CLOCKout, 0 );              // clock off
    }
    delay1();delay1();
    digitalWrite( STROBE, 1 );                    // strobe on
    delay1();delay1();
    digitalWrite( STROBE, 0 );                    // strobe off

    out_flag = 0;  // reset flag
    return 0;
}

int calc_locks()
{
    // skip if recalculate flag not set
    if( lock_flag == 0 ) return 0;
    // calculate lever locks
    x = 0;
    while( x < 74 )
    {
        b = 1;  // start as lever lock flag as free
        if( lever_status[x][2] == -1 ) goto skipv1;
        // columns 2-6: TC clear
        y = 2;
        while( y < 7 )
        {
            z = lever_status[x][y];
            if( z == 0 ) goto skipv2;  // finished if zero
            if( TC_status[z][0] == 1 )
            {
                b = 0;    // lock if TC occupied
                goto skipv1;
            }
            y = y + 1;
        }
skipv2:
        // column 7: TC occupied
        z = lever_status[x][7];
        if( z > 0 ) if( TC_status[z][0] == 0 )
            {
                b = 0;    // lock if TC clear
                goto skipv1;
            }
        // columns 8-9: levers normal
        y = 8;
        while( y < 10 )
        {
            z = lever_status[x][y];
            if( z == 0 ) goto skipv3;  // finished if zero
            if( lever_status[z][0] == 0 )
            {
                b = 0;    // lock if lever not normal
                goto skipv1;
            }
            y = y + 1;
        }
skipv3:
        // column 10: needs LC, else perhaps TC occupied
        z = lever_status[x][10];
        if( z > 0 )
        {
            // starters
            if( z == 1 ) { if( block_output[2][0] == 0 || block_locks[1][11] == 1 ) b = 0; }  // NS locked - no LC or started pulled
            else if( z == 2 ) { if( block_output[7][0] == 0 || block_locks[2][11] == 1 ) b = 0; }  // HS locked - no LC or started pulled
            else if( z == 3 ) { if( block_output[15][0] == 0 || block_locks[3][11] == 1 ) b = 0; }  // NF locked - no LC or started pulled
            else if( z == 4 ) { if( block_output[20][0] == 0 || block_locks[4][11] == 1 ) b = 0; }  // HF locked - no LC or started pulled
            // up homes
            else if( z > 100 )
            {
                if( z == 101 ) w = 2;  // NS LC non-pegger
                else if( z == 103 ) w = 15;  // NF
                if( block_output[w][0] == 0 ) b = 0;  // no LC - locked
                if( TC_status[lever_status[x][14]][0] == 1 ) b = 1;  // berth TC occupied - free
            }
        }
        // columns 11-14: needs TCs clear, or berth TC occupied
        else
        {
            if( lever_status[x][11] == 0 ) goto skipv1;
            z = lever_status[x][11];
            if( TC_status[z][0] == 1 ) b = 0;  // TC occupied - locked
            z = lever_status[x][12];
            if( TC_status[z][0] == 1 ) b = 0;  // TC occupied - locked
            z = lever_status[x][13];
            if( TC_status[z][0] == 1 ) b = 0;  // TC occupied - locked
            z = lever_status[x][14];
            if( TC_status[z][0] == 1 ) b = 1;  // berth TC occupied - free
        }
skipv1:
        lever_status[x][16] = b;  // set lever lock flag
        x = x + 1;
    }
    lock_flag = 0;  // reset recalculation flag
    return 0;
}

int calc_block_locks()
{
    // skip if recalculate flag not set
    if( block_flag == 0 ) return 0;
    // for each line, if lock is set (or Demo mode, only for TOL release)
    for(x=1; x<5; x++) if( block_locks[x][0] == 1 || demo == 1 )
    {
        w = 0;
        // Demo mode, only for TOL release
        if( demo == 1 && block_locks[x][4] == 1 )  // Commutator is at TOL
        {
            if( ( x % 2 ) == 0 ) { w = 4; y = 6 * x - 5; }  // y: DS=7, DF=19
            goto skipr1;
        }
        // Welwyn winder flags
        for(z=6; z<10; z++) w = w + block_locks[x][z];  // w is sum of block lock flags
        // Welwyn winder not in use (w = 0)
        // calculate block locks
        if( w == 0 )
        {
            if( x == 1 ) y = 1;  // first TC is TC1 up slow
            else if ( x == 2 ) y = 7;  // ... TC7 down slow
            else if ( x == 3 ) y = 14;  // ... TC14 up fast
            else if ( x == 4 ) y = 19;  // ... TC19 down fast
            z = 1;
            while( z < 4 )
            {
                if( block_locks[x][z] == 0 )
                {
                    if( z == 1 ) { if( TC_status[y][0] == 1 ) block_locks[x][z] = 1; z = 4; }  // TC1 occupied
                    else if( z == 2 ) { if( TC_status[y+1][0] == 1 ) block_locks[x][z] = 1; z = 4; }  // TC2 occupied
                    else if( z == 3 ) { if( TC_status[y][0] == 0 ) block_locks[x][z] = 1; z = 4; }  // TC1 clear
                }
                else z = z + 1;
            }
        }
        // clear locks
        if( w == 0 )  // not Welwyn, so check TC locks
        {
            for(z=1; z<5; z++) w = w + block_locks[x][z];  // w is sum of block lock flags
            if( w == 1 && mode == 15 ) w = 4;  // Passive only needs 'Comm TOL'
        }
    skipr1:
        if( w == 4 && TC_status[y][0] == 0 )  // all conditions met, and berth TC clear (in case Welwyn operated with train at signal)
        {
            for(z=0; z<10; z++) block_locks[x][z] = 0;  // clear locks
            if( (x % 2) == 0 )  // down lines - Rotary
            {
                if( x == 2 ) block_output[21][1] = 1;  // Down slow TOL release
                else block_output[10][1] = 1;  // Down fast TOL release
                bell_flags[x][4] = clock0();  // reference time to switch release off
            }
            printf("%03.1f Line %d block released\r\n",(fx=clock0())/1000,x);
        }
    }
    block_flag = 0;  // reset calculation flag
    return 0;
}

int create_train()
{
    if( mode == 12 )  // timetable mode
    {
        b = 0;  // count of items processed
        x = 1; while( x <= trainqueue2[0][0] )
        {
            // start time reached and not already processed (line 1-4)
            if( clock0() > ( tt_time + 60000 * trainqueue2[x][2] / 10 ) && trainqueue2[x][0] < 5 )
            {
                // check whether line is free
                if( trainqueue2[0][trainqueue2[x][0]] == 0 )
                {
                    trainqueue2[0][trainqueue2[x][0]] = 1;  //set line to not free
                    bell_flags[trainqueue2[x][0]][7] = tt_time + 60000 * trainqueue2[x][2] / 10;  // save original start time (times have factor of 10)
                    trainqueue2[x][0] = trainqueue2[x][0] + 100;  // indicates record processed
                    goto jumpr1;
                }
            }
            // increase count if already processed
            else if( trainqueue2[x][0] > 100 ) b = b + 1;
            x = x + 1;
        }
        // all trains processed and lever 21 activated
        // ...and 3 seconds beyond start time to allow for forced lever read otherwise causing restart
        if( b == trainqueue2[0][0] && ( lever_status[34][0] == 0 || input_data[78] == 1 ) && clock0() > ( tt_time + 3000 ) )
        {
            // reset processed flag
            for(x=1; x<=trainqueue2[0][0]; x++) trainqueue2[x][0] = trainqueue2[x][0] - 100;
            tt_time = clock0();  // reset start time offset
            b = 0;
            printf("Timetable restarted\r\n");
        }
        return 0;
    }
    else // all other modes
    {
        x = 1; while( x < 5 )  // routes 1 to 4
        {
            if (trainqueue[0][x] > 100)  // ie. next train required
            {
                trainqueue[0][x] = trainqueue[0][x] - 100;  // reset 'next train' flag
                b = trainqueue[trainqueue[0][x]][x];  // save train bell code
                if (trainqueue[0][x] < 6) trainqueue[0][x] = trainqueue[0][x] + 1;  // increase queue counter, or back to 1
                else trainqueue[0][x] = 1;
                goto jumpr1;
            }
            x = x + 1;
        }
        return 0;
    }
jumpr1:
    // find next free record in 'trains'
    nextfree = 0; while(trains[nextfree][0][0] > 0)
    {
        nextfree = nextfree + 1;
        if( demo == 1 ) return 0;  // create Demos one at a time, ie. only if nexttrain = 0
        if( nextfree > 18 ) return 0;
    }
    // Demo modes
    if( demo == 1 )
    {
        trains[nextfree][0][0] = 31;  // always 31 code
        trains[nextfree][1][0] = x;  // train route
        trains[nextfree][4][0] = clock0() + 8000;  // Demos start in 8 seconds
    }
    // Timetable mode
    else if( mode == 12 )
    {
        trains[nextfree][0][0] = trainqueue2[x][1];  // train bell code
        trains[nextfree][1][0] = trainqueue2[x][0] - 100;  // train route
        trains[nextfree][4][0] = tt_time + 60000 * trainqueue2[x][2] / 10;  // start time
        trains[nextfree][5][0] = tt_time + 60000 * trainqueue2[x][3] / 10;  // TES time
        if( trainqueue2[x][4] > 0 ) trains[nextfree][8][0] = tt_time + 60000 * trainqueue2[x][4] / 10;  // departure time for stopping train
    }
    // All other modes
    else
    {
        trains[nextfree][0][0] = b;  // train bell code
        trains[nextfree][1][0] = x;  // train route
        trains[nextfree][4][0] = clock0() + 300 + random2(101) * 4200;  // start time between 0 and 7 minutes from now
        bell_flags[x][7] = trains[nextfree][4][0];  // save original start time
    }
    trains[nextfree][2][0] = 0;  // train status (used in 'movetrain')
    trains[nextfree][3][0] = trains[nextfree][1][0];  // train line

    // convert bell code to table ref.
    y = 0;
    while( bellcode[y][1] != trains[nextfree][0][0] ) y = y + 1;
    trains[nextfree][0][0] = y;

    // copy TCs, Yards and Speed
    dmu = 0;
    if( trains[nextfree][0][0] == 4 || trains[nextfree][8][0] > 0 ) dmu = 2;  // set flag for DMU or stopping train (mode 12)
    x = trains[nextfree][1][0];  // x = route (necessary for mode 12)
    for(y=1; y<3; y++) for(z=0; z<9; z++) trains[nextfree][z][y] = route[y-1][z][x];  // TCs and yards
    for(z=0; z<9; z++) trains[nextfree][z][3] = route[2+dmu][z][x];  // speeds

    // set train max speed
    speed = bellcode[trains[nextfree][0][0]][3];
    for(y=0; y<9; y++) if(speed < trains[nextfree][y][3]) trains[nextfree][y][3] = speed;

    printf("\r\ntrain created for route %d",x);
    if( dmu == 2) printf(" (DMU speeds)");
    printf("\r\n");
    for(z=0; z<9; z++)
    {
        for(y=0; y<4; y++) printf(" %d",trains[nextfree][z][y]);
        printf("\r\n");
    }

    return 0;
}

int move_train()
{
    nexttrain = 0;
    while(nexttrain < 19)  // up to 19 trains to process
    {
        if (trains[nexttrain][0][0] > 0)
        {
            line = trains[nexttrain][3][0];  // set line, and hence which bell to ring.
            // send train bell code if start time reached and bell is idle
            if ( trains[nexttrain][2][0] == 0 && clock0() > trains[nexttrain][4][0] && bell_status[line][3] == 0 )
            {
                // check for Line Blocked
                b = 0;
                if( line == 1 && block_output[4][0] == 0 && block_output[5][0] == 0 ) b = 1;
                else if( line == 2 && block_output[9][0] == 0 && block_output[22][0] == 0 ) b = 1;
                else if( line == 3 && block_output[17][0] == 0 && block_output[18][0] == 0 ) b = 1;
                else if( line == 4 && block_output[11][0] == 0 && block_output[12][0] == 0 ) b = 1;
                // OK to send bell code
                if( b == 1 )
                {
                    bell_status[line][3] = 1;  // set bell to 'sending'
                    bell_status[line][4] = nexttrain;  // set train, for data back from bell_status
                    bell_status[line][7] = trains[nexttrain][0][0];  // set bell code
                    printf ("%03.1f code %d %d-%d %03.1fs\r\n",(fx=clock0())/1000,bellcode[trains[nexttrain][0][0]][1],trains[nexttrain][1][0],trains[nexttrain][2][0],(fy=trains[nexttrain][4][0])/1000);
                }
            }
            // train bell code been acknowledged (set to 100 in 'process_bell')
            else if (trains[nexttrain][2][0] == 100)
            {
                // has Line Clear been given
                if( line == 1 ) b = 5;  // ref for LC in block_status
                else if( line == 2 ) b = 22;
                else if( line == 3 ) b = 18;
                else if( line == 4 ) b = 12;
                if( block_output[b][0] == 1 )
                {
                    trains[nexttrain][2][0] = 200;  // LC given (set to 200)
                    printf ("%03.1f code %d %d-%d %03.1fs\r\n",(fx=clock0())/1000,bellcode[trains[nexttrain][0][0]][1],trains[nexttrain][1][0],trains[nexttrain][2][0],(fy=trains[nexttrain][5][0])/1000);
                }
                else if( clock0() > trains[nexttrain][4][0] + 10000 )  // no LC after 10 seconds
                {
                    // send train code again with no call attention
                    if( bell_status[line][3] == 0 )
                    {
                        trains[nexttrain][4][0] = clock0();  // to allow time to check for LC again
                        bell_status[line][3] = 1;  // set bell to 'send'
                        bell_status[line][4] = nexttrain;  // set train, for data back from bell_status
                        bell_status[line][7] = trains[nexttrain][0][0];  // set bell code
                        code = bell_status[line][7];
                        bell_status[line][6] = 7;  // set to 'sending' train
                        bell_status[line][14] = clock0() + 500;  // set send time 0.5 second
                        bell_status[line][16] = 1;  // set count to 1
                        bell_status[line][17] = 0;  // start as off
                        for(y=18; y<28; y++) bell_status[line][y] = bellcode[code][y-14]; // copy bell intervals
                        printf("%d-",line);
                        for(z=0; z<28; z++) printf("%d ",bell_status[line][z]);
                        printf("\r\n");
                    }
                }
            }
            // train has been refused
            else if (trains[nexttrain][2][0] == 101)
            {
                if( clock0() > trains[nexttrain][4][0] )  // repeat time reached
                {
                    // send train code again with no call attention
                    if( bell_status[line][3] == 0 )
                    {
                        trains[nexttrain][4][0] = clock0();  // to allow time to check for LC again
                        bell_status[line][3] = 1;  // set bell to 'send'
                        bell_status[line][4] = nexttrain;  // set train, for data back from bell_status
                        bell_status[line][7] = trains[nexttrain][0][0];  // set bell code
                        code = bell_status[line][7];
                        bell_status[line][6] = 3;  // set to 'sending' train
                        bell_status[line][14] = clock0() + 500;  // set send time 0.5 second
                        bell_status[line][16] = 1;  // set count to 1
                        bell_status[line][17] = 0;  // start as off
                        for(y=18; y<28; y++) bell_status[line][y] = bellcode[code][y-14]; // copy bell intervals
                        printf("%d-",line);
                        for(z=0; z<28; z++) printf("%d ",bell_status[line][z]);
                        printf("\r\n");
                        bell_flags[line][1] = 0;  // clear refused bell flags
                        bell_flags[line][2] = 0;
                        trains[nexttrain][2][0] = 0;  // reset status
                        printf ("%03.1f code %d %d-%d (was 101)\r\n",(fx=clock0())/1000,bellcode[trains[nexttrain][0][0]][1],trains[nexttrain][1][0],trains[nexttrain][2][0]);
                    }
                }
            }
            // LC has been given; set time for Train Entering Section
            else if (trains[nexttrain][2][0] == 200)
            {
                // TES for timetable mode from table , or 2 seconds from now if already reached
                if( mode == 12 ) { if( clock0() > trains[nexttrain][5][0] ) trains[nexttrain][5][0] = clock0() + 2000; }
                // TES Up lines: immediate (2 seconds) or demo 25 seconds
                else if (trains[nexttrain][1][0] == 1 || trains[nexttrain][1][0] == 3)
                    trains[nexttrain][5][0] = clock0() + 2000 + demo * 23000;
                // TES random between 1 and 3 minutes for Down lines
                else trains[nexttrain][5][0] = clock0() + (10 + random2(21)) * 6000;
                trains[nexttrain][2][0] = 400;
                printf ("%03.1f code %d %d-%d %03.1fs\r\n",(fx=clock0())/1000,bellcode[trains[nexttrain][0][0]][1],trains[nexttrain][1][0],trains[nexttrain][2][0],(fy=trains[nexttrain][5][0])/1000);
            }
            // send Train Entering Section if time reached and bell idle, and calculate time for first TC
            else if (trains[nexttrain][2][0] == 400)
            {
                if( clock0() > trains[nexttrain][5][0] && bell_status[line][3] == 0 )
                {
                    // setup TES bell code to send
                    bell_status[line][3] = 1;  // set to send
                    bell_status[line][7] = 1;  // bell code 2
                    // for up lines, reduce yardage if TES later than 10 seconds after original offer time
                    if( trains[nexttrain][1][0] % 2 == 1 && clock0() > (bell_flags[trains[nexttrain][1][0]][7] + 10000) )
                    {
                        // difference is time x speed
                        trains[nexttrain][0][2] = trains[nexttrain][0][2] - ( ( clock0() - bell_flags[trains[nexttrain][1][0]][7] - 10000 ) * trains[nexttrain][0][3] / 1000 ) *1760 / 3600;
                        // maximum reduction to 5500 yards, and reduce speed by a third
                        if( trains[nexttrain][0][2] < 5500 )
                        {
                            trains[nexttrain][0][2] = 5500;
                            trains[nexttrain][0][3] = ( trains[nexttrain][0][3] * 2 ) / 3;
                        }
                    }
                    // time is yards / speed x 2045 in milliseconds
                    trains[nexttrain][1][4] = clock0() + (trains[nexttrain][0][2] * 2045) / trains[nexttrain][0][3];
                    if( demo == 1 ) trains[nexttrain][1][4] = clock0() + 65000;  // demo: 65 seconds to first TC (was 55 seconds)
                    trains[nexttrain][2][0] = 501;
                    printf ("%03.1f code %d %d-%d %03.1fs %dyds %dmph\r\n",(fx=clock0())/1000,bellcode[trains[nexttrain][0][0]][1],trains[nexttrain][1][0],trains[nexttrain][2][0],(fy=trains[nexttrain][1][4])/1000,trains[nexttrain][0][2],trains[nexttrain][0][3]);
                }
            }
            // has next TC been reached
            else if (trains[nexttrain][2][0] > 500 && trains[nexttrain][2][0] < 550)
            {
                x = trains[nexttrain][2][0] % 100;  // modulus gives last digit, ie. TC reference
                if( clock0() > trains[nexttrain][x][4] )
                {
                    // Annunciator - check if home signal ON
                    if( x == 1 )  // berth TC
                    {
                        signal = route[3][x][trains[nexttrain][1][0]];
                        if( lever_status[signal][0] == 0 )  // signal not OFF
                        {
                            if( annun_status[0][2] == 1 ) annun_status[0][3] = clock0() + 2000;  // annunciator already on - extend time (another 12 seconds)
                            else annun_status[0][1] = 1;  // turn annunciator on
                            printf("%03.1f Annunciator ON\r\n",(fx=clock0())/1000);
                        }
                    }
                    // check that signal still clear (if applicable)
                    // ... previous is controlled; signal at Danger
                    signal = route[3][x-1][trains[nexttrain][1][0]];
                    if( signal > 0 && x > 1 ) if( lever_status[signal][0] == 0 )
                        {
                            trains[nexttrain][2][0] = trains[nexttrain][2][0] + 50 - 1;  // step back status, and set signal flag
                            // set stop flag if signal 6 38 37 9 11 42 41 (re TC off calculation)
                            if( signal == 11 || signal == 61 || signal == 59 || signal == 17 || signal == 21 || signal == 69 || signal == 67 ) trains[nexttrain][7][0] = 1;
                            printf ("%03.1f code %d %d-%d STOPPED lever ref %d\r\n",(fx=clock0())/1000,bellcode[trains[nexttrain][0][0]][1],trains[nexttrain][1][0],trains[nexttrain][2][0],signal);
                            reduce_speed();
                            goto jumpw1;  // goto end; don't enter next TC
                        }
                    z = bellcode[trains[nexttrain][0][0]][2];  // length of train (for TC off calc)
                    // skip if passed last TC
                    if (trains[nexttrain][x][1] == 99)
                    {
                        // calculate TC off time
                        calc_TCoff();
                        // calculate time to Train Out Of Section, and set status
                        trains[nexttrain][6][0] = clock0() + (trains[nexttrain][x][2] * 2045) / trains[nexttrain][x][3];
                        if( demo == 1 ) trains[nexttrain][6][0] = clock0() + 40000;  // demo: TOS in 40 seconds
                        trains[nexttrain][2][0] = 600;
                        printf ("%03.1f code %d %d-%d %03.1fs TOS\r\n",(fx=clock0())/1000,bellcode[trains[nexttrain][0][0]][1],trains[nexttrain][1][0],trains[nexttrain][2][0],(fy=trains[nexttrain][6][0])/1000);
                    }
                    // switch TC on (reference is in trains[nexttrain][x][1])
                    else
                    {
                        TC_status[trains[nexttrain][x][1]][1] = 1;
                        if( trains[nexttrain][2][0] == 501 && demo == 0 )
                        {
                            // ready to queue next train
                            if( mode == 12 ) trainqueue2[0][trains[nexttrain][1][0]] = 0;
                            else trainqueue[0][trains[nexttrain][1][0]] = trainqueue[0][trains[nexttrain][1][0]] + 100;  // queue next train when first TC occupied (not for demo)
                        }
                        trains[nexttrain][2][0] = trains[nexttrain][2][0] + 1;
                        // calculate time to next TC
                        trains[nexttrain][x + 1][4] = clock0() + (trains[nexttrain][x][2] * 2045) / trains[nexttrain][x][3];
                        printf ("%03.1f code %d %d-%d %03.1fs TC%d on (%dyds %dmph)\r\n",(fx=clock0())/1000,bellcode[trains[nexttrain][0][0]][1],trains[nexttrain][1][0],trains[nexttrain][2][0],(fy=trains[nexttrain][x+1][4])/1000,trains[nexttrain][x+1][1],trains[nexttrain][x][2],trains[nexttrain][x][3]);
                        // calculate TC off time for previous TC
                        calc_TCoff();
                        // check for DMU or stop flag set, and stop in station
                        b = 0;
                        if( trains[nexttrain][0][0] == 4 || trains[nexttrain][8][0] > 0 )
                        {
                            if( trains[nexttrain][3][0] == 1 && trains[nexttrain][x][1] == 3 ) b = 1;  // line 1, TC3
                            else if( trains[nexttrain][3][0] == 2 && trains[nexttrain][x][1] == 10 ) b = 1;  // line 2, TC10
                            else if( trains[nexttrain][3][0] == 3 && trains[nexttrain][x][1] == 15 ) b = 1;  // line 3, TC15
                            else if( trains[nexttrain][3][0] == 4 && trains[nexttrain][x][1] == 20 ) b = 1;  // line 4, TC20
                            if( b == 1 )
                            {
                                trains[nexttrain][2][0] = trains[nexttrain][2][0] + 70 - 1;  // step back status, and set station stop
                                if( mode == 12 )
                                {
                                    if( (clock0() + 80000) > trains[nexttrain][8][0] ) trains[nexttrain][5][0] = clock0() + 90000;  // minimum 60 second stop (plus 20 to stop and 10 to start)
                                    else trains[nexttrain][5][0] = trains[nexttrain][8][0] + 10000;  // set departure time for timetable mode from table
                                }
                                else trains[nexttrain][5][0] = clock0() + 60000 - demo * 30000;  // set departure time 60 seconds (demo 40 seconds)
                                trains[nexttrain][7][0] = 1;  // set stop flag (forces method 1 for TC off calculation)
                                printf ("%03.1f code %d %d-%d Station stop - depart %03.1f\r\n",(fx=clock0())/1000,bellcode[trains[nexttrain][0][0]][1],trains[nexttrain][1][0],trains[nexttrain][2][0],(fy=trains[nexttrain][5][0])/1000);
                            }
                        }
                        // flag signal at Danger
                        // ... current is controlled; signal at Danger
                        if( b == 0 )  // ie. not a DMU dealt with above
                        {
                            signal = route[3][x][trains[nexttrain][1][0]];
                            if( signal > 0 ) if( lever_status[signal][0] == 0 )
                                {
                                    trains[nexttrain][2][0] = trains[nexttrain][2][0] + 50 - 1;  // step back status, and set signal flag
                                    // set stop flag if signal 6 38 37 9 11 42 41 (re TC off calculation)
                                    if( signal == 11 || signal == 61 || signal == 59 || signal == 17 || signal == 21 || signal == 69 || signal == 67 ) trains[nexttrain][7][0] = 1;
                                    printf ("%03.1f code %d %d-%d\r\n",(fx=clock0())/1000,bellcode[trains[nexttrain][0][0]][1],trains[nexttrain][1][0],trains[nexttrain][2][0]);
                                    reduce_speed();
                                }
                        }
                    }
                }
            }
            // signal is at Danger
            else if (trains[nexttrain][2][0] > 550 && trains[nexttrain][2][0] < 570)
            {
                // signal now Clear
                x = trains[nexttrain][2][0] % 50;  // modulus gives last digit, ie. TC reference
                signal = route[3][x][trains[nexttrain][1][0]];
                if( lever_status[signal][0] == 1 )
                {
                    trains[nexttrain][2][0] = trains[nexttrain][2][0] - 50 + 1;  // restore status to normal
                    if( x == 1 ) annun_status[0][1] = 0;  // turn off annunciator (berth TC)
                    if( clock0() > trains[nexttrain][x+1][4] - 10000 ) trains[nexttrain][x+1][4] = clock0() + 10000;  // train at stand or within 10 sec; 10 seconds to start
                    else if( clock0() > trains[nexttrain][x+1][4] - 15000 ) trains[nexttrain][x+1][4] = trains[nexttrain][x+1][4] + 5000;  // within 15 sec; add 5 seconds to next
                    printf ("%03.1f code %d %d-%d (was %d)\r\n",(fx=clock0())/1000,bellcode[trains[nexttrain][0][0]][1],trains[nexttrain][1][0],trains[nexttrain][2][0],trains[nexttrain][2][0]+49);
                }
                // check for new route
                else
                {
                    newroute = 0;
                    change_route();
                    if( newroute > 0 )  // route has changed
                    {
                        // copy TCs, Yards and Speed
                        for(y=1; y<4; y++) for(z=0; z<9; z++) trains[nexttrain][z][y] = route[y-1][z][newroute];
                        // set train max speed
                        speed = bellcode[trains[nexttrain][0][0]][3];
                        for(y=0; y<9; y++) if(speed < trains[nexttrain][y][3]) trains[nexttrain][y][3] = speed;
                        printf("\r\nRoute changed from %d to %d\r\n",trains[nexttrain][1][0],newroute);
                        trains[nexttrain][1][0] = newroute;  // set route in 'trains'
                        trains[nexttrain][3][0] = route[3][0][newroute];  // set line in 'trains'
                        // set status
                        if( newroute == 3 || newroute == 7 ) trains[nexttrain][2][0] = 503;
                        else trains[nexttrain][2][0] = 502;
                        x = trains[nexttrain][2][0] % 100;
                        // set start time
                        if( newroute < 8 )  // new routes 2 to 7
                        {
                            if( clock0() > trains[nexttrain][x][4] - 10000 ) trains[nexttrain][x][4] = clock0() + 10000;  // train at stand or within 10 sec; 10 seconds to start
                            else if( clock0() > trains[nexttrain][x][4] - 15000 ) trains[nexttrain][x][4] = trains[nexttrain][x][4] + 5000;  // within 15 sec; add 5 seconds to next
                        }
                        else  // new routes 8 to 9
                        {
                            // Timetable mode
                            if( mode == 12 )
                            {
                                trains[nexttrain][x][4] = clock0() + 20000;  // default if no train found: start after 20 seconds
                                // find next train not already processed for newroute
                                y = 1; while( y <= trainqueue2[0][0] )
                                {
                                    if( trainqueue2[y][0] == newroute )
                                    {
                                        trains[nexttrain][x][4] = tt_time + ( 60000 * trainqueue2[y][4] ) / 10 + 10000;  // start time from timetable + 10 seconds
                                        if( clock0() > trains[nexttrain][x][4] ) trains[nexttrain][x][4] = clock0() + 20000;  // if start time alrady reached, start after 20 seconds
                                        trainqueue2[y][0] = trainqueue2[y][0] + 100;  // mark as processed
                                        y = trainqueue2[0][0];  // skip to end of 'while' loop
                                    }
                                    y = y + 1;
                                }
                            }
                            // other modes (ie. Full operation)
                            else
                            {
                                if( clock0() > trains[nexttrain][newroute-3][4] + 300000 ) trains[nexttrain][x][4] = clock0() + 20000;  // train standing more than 5 minutes; 20 seconds to start
                                else trains[nexttrain][x][4] = trains[nexttrain][newroute-3][4] + 300000;  // train stands for minimum 5 minutes
                            }
                            if( newroute == 9 ) trains[nexttrain][7][0] = 1;  // force method 1 for TC3 when route 9 train moves
                        }
                        // turn off annunciator (signal 25 or 36)
                        if( newroute == 5 || newroute == 6 ) annun_status[0][1] = 0;
                    }
                }
            }
            // train stopped in station
            else if (trains[nexttrain][2][0] > 570 && trains[nexttrain][2][0] < 600)
            {
                if( clock0() > trains[nexttrain][5][0] )  // time reached for re-start
                {
                    trains[nexttrain][2][0] = trains[nexttrain][2][0] - 70 + 1;  // restore status to normal
                    printf ("%03.1f code %d %d-%d (was %d)\r\n",(fx=clock0())/1000,bellcode[trains[nexttrain][0][0]][1],trains[nexttrain][1][0],trains[nexttrain][2][0],trains[nexttrain][2][0]+69);
                }
            }
            // next step is TOS
            else if( trains[nexttrain][2][0] == 600 )
            {
                if( clock0() > trains[nexttrain][6][0] )  // time for TOS reached?
                {
                    // check that block is at TOL
                    // note that line 1 in this case means in advance: NS non-pegger, etc.
                    if( line == 1 ) b = 1;  // NS - ref for TOL in block_status
                    else if( line == 2 ) b = 6;  // HS
                    else if( line == 3 ) b = 14;  // NF
                    else if( line == 4 ) b = 19;  // HF
                    if( block_output[b][0] == 0 )  // TOL not set
                    {
                        block_output[b][1] = 1;  // set TOL
                        block_output[b+1][1] = 0;  // clear LC
                        trains[nexttrain][6][0] = trains[nexttrain][6][0] + 30000;  // add 30 seconds to TOS time
                    }
                    else
                    {
                        // switch to opposite line bell
                        if( line % 2 == 1 ) line = line + 1;
                        else line = line - 1;
                        // setup TOS bell code to send
                        if( bell_status[line][3] == 0 )  // bell is idle
                        {
                            bell_status[line][3] = 1;  // set to send
                            bell_status[line][4] = nexttrain;  // set train, for data back from bell_status
                            bell_status[line][7] = 2;  // bell code 2-1
                            trains[nexttrain][2][0] = 700;
                        }
                    }
                }
            }
            // TOS has been sent and acknowledged
            else if( trains[nexttrain][2][0] == 800 )
            {
                // note use of 'opposite' line
                if( line == 1 ) b = 1;  // ref for TOL in block_status
                else if( line == 2 ) b = 6;
                else if( line == 3 ) b = 14;
                else if( line == 4 ) b = 19;
                block_output[b][1] = 0;  // clear TOL
                for(y=0; y<9; y++) for(z=0; z<6; z++) trains[nexttrain][y][z] = 0;  // clear train
            }
            // switch TCs off
            y = 1;
            while( trains[nexttrain][y][1] < 99  && y < 9 )
            {
                if( trains[nexttrain][y][5] > 0 )
                {
                    if( clock0() > trains[nexttrain][y][5] )
                    {
                        TC_status[trains[nexttrain][y][1]][1] = 0;  // time reached, switch TC off
                        // set TC off time to zero, so that another train using same TC is not corrupted
                        if( TC_status[trains[nexttrain][y+1][1]][5] != 2 ) trains[nexttrain][y][5] = 0;  // time needed for method 2
                        trains[nexttrain][y-1][5] = 0;  // in case previous was method 2
                    }
                }
                y = y + 1;
            }
        }
jumpw1:
        nexttrain = nexttrain + 1;
    }
    return 0;
}

int calc_TCoff()
{
    // ... x is position of next TC
    // ... z is length of train
    // ... b is TC to be switched off
    // ... method is in TC_status table
    b = trains[nexttrain][x-1][1];
    // method 3 - time to travel train length, at current TC speed
    if( TC_status[b][5] == 3 )
    {
        trains[nexttrain][x-1][5] = clock0() + (z * 2045) / trains[nexttrain][x-1][3];  // TC off time
        trains[nexttrain][7][0] = 0;  // clear stop flag (needed for route 8, TC10)
        printf ("%03.1f code %d %d-%d time %03.1f TC%d off(3)\r\n",(fx=clock0())/1000,bellcode[trains[nexttrain][0][0]][1],trains[nexttrain][1][0],trains[nexttrain][2][0],(fy=trains[nexttrain][x-1][5])/1000,trains[nexttrain][x-1][1]);
    }
    // method 1 - time to travel train length
    else if( TC_status[b][5] == 1 || trains[nexttrain][7][0] == 1 )  // also forced by signal/station stop flag
    {
        trains[nexttrain][x-1][5] = clock0() + (z * 2045) / trains[nexttrain][x][3];  // TC off time
        trains[nexttrain][7][0] = 0;  // clear stop flag
        printf ("%03.1f code %d %d-%d time %03.1f TC%d off(1)\r\n",(fx=clock0())/1000,bellcode[trains[nexttrain][0][0]][1],trains[nexttrain][1][0],trains[nexttrain][2][0],(fy=trains[nexttrain][x-1][5])/1000,trains[nexttrain][x-1][1]);
    }
    // method 2 - time to travel TC length from last TC off
    else if( TC_status[b][5] == 2 )
    {
        trains[nexttrain][x-1][5] = trains[nexttrain][x-2][5] + (trains[nexttrain][x-1][2] * 2045) / trains[nexttrain][x][3];  // TC off time; time to travel length of TC at current speed
        printf ("%03.1f code %d %d-%d time %03.1f TC%d off(2)\r\n",(fx=clock0())/1000,bellcode[trains[nexttrain][0][0]][1],trains[nexttrain][1][0],trains[nexttrain][2][0],(fy=trains[nexttrain][x-1][5])/1000,trains[nexttrain][x-1][1]);
    }
    // special case - train length 240+ will not fit in TC3, may be stopped at signal 6
    if( trains[nexttrain][1][0] == 1 && z > 240 )  // route 1 and train length is 240+
    {
        if( x == 3 )  // can't calculate TC2 off time yet
        {
            trains[nexttrain][x-1][5] = 0;
            printf ("%03.1f code %d %d-%d TC%d off cancel(special)\r\n",(fx=clock0())/1000,bellcode[trains[nexttrain][0][0]][1],trains[nexttrain][1][0],trains[nexttrain][2][0],trains[nexttrain][x-1][1]);
        }
        else if( x == 4 )  // now calculate TC2 off
        {
            trains[nexttrain][x-2][5] = clock0() + ((z - 240) * 2045) / trains[nexttrain][x][3];  // TC off time, re 60 yards of train
            trains[nexttrain][x-1][5] = trains[nexttrain][x-2][5] + (trains[nexttrain][x-1][2] * 2045) / trains[nexttrain][x][3];  // TC off time; time to travel length of TC at current speed
            printf ("%03.1f code %d %d-%d time %03.1f TC%d off(special)\r\n",(fx=clock0())/1000,bellcode[trains[nexttrain][0][0]][1],trains[nexttrain][1][0],trains[nexttrain][2][0],(fy=trains[nexttrain][x-2][5])/1000,trains[nexttrain][x-2][1]);
            printf ("%03.1f code %d %d-%d time %03.1f TC%d off(special)\r\n",(fx=clock0())/1000,bellcode[trains[nexttrain][0][0]][1],trains[nexttrain][1][0],trains[nexttrain][2][0],(fy=trains[nexttrain][x-1][5])/1000,trains[nexttrain][x-1][1]);
        }
    }
    return 0;
}

int reduce_speed()
{
    b = trains[nexttrain][1][0] * 1000 + trains[nexttrain][2][0];  // reference in table (eg. 1551)
    y = 1;
    while( y < 8 )
    {
        if( b == route[5][0][y] )
        {
            for(z=1; z<9; z++) if( route[5][z][y] < trains[nexttrain][z][3] )
                {
                    // reduced speed is less than current speed, so replace it
                    trains[nexttrain][z][3] = route[5][z][y];
                }
            y = 8;  // end while loop
            printf ("%03.1f code %d %d-%d reduced speed\r\n",(fx=clock0())/1000,bellcode[trains[nexttrain][0][0]][1],trains[nexttrain][1][0],trains[nexttrain][2][0]);
        }
        else y = y + 1;
    }
    return 0;
}

int change_route()
{
    // check if a signal is cleared to a new route
    if( trains[nexttrain][1][0] == 2 )  // currently route 2
    {
        if( x == 1 && lever_status[57][0] == 1 ) newroute = 5;  // TC 1, signal 36
        else if( x == 1 && lever_status[39][0] == 1 ) newroute = 6;  // TC 1, signal 25
        else if( x == 4 && lever_status[27][0] == 1 ) newroute = 8;  // TC 4, signal 16
    }
    else if( trains[nexttrain][1][0] == 3 )  // currently route 3
    {
        if( x == 2 && lever_status[17][0] == 1 ) newroute = 7;  // TC 2, signal 9
        // if( x == 2 && lever_status[17][0] == 1 && demo == 0 ) newroute = 7;  // TC 2, signal 9 (not for demo)
    }
    else if( trains[nexttrain][1][0] == 5 )  // currently route 5
    {
        if( x == 1 && lever_status[25][0] == 1 ) newroute = 6;  // TC 1, signal 25
        else if( x == 1 && lever_status[63][0] == 1 ) newroute = 2;  // TC 1, signal 39
    }
    else if( trains[nexttrain][1][0] == 6 )  // currently route 6
    {
        if( x == 1 && lever_status[57][0] == 1 ) newroute = 5;  // TC 1, signal 36
        else if( x == 1 && lever_status[63][0] == 1 ) newroute = 2;  // TC 1, signal 39
        else if( x == 5 && lever_status[11][0] == 1 ) newroute = 9;  // TC 1, signal 6
    }
    else if( trains[nexttrain][1][0] == 7 )  // currently route 7
    {
        if( x == 2 && lever_status[21][0] == 1 ) newroute = 3;  // TC 2, signal 11
    }
    else if( trains[nexttrain][1][0] == 8 )  // currently route 8
    {
        if( x == 1 && lever_status[61][0] == 1 ) newroute = 2;  // TC 1, signal 38
    }
    // route 6 only allowable for bell code 3-1, 2-2-1 and 2-3
    if( newroute == 6 && trains[nexttrain][0][0] != 4 && trains[nexttrain][0][0] != 8 && trains[nexttrain][0][0] != 14 ) newroute = 0;

    return 0;
}

int annunciator()
{
    // changed status
    x = 1; while( x >= 0 )  // 1 - Lever alarm; 0 - Annunciator
    {
        if( annun_status[x][0] != annun_status[x][1] )
        {
            if( annun_status[x][1] == 1 )  // switching on
            {
                if( !(x == 0 && annun_status[1][2] == 1) )  // only set flag if Lever alarm already on
                  block_output[23][1] = 1;  // turn annunciator on
                annun_status[x][2] = 1;  // flag on
                annun_status[x][3] = clock0();  // record start time
                if( x == 1 ) printf("%03.1f Annunciator ON (lever alarm)\r\n",(fx=clock0())/1000);
            }
            else  // switching off
            {
                block_output[23][1] = 0;  // turn annunciator off
                // if Annunciator is On (and had been overriden by Lever Alarm)
                //   restore to On and reset time to 7 seconds
                if( x == 1 && annun_status[0][2] == 1)
                {
                    block_output[23][1] = 1;  // turn annunciator on
                    annun_status[0][3] = clock0() - 3000;
                }
                else annun_status[x][2] = 0;  // flag off
                if( x == 0 ) printf("%03.1f Annunciator OFF (signal)\r\n",(fx=clock0())/1000);
                else printf("%03.1f Annunciator OFF (lever alarm)\r\n",(fx=clock0())/1000);
            }
            annun_status[x][0] = annun_status[x][1];  // set new status
        }
        x = x - 1;
    }
    // Lever alarm
    //   turns On and Off every 0.5 seconds
    if( annun_status[1][0] == 1 )
    {
        if( annun_status[1][2] == 1 && clock0() > ( annun_status[1][3] + 500 ) )  // On, and time reached
        {
            block_output[23][1] = 0;  // turn annunciator off
            annun_status[1][2] = 0;  // flag off
            annun_status[1][3] = clock0();  // record time for next On
        }
        if( annun_status[1][2] == 0 && clock0() > ( annun_status[1][3] + 500 ) )  // Off, and time reached
        {
            block_output[23][1] = 1;  // turn annunciator on
            annun_status[1][2] = 1;  // flag on
            annun_status[1][3] = clock0();  // record time for next Off
        }
    }
    // Annunciator
    //   On for 10 seconds
    //   (only actioned if Lever alarm is not activated)
    else if( annun_status[0][2] == 1 )  // flag is on (annunciator is on)
    {
        if( clock0() > ( annun_status[0][3] + 10000 ) )  // switch off after 10 seconds
        {
            block_output[23][1] = 0;  // turn annunciator off
            annun_status[0][0] = 0; annun_status[0][1] = 0;  // reset status
            annun_status[0][2] = 0;  // flag off
            printf("%03.1f Annunciator OFF (time)\r\n",(fx=clock0())/1000);
        }
    }
    return 0;
}

int file_read()  // read data from file, construct address tables, and other initialisations
{
    if( read2 == 1 ) goto jumpa;  // file already read
    FILE* file;
    file = fopen("/home/pi/Codeblocks/SIM-C/data262.txt","r");
    // file = fopen("C:\\SIM-C\\DATA-C201.TXT","r");
    if (file == 0)
    {
        printf("file error \r\n");
        return 1;
    }
// read lever_status data from file
    for(x=0; x<74; x++) for(y=2; y<16; y++) fscanf(file, "%d", &lever_status[x][y]);
// read indicator_status data from file
    for(x=0; x<40; x++)
    {
        fscanf(file, "%d", &indicator_status[x][2]);
        for(y=5; y<16; y++) fscanf(file, "%d", &indicator_status[x][y]);
    }
// read block_input data from file
    for(x=0; x<16; x++) fscanf(file, "%d", &block_input[x][2]);
    for(x=0; x<16; x++) fscanf(file, "%d", &block_input[x][4]);
// read block_output data from file
    for(x=0; x<28; x++) fscanf(file, "%d", &block_output[x][3]);
// read bellcode data from file
    for(x=0; x<20; x++) for(y=0; y<14; y++) fscanf(file, "%d", &bellcode[x][y]);
// read TC status data from file
    for(y=2; y<7; y++) for(x=1; x<24; x++) fscanf(file, "%d", &TC_status[x][y]);
// read route data from file (6 blocks of data)
    for(x=0; x<6; x++) for(y=1; y<10; y++) for(z=0; z<9; z++) fscanf(file, "%d", &route[x][z][y] );
// read train queue data from file
    for(x=1; x<5; x++) for(y=0; y<7; y++) fscanf(file, "%d", &trainqueue[y][x] );
// read timetable data from file
    fscanf(file, "%d", &temp );  // read number of timetable records into 'temp'
    for(x=0; x<temp; x++) for(y=0; y<5; y++) fscanf(file, "%d", &timetable[x][y] );

    fclose (file);
    printf("\r\nFile read OK\r\n");

// construct ind_addr table
    for(x=0; x<10; x++)
    {
        ind_addr[x][0] = 21;
        ind_addr[x+10][0] = 21;
        ind_addr[x+20][0] = 22;
        ind_addr[x+30][0] = 22;
        ind_addr[x][1] = 16 * (x + 1);
        ind_addr[x+10][1] = x + 1;
        ind_addr[x+20][1] = 16 * (x + 1);
        ind_addr[x+30][1] = x + 1;
    }
    for(x=0; x<7; x++)
    {
        ind_addr[x+40][0] = 23;
        ind_addr[x+40][1] = 16 * (x + 1);
    }

// construct TC_addr table
    for(x=0; x<10; x++)
    {
        TC_addr[x][0] = 11;
        TC_addr[x+10][0] = 11;
        TC_addr[x][1] = 16 * (x + 1);
        TC_addr[x+10][1] = x + 1;
    }
    TC_addr[20][0] = 23;
    TC_addr[20][1] = 128;

    read2 = 1;  // don't read file and construct tables again when read = 1

jumpa:
    srand((unsigned) time(NULL));  // prime rand()
    if( mode < 20 )
    {
        // zeroise lever and indicator status...
        // so that levers read from scratch and indicators set accordingly...
        // in case change of mode or 2nd time through
        for(x=0; x<74; x++) { for(y=0; y<2; y++) lever_status[x][y] = 0; lever_status[x][16] = 1; lever_status[x][17] = 0; }  // also lever locks free
        for(x=0; x<39; x++) { for(y=0; y<2; y++) indicator_status[x][y] = 0; indicator_status[x][16] = 1; }  // also initialise C-L status to 'On' for serial output
        indicator_status[4][9] = 0; indicator_status[4][11] = 0;  // clear flags for N-R indicators
        indicator_status[16][9] = 0; indicator_status[16][11] = 0;
        indicator_status[17][9] = 0; indicator_status[17][11] = 0;
        for(x=1; x<24; x++) for(y=0; y<2; y++) TC_status[x][y] = 0;
        for(x=0; x<16; x++) for(y=0; y<2; y++) block_input[x][y] = 0;
        for(x=1; x<5; x++) for(y=0; y<28; y++) bell_status[x][y] = 0;
        for(x=0; x<28; x++) for(y=0; y<3; y++) block_output[x][y] = 0;
        for(x=0; x<5; x++) for(y=0; y<12; y++) block_locks[x][y] = 0;
        for(x=0; x<14; x++) for(y=0; y<9; y++) for(z=0; z<6; z++) trains[x][y][z] = 0;
        for(x=0; x<50; x++) for(y=0; y<6; y++) trainqueue2[x][y] = 0;
        for(x=0; x<3; x++) for(y=0; y<4; y++) annun_status[x][y] = 0;

        // copy selected timetable data
        if( mode == 12 )
        {
            w = 0; while( w < 50 )
            {
                if( ( timetable[w][0] / 1000 ) == tt )  // is required tiemtable found
                {
                    y = timetable[w][0] % 1000;  // number of timetable records
                    trainqueue2[0][0] = y;
                    for(x = w + 1; x < (w + 1 + y); x++) for(z = 0; z < 5; z++)
                    {
                        trainqueue2[x - w][z] = timetable[x][z];  // copy selected timetable into trainqueue2
                    }
                    printf("\r\nSelected timetable data\r\n");
                    for(x = 0; x < (y + 1); x++)
                    //for(x = 1; x < 20; x++)
                    {
                        for(z = 0; z < 5; z++) printf("%d ",trainqueue2[x][z] );
                        printf("\r\n");
                    }
                    printf("\r\n");
                    w = 50;
                }
                else w = w + 1;
            }
        }
        // for Demos, trains do not create automatically
        // for Passive, no trains
        if( mode == 13 || mode == 14 || mode == 15 ) for( x=1; x<5; x++ ) trainqueue[0][x] = 1;

        // TEMPORARY - disable slow line trains
        if( mode == 11 && m6503 == 0 ) for( x=1; x<3; x++ ) trainqueue[0][x] = 1;

        // activate Welwyn for Full Operation, Timetablee and Passive
        if( mode == 11 || mode == 12 || mode == 15 ) welwyn = 1;

        // set Welwyn N=1 at start-up
        block_input[7][0] = 1;
        block_input[15][0] = 1;

        /*
        printf("\r\nLever status\r\n");
        for(x=0; x<74; x++)
        {
            for(y=0; y<16; y++) printf(" %d", lever_status[x][y] );
            printf("\r\n");
        }
        printf("\r\nIndicator status\r\n");
        for(x=0; x<39; x++)
        {
            for(y=0; y<15; y++) printf(" %d", indicator_status[x][y] );
            printf("\r\n");
        }
        printf("\r\nIndicator address\r\n");
        for(x=0; x<46; x++)
        {
            for(y=0; y<2; y++) printf(" %d", ind_addr[x][y] );
            printf("\r\n");
        }
        printf("\r\nTC address\r\n");
        for(x=0; x<21; x++)
        {
            for(y=0; y<2; y++) printf(" %d", TC_addr[x][y] );
            printf("\r\n");
        }
        printf("\r\nblock_input\r\n");
        for(x=0; x<16; x++)
        {
            for(y=0; y<3; y++) printf(" %d", block_input[x][y] );
            printf("\r\n");
        }
        printf("\r\nbellcode\r\n");
        for(x=0; x<20; x++)
        {
            for(z=0; z<14; z++) printf(" %d", bellcode[x][z] );
            printf("\r\n");
        }
        printf("\r\nTC_status\r\n");
        for(x=0; x<24; x++)
        {
            for(z=0; z<6; z++) printf(" %d", TC_status[x][z] );
            printf("\r\n");
        }
        printf("\r\nroute (6 blocks - TC, yards, max speed, signal, DMU speed, reduce speed)\r\n");
        for(x=0; x<6; x++) for(y=1; y<10; y++)
            {
                for(z=0; z<9; z++) printf(" %d", route[x][z][y] );
                printf("\r\n");
            }
        */
        printf("\r\ntrain queue\r\n");
        for(x=1; x<5; x++)
        {
            for(y=0; y<7; y++) printf(" %d", trainqueue[y][x] );
            printf("\r\n");
        }
        printf("\r\ntimetable data\r\n");
        for(x=0; x<60; x++)
        {
            for(y=0; y<5; y++) printf(" %d", timetable[x][y] );
            printf("\r\n");
        }
    }
    return 0;
}

int random2(int max)
{
    int clk = clock0();
    return ( ( ((rand() + 200001 ) / ( 1 + rand())) * (( clk + 200000 ) % ( 1 + rand())) ) % 51734 % 3167 % max );
}

void delay0 (void)
{
  i=0;
  while(i < 40) i++;
}

void delay1 (void)
{
  i=0;
  while(i < 15) i++;
}

double clock0()
{
   struct timespec now;
   int clock_gettime(clockid_t, struct timespec *);
   double time1;
   clock_gettime(CLOCK_REALTIME, &now);
   time1 = now.tv_sec*1000 + now.tv_nsec/1000000.0;
   if(clock_flag == 0)
   {
       start_time = now.tv_sec*1000;
       clock_flag = 1;
   }
   return time1-start_time;
}

int serial_input_test(void)
{
    printf("\r\nSerial input test - ESC to finish\r\n");
    for(y=0; y<129; y++) input_test[y] = 0;  // clear test data before starting
loop_sit:
    serial_input();
    // for(y=0;y<18;y++) printf("%d ",USBinput[y]);
    // printf("\r\n");
    // for(y=1; y<97; y++) printf("%d",input_data[y]);
    // printf("\n");

    // check for change
    w = 1; z = 0; while( w < 129 )
    {
        if( input_data[w] != input_test[w] )
        {
            input_test[w] = input_data[w];
            z = 1;
        }
        w = w + 1;
    }
    if( z == 1 ) goto jump_sit;  // change
    usleep(100000);  // 100ms delay
    if( kbhit() )
    {
        x = getch();
        if( x == 27 || x == 13 ) return 0;  // char 27 is ESC
    }
        goto loop_sit;

    // print if change
jump_sit:
    w = 0; while( w < 24 )
    {
        z = 1; temp = 0; while( z < 5 )
        {
            temp = temp << 1;                        // shift bits left (it starts at 00000000)
            temp = temp + input_test[ w * 4 + 5 - z ];   // ... then add next bit
            z = z + 1;
        }
        if( temp < 10 ) printf("%d",temp);
        else if( temp == 10 ) printf("A");
        else if( temp == 11 ) printf("B");
        else if( temp == 12 ) printf("C");
        else if( temp == 13 ) printf("D");
        else if( temp == 14 ) printf("E");
        else if( temp == 15 ) printf("F");

        w = w + 1;
        if( w % 4 == 0 ) printf(" ");
    }
    printf("\r\n");

    goto loop_sit;
}

int serial_output_test(void)
{
    printf("\r\nSerial output test\r\n");
loop1_sot:
    for( x=0;x<121;x++ ) output_data[x] = 0;
    output_data[26] = 1;    // power indicator
    out_flag = 2;
    serial_output();
loop2_sot:
    printf("Enter ref 1 - 120; 0 to finish\r\n");
    scanf("%d",&x);
    if( x<0 || x>120 ) goto loop2_sot;
    if( x==0 ) return 0;
    printf("%d\r\n",x);
    output_data[x] = 1;
    fy = clock0();
    if( x==41 || x==42 || x==45 || x==46 ) fy = fy + 150;
    else if( x==43 || x==47 ) fy = fy + 300;
    else fy = fy + 1000;
    out_flag = 2;
    serial_output();
loop3_sot:
    usleep(10000);          // 10ms delay
    if( clock0() > fy ) goto loop1_sot;
    goto loop3_sot;
}
