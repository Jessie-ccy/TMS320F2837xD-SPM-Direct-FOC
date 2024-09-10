/*
 input_
 - ADCINA0 is on (PIN 30)
 - ADCINA1 is on (PIN 70)
 - ADCINA2 is on (PIN 29)
 - encoder is on J14
 output_
 - ePWM1A is on GPIO0 (PIN 40)
 - ePWM1B is on GPIO1 (PIN 39)
 - ePWM2A is on GPIO2 (PIN 38)
 - ePWM2B is on GPIO3 (PIN 37)
 - ePWM3A is on GPIO4 (PIN 36)
 - ePWM3B is on GPIO5 (PIN 35)
 */
#include "F28x_Project.h"

void InitEPwm1Example(void); // Initialize EPWM1
void InitEPwm2Example(void); // Initialize EPWM2
void InitEPwm3Example(void); // Initialize EPWM3
__interrupt void epwm1_isr(void); // EPWM interrupt, base on switching frequency
void ConfigureADC(void);
void SetupADCEpwm(void);
void Eqep1_Init(void);
void POSSPEED_Calc(void);
interrupt void adca1_isr(void);

// Globals
const float SwitchingFreq = 10000; // Switching frequency
const float EPWM_DB = 150; // Deadband pulse number, EPWM_DB=BaseClock(Hz)*Deadtime(sec)
int TimeBase;
double t_err;
double ia, ib, ic;
double i_max = 45.0;
double ialpha, ibeta, id, iq;
double valpha, vbeta, vd, vq;
int theta_m, theta_e, theta_e_cos;
int encodercount, encoderoldcount, encoderdirect;
//¡õ if "double" may out of memory size
int sine_table[360] = { 10000, 10175, 10349, 10523, 10698, 10872, 11045, 11219,
                        11392, 11564, 11736, 11908, 12079, 12250, 12419, 12588,
                        12756, 12924, 13090, 13256, 13420, 13584, 13746, 13907,
                        14067, 14226, 14384, 14540, 14695, 14848, 15000, 15150,
                        15299, 15446, 15592, 15736, 15878, 16018, 16157, 16293,
                        16428, 16561, 16691, 16820, 16947, 17071, 17193, 17314,
                        17431, 17547, 17660, 17771, 17880, 17986, 18090, 18192,
                        18290, 18387, 18480, 18572, 18660, 18746, 18829, 18910,
                        18988, 19063, 19135, 19205, 19272, 19336, 19397, 19455,
                        19511, 19563, 19613, 19659, 19703, 19744, 19781, 19816,
                        19848, 19877, 19903, 19925, 19945, 19962, 19976, 19986,
                        19994, 19998, 20000, 19998, 19994, 19986, 19976, 19962,
                        19945, 19925, 19903, 19877, 19848, 19816, 19781, 19744,
                        19703, 19659, 19613, 19563, 19511, 19455, 19397, 19336,
                        19272, 19205, 19135, 19063, 18988, 18910, 18829, 18746,
                        18660, 18572, 18480, 18387, 18290, 18192, 18090, 17986,
                        17880, 17771, 17660, 17547, 17431, 17314, 17193, 17071,
                        16947, 16820, 16691, 16561, 16428, 16293, 16157, 16018,
                        15878, 15736, 15592, 15446, 15299, 15150, 15000, 14848,
                        14695, 14540, 14384, 14226, 14067, 13907, 13746, 13584,
                        13420, 13256, 13090, 12924, 12756, 12588, 12419, 12250,
                        12079, 11908, 11736, 11564, 11392, 11219, 11045, 10872,
                        10698, 10523, 10349, 10175, 10000, 9825, 9651, 9477,
                        9302, 9128, 8955, 8781, 8608, 8436, 8264, 8092, 7921,
                        7750, 7581, 7412, 7244, 7076, 6910, 6744, 6580, 6416,
                        6254, 6093, 5933, 5774, 5616, 5460, 5305, 5152, 5000,
                        4850, 4701, 4554, 4408, 4264, 4122, 3982, 3843, 3707,
                        3572, 3439, 3309, 3180, 3053, 2929, 2807, 2686, 2569,
                        2453, 2340, 2229, 2120, 2014, 1910, 1808, 1710, 1613,
                        1520, 1428, 1340, 1254, 1171, 1090, 1012, 937, 865, 795,
                        728, 664, 603, 545, 489, 437, 387, 341, 297, 256, 219,
                        184, 152, 123, 97, 75, 55, 38, 24, 14, 6, 2, 0, 2, 6,
                        14, 24, 38, 55, 75, 97, 123, 152, 184, 219, 256, 297,
                        341, 387, 437, 489, 545, 603, 664, 728, 795, 865, 937,
                        1012, 1090, 1171, 1254, 1340, 1428, 1520, 1613, 1710,
                        1808, 1910, 2014, 2120, 2229, 2340, 2453, 2569, 2686,
                        2807, 2929, 3053, 3180, 3309, 3439, 3572, 3707, 3843,
                        3982, 4122, 4264, 4408, 4554, 4701, 4850, 5000, 5152,
                        5305, 5460, 5616, 5774, 5933, 6093, 6254, 6416, 6580,
                        6744, 6910, 7076, 7244, 7412, 7581, 7750, 7921, 8092,
                        8264, 8436, 8608, 8781, 8955, 9128, 9302, 9477, 9651,
                        9825 };
double sin, cos;
double speed_fb = 0;
Uint16 dir;                         // direction  0:reverse 1:forward
int oldpos = 0;
Uint16 encres = 4096;           // encoder resolution
double encunit;                  // 360 / encoder resolution
double adcunit = 0.0004883;     // 1/2048(half of 12 bits)

// Motor parameters
double vdc = 220.0; //unit:V
double vdcrec;   // =1 / vdc
double Rs = 0.1416; //unit:ohm
double Ld = 0.00076; //unit:H
double Lq = 0.00161; //unit:H
int PolePair = 4;
double Kt_rec = 2.083;  // 1/Kt
double J = 0.00633;
double B = 0.002;

// Controller parameters
double fbw_w = 1;
double fbw_i = 100;
double kp_w, ki_w, kp_d, ki_d, kp_q, ki_q;
double speed_comm, iq_comm, id_comm;
double s_w = 0;
double s_d = 0;
double s_q = 0;

//SVPWM parameters
int SectionTemp, Section;
float X, Y, Z, T1, T2, T0, Ta, Tb, Tc;

//test signal
double TestMechPosition = 0;
double TestCLC = 0;
double Testa = 0, Testb = 0;
int TestCurrentMaker = 0;
double TestIa = 0, TestIb = 0, TestIc = 0;
int TestTheta_eA = 0, TestTheta_eB = 0, TestTheta_eC = 0;

/**
 * main.c
 */
void main(void)
{
    // Initialize system control
    InitSysCtrl();

    // Initialize GPIO pins for ePWM1, ePWM2, ePWM3, eqep1
    InitEPwm1Gpio();
    InitEPwm2Gpio();
    InitEPwm3Gpio();
    InitEQep1Gpio();
    // enable PWM1, PWM2 and PWM3
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM3 = 1;

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags are cleared.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    InitPieVectTable();

    // Setting interrupt linked
    EALLOW;
    PieVectTable.EPWM1_INT = &epwm1_isr;
    EDIS;

    // Variable calculations
    TimeBase = 50000000.0 / 2 / SwitchingFreq; //  50M/2/10k=2.5k  Page115 for "/2"
    t_err = 1.0 / SwitchingFreq;
    encunit = 360.0 / encres;
    vdcrec = 1 / vdc;
    kp_w = 2.0 * 3.1415926 * fbw_w * J;
    ki_w = 2.0 * 3.1415926 * fbw_w * B;
    kp_d = 2.0 * 3.1415926 * fbw_i * Ld;
    ki_d = 2.0 * 3.1415926 * fbw_i * Rs;
    kp_q = 2.0 * 3.1415926 * fbw_i * Lq;
    ki_q = 2.0 * 3.1415926 * fbw_i * Rs;

    // Initialize all the Device Peripherals
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0; // Closed synchronize between base clock and all enabled ePWM modules
    EDIS;

    InitEPwm1Example();
    InitEPwm2Example();
    InitEPwm3Example();

    ConfigureADC();
    SetupADCEpwm();

    Eqep1_Init();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1; // Enable synchronize between base clock and all enabled ePWM modules
    EDIS;

    IER |= M_INT3; //for epwm interrupt
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1; // Base on PIE MUXed vector table, enable EPWM1 interrupt

    // Enable Global interrupt INTM
    EINT;
    //Enable Global realtime interrupt DBGM
    ERTM;

    speed_comm = 1000.0;    //speed command

    while (1)
    {
        //empty
    }
}

//
// InitEPwm1Example - Initialize EPWM1 configuration
//
void InitEPwm1Example()
{
    EPwm1Regs.TBPRD = TimeBase;                       // Set timer period
    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;                     // Clear counter

    // Setup TBCLK
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count UPDOWN
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;  // TBCLK=SYSCLKOUT/(HSPCLKDIV¡ÑCLKDIV)

    // Setup compare
    EPwm1Regs.CMPA.bit.CMPA = TimeBase;
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;           // If counter>CMPA, output high
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;          // If counter<CMPA, output low

    // Active Low PWMs - Setup Deadband
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // Set deadband mode: Both rising and falling edge
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // Active high mode. Neither EPWMxA nor EPWMxB is inverted
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL; // EPWMxA In (from the action-qualifier) is the source for both falling-edge and rising-edge delay.
    EPwm1Regs.DBRED.bit.DBRED = EPWM_DB; // Setting Dead-Band Generator Rising Edge Delay Count Register
    EPwm1Regs.DBFED.bit.DBFED = EPWM_DB; // Setting Dead-Band Generator Falling Edge Delay Count Register

    //Configure SOC
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;    // Enable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = 2;   // Select SOC on period
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;       // Generate pulse on 1st event

    // Setup interrupt
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;    // Select INT on Zero event
    EPwm1Regs.ETSEL.bit.INTEN = 1;               // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST; // Generate an interrupt on the first event

}

//
// InitEPwm2Example - Initialize EPWM2 configuration
//
void InitEPwm2Example()
{
    EPwm2Regs.TBPRD = TimeBase;                       // Set timer period
    EPwm2Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
    EPwm2Regs.TBCTR = 0x0000;                     // Clear counter

    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count UPDOWN
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm2Regs.CMPA.bit.CMPA = TimeBase;
    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM2A on Zero
    EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;

    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm2Regs.DBRED.bit.DBRED = EPWM_DB;
    EPwm2Regs.DBFED.bit.DBFED = EPWM_DB;

    EPwm2Regs.ETSEL.bit.INTEN = 0;                // Disable interrupt

}

//
// InitEPwm3Example - Initialize EPWM3 configuration
//
void InitEPwm3Example()
{
    EPwm3Regs.TBPRD = TimeBase;
    EPwm3Regs.TBPHS.bit.TBPHS = 0x0000;
    EPwm3Regs.TBCTR = 0x0000;

    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm3Regs.CMPA.bit.CMPA = TimeBase;
    EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;
    EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;

    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm3Regs.DBRED.bit.DBRED = EPWM_DB;
    EPwm3Regs.DBFED.bit.DBFED = EPWM_DB;

    EPwm3Regs.ETSEL.bit.INTEN = 0;                 // Disable interrupt
}

void ConfigureADC(void)
{
    EALLOW;

    //
    //write configurations
    //
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    //
    //Set pulse positions to late
    //
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
    //power up the ADC
    //
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //
    //delay for 1ms to allow ADC time to power up
    //
    DELAY_US(1000);

    EDIS;
}

//
// SetupADCEpwm - Setup ADC EPWM acquisition window
//
void SetupADCEpwm(void)
{
    EALLOW;
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;  //SOC0 will convert pin A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 1;  //SOC1 will convert pin A1
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C
    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 2;  //SOC2 will convert pin A2
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;
}

void Eqep1_Init(void)
{
    EQep1Regs.QUPRD = 500000; // Unit Timer for 200Hz at 100 MHz, for speed calculation
                              // SYSCLKOUT
    EQep1Regs.QDECCTL.bit.QSRC = 00;      // QEP quadrature count mode
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2;
    EQep1Regs.QEPCTL.bit.PCRM = 00;       // PCRM=00 mode - QPOSCNT reset on
                                          // index event
    EQep1Regs.QEPCTL.bit.UTE = 1;  // Unit Timeout Enable, for speed calculation
    EQep1Regs.QEPCTL.bit.QCLM = 1; // Latch on unit time out, for speed calculation
    EQep1Regs.QPOSMAX = encres - 1;       // Encoder resolution
    EQep1Regs.QEPCTL.bit.QPEN = 1;        // QEP enable
}

void POSSPEED_Calc(void)
{
//    TestCLC++;            //for no encoder
//    if (TestCLC > (100))  //if its not work ,increase the number
//    {
//        TestCLC = 0;
//
//        TestMechPosition++;
//        if (TestMechPosition > (EQep1Regs.QPOSMAX))
//        {
//            TestMechPosition = 0;
//        }
//
//    }
//
//    encodercount = TestMechPosition;
//    encoderoldcount = encodercount - 1000;
//    if (encoderoldcount < 0)
//    {
//        encoderoldcount = encoderoldcount + EQep1Regs.QPOSMAX;
//    }
//
//    encoderdirect = 1;


     // Motor direction:
     // 0=reverse, 1=forward
     encoderdirect = EQep1Regs.QEPSTS.bit.QDF;
     encodercount = EQep1Regs.QPOSCNT;
     encoderoldcount = EQep1Regs.QPOSLAT;


    theta_m = encodercount * encunit;
    theta_e = theta_m * PolePair;
    while (theta_e >= 360)
    {
        theta_e -= 360;
    }
    theta_e_cos = theta_e + 90;   //cos = sin + 90

    while (theta_e_cos >= 360)
    {
        theta_e_cos -= 360;
    }
    sin = 0.0001 * sine_table[theta_e] - 1.0;

    cos = 0.0001 * sine_table[theta_e_cos] - 1.0;

    dir = encoderdirect;
    // 0=reverse, 1=forward



    /////////////
//    TestTheta_eA = theta_e+90;
//    while (TestTheta_eA > 360)
//    {
//        TestTheta_eA -= 360;
//    }
//    TestIa = (0.0001 * sine_table[TestTheta_eA] - 1.0) * 2000+2048;
//
//    TestTheta_eB = theta_e + 240+90;
//    while (TestTheta_eB > 360)
//    {
//        TestTheta_eB -= 360;
//    }
//    TestIb = (0.0001 * sine_table[TestTheta_eB] - 1.0) * 2000+2048;
//    TestTheta_eC = theta_e + 120+90;
//    while (TestTheta_eC > 360)
//    {
//        TestTheta_eC -= 360;
//    }
//    TestIc = (0.0001 * sine_table[TestTheta_eC] - 1.0) * 2000+2048;

    ///////////

    //SPEED
    if (EQep1Regs.QFLG.bit.UTO == 1)
    {                                   //calculate the speed every 1/200 second
        if (dir == 1)
        {
            if (encoderoldcount >= oldpos) // QPOSLAT records the position when the counter counts to QUPRD.
                speed_fb = (encoderoldcount - oldpos) * encunit * 33.3333; // degree / (1/200) / 360 * 60 = rpm
            else
                speed_fb = (encoderoldcount + EQep1Regs.QPOSMAX - oldpos)
                        * encunit * 33.3333;
        }
        else
        {
            if (encoderoldcount > oldpos)
                speed_fb = (encoderoldcount - oldpos - encoderoldcount)
                        * encunit * 33.3333;
            else
                speed_fb = (encoderoldcount - oldpos) * encunit * 33.3333;
        }

        oldpos = encoderoldcount;
        EQep1Regs.QCLR.bit.UTO = 1;                        // clear the UTO flag
    }
    //It works like how many counts in fixed time.
    //Where count is "<new data>EQep1Regs.QPOSLAT - oldpos<old data>", and fixed time is "QUPRD", which is the period to read encoder.
}
//
// adca1_isr - Read ADC Buffer in ISR
//
interrupt void epwm1_isr(void)
{
    //speed_comm = 500.0;    //speed command
    POSSPEED_Calc();

     //Calculate the current # Remember to adjust the value
     ia = AdcaResultRegs.ADCRESULT0 * adcunit * i_max - i_max;   //-i_max ~ i_max
     ib = AdcaResultRegs.ADCRESULT1 * adcunit * i_max - i_max;
     ic = AdcaResultRegs.ADCRESULT2 * adcunit * i_max - i_max;

    //test
//    ia = TestIa * adcunit * i_max - i_max;   //-i_max ~ i_max
//    ib = TestIb * adcunit * i_max - i_max;
//    ic = TestIc * adcunit * i_max - i_max;

    //Clarke transform
    ialpha = 0.6667 * ia - 0.3333 * ib - 0.3333 * ic;
    ibeta = 0.57735 * ib - 0.57735 * ic;
    //Park transform
    id = ialpha * cos + ibeta * sin;
    iq = -ialpha * sin + ibeta * cos;

    //Speed loop
    s_w += (speed_comm - speed_fb) * t_err * Kt_rec;
    iq_comm = kp_w * (speed_comm - speed_fb)
            + ki_w * s_w * 3.1415926 * 0.0333333;

    if (iq_comm > i_max)
    {
        iq_comm = i_max;
    }
    if (iq_comm < -i_max)
    {
        iq_comm = -i_max;
    }

    //Current loop
    s_d += (id_comm - id) * t_err;
    s_q += (iq_comm - iq) * t_err;
    vd = kp_d * (id_comm - id) + ki_d * s_d;
    if (vd > (vdc * 0.577))
    {
        vd = vdc * 0.577;
    }
    else if (vd < (-vdc * 0.577))
    {
        vd = -vdc * 0.577;
    }
    vq = kp_q * (iq_comm - iq) + ki_q * s_q;
    if (vq > (vdc * 0.577))
    {
        vq = vdc * 0.577;
    }
    else if (vq < (-vdc * 0.577))
    {
        vq = -vdc * 0.577;
    }

    // Inverse Park transform
    valpha = vd * cos - vq * sin;
    vbeta = vd * sin + vq * cos;
    // SVPWM
    // Determine section
    SectionTemp = 0;  //read the command and change into each section
    if (vbeta > 0)
        SectionTemp += 1;
    if ((valpha * 1.73205 - vbeta) > 0)
        SectionTemp += 2;
    if ((-vbeta - 1.73205 * valpha) > 0)
        SectionTemp += 4;
//use table or case to do logical switch
    switch (SectionTemp)
    {
    case 1:
        Section = 2;
        break;
    case 2:
        Section = 6;
        break;
    case 3:
        Section = 1;
        break;
    case 4:
        Section = 4;
        break;
    case 5:
        Section = 3;
        break;
    case 6:
        Section = 5;
        break;
    }

    X = vbeta * 1.73205 * vdcrec;
    Y = (valpha * 1.5 + vbeta * 0.866025) * vdcrec;
    Z = (valpha * 1.5 - vbeta * 0.866025) * vdcrec;

    switch (Section)
    {
    case 1:
        T1 = Z;
        T2 = X;
        break;
    case 2:
        T1 = Y;
        T2 = -Z;
        break;
    case 3:
        T1 = X;
        T2 = -Y;
        break;
    case 4:
        T1 = -Z;
        T2 = -X;
        break;
    case 5:
        T1 = -Y;
        T2 = Z;
        break;
    case 6:
        T1 = -X;
        T2 = Y;
        break;
    }

    if ((T1 + T2) > 1)
    {
        T1 = T1 / (T1 + T2);
        T2 = 1 - T1;
    }

    T0 = (1 - T1 - T2) * 0.5;

    switch (Section)
    {
    case 1:
        Ta = T1 + T2 + T0;
        Tb = T2 + T0;
        Tc = T0;
        break;
    case 2:
        Ta = T1 + T0;
        Tb = T1 + T2 + T0;
        Tc = T0;
        break;
    case 3:
        Ta = T0;
        Tb = T1 + T2 + T0;
        Tc = T2 + T0;
        break;
    case 4:
        Ta = T0;
        Tb = T1 + T0;
        Tc = T1 + T2 + T0;
        break;
    case 5:
        Ta = T2 + T0;
        Tb = T0;
        Tc = T1 + T2 + T0;
        break;
    case 6:
        Ta = T1 + T2 + T0;
        Tb = T0;
        Tc = T1 + T0;
        break;
    }

    // Set time compare register
    EPwm1Regs.CMPA.bit.CMPA = TimeBase * (1.0 - Ta);
    EPwm2Regs.CMPA.bit.CMPA = TimeBase * (1.0 - Tb);
    EPwm3Regs.CMPA.bit.CMPA = TimeBase * (1.0 - Tc);

    // Clear interrupt
    EPwm1Regs.ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

//
// End of file
//
