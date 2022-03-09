/***************************************************************************//**
 * @file
 * @brief Top level application functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/


#include "em_cmu.h"
#include "em_emu.h"
#include "em_iadc.h"
#include "em_ldma.h"
#include "em_prs.h"
#include "em_letimer.h"


/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

#define USE_LETIMER_AS_SAMPLING_TRIGGER 1

// How many samples to capture
#define NUM_SAMPLES               64

// Set CLK_ADC to 1 MHz
#define CLK_SRC_ADC_FREQ          20000000 // CLK_SRC_ADC
#define CLK_ADC_FREQ              4000000 // CLK_ADC

#if (USE_LETIMER_AS_SAMPLING_TRIGGER)
// ADC sample rate @2340.6Hz
#define SAMPLING_TIME_TICK                14u // top value for timer @ 32768Hz = CMU_ClockFreqGet(cmuClock_LETIMER0) / SAMPLING_FREQ_HZ
#define LETIMER_TRIGGER_PRS_CHANNEL       1         //PRS channel 1 as trigger for LETIMER operations
#else
// Set IADC timer cycles
#define TIMER_CYCLES              8545// expected 8545 to achieve the 2.4kHz sampling rate @20MHz src clock
#endif

/*
 * Specify the IADC input using the IADC_PosInput_t typedef.  This
 * must be paired with a corresponding macro definition that allocates
 * the corresponding ABUS to the IADC.  These are...
 *
 * GPIO->ABUSALLOC |= GPIO_ABUSALLOC_AEVEN0_ADC0
 * GPIO->ABUSALLOC |= GPIO_ABUSALLOC_AODD0_ADC0
 * GPIO->BBUSALLOC |= GPIO_BBUSALLOC_BEVEN0_ADC0
 * GPIO->BBUSALLOC |= GPIO_BBUSALLOC_BODD0_ADC0
 * GPIO->CDBUSALLOC |= GPIO_CDBUSALLOC_CDEVEN0_ADC0
 * GPIO->CDBUSALLOC |= GPIO_CDBUSALLOC_CDODD0_ADC0
 *
 * ...for port A, port B, and port C/D pins, even and odd, respectively.
 */
//#define IADC_INPUT_0_PORT_PIN     iadcPosInputPortAPin5;
//
//#define IADC_INPUT_0_BUS          ABUSALLOC
//#define IADC_INPUT_0_BUSALLOC     GPIO_ABUSALLOC_AODD0_ADC0

// analog recommanded to be on A or B for EM2.
#define IADC_INPUT_BUS0            BBUSALLOC
#define IADC_INPUT_BUS1            BBUSALLOC
#define IADC_INPUT_BUSALLOC0       GPIO_BBUSALLOC_BEVEN0_ADC0
#define IADC_INPUT_BUSALLOC1       GPIO_BBUSALLOC_BODD0_ADC0
#define IADC_INPUT_POS             iadcPosInputPortBPin2 // With WSK board P12 h15
#define IADC_INPUT_NEG             iadcNegInputPortBPin3 // With WSK board P13 h16

/*******************************************************************************
 ***************************   GLOBAL VARIABLES   *******************************
 ******************************************************************************/

// Globally declared LDMA link descriptor
LDMA_Descriptor_t descriptor;

// buffer to store IADC samples
uint32_t singleBuffer[NUM_SAMPLES];

/*******************************************************************************
 ***************************   GLOBAL DECLARATIONS  ****************************
 ******************************************************************************/

#if (USE_LETIMER_AS_SAMPLING_TRIGGER)
static void letimerInit(void)
{
  // Select LETimer0 clock to run off LFXO
  // Reference: EFR32xG22 RM, Figure 8.3
  CMU_ClockSelectSet(cmuClock_EM23GRPACLK, cmuSelect_LFXO);

  // Enable LETimer0 clock
  CMU_ClockEnable(cmuClock_LETIMER0, true);

  // Declare init struct
  LETIMER_Init_TypeDef init = LETIMER_INIT_DEFAULT;

  // Initialize letimer to run in free running mode
  // Reference: EFR32xG22 RM, Section 18.3.2
  init.repMode = letimerRepeatFree;

  // Pulse output for PRS
  init.ufoa0 = letimerUFOAPulse;

  // Set frequency
  //init.topValue = CMU_ClockFreqGet(cmuClock_LETIMER0) / LETIMER_TRIGGER_FREQ_HZ;
    // Warning : Trig each Topvalue+1, so minus 1 is needed, but not done in LETIMER_Init
    init.topValue = SAMPLING_TIME_TICK-1; //CMU_ClockFreqGet(cmuClock_LETIMER0) / SAMPLING_FREQ_HZ;


  // Enable letimer
  init.enable = true;
  init.debugRun = false;

  // Initialize free-running letimer
  LETIMER_Init(LETIMER0, &init);
}
#endif

#if (USE_LETIMER_AS_SAMPLING_TRIGGER)
static void prsInit(void)
{
  CMU_ClockEnable(cmuClock_PRS, true);


  // LETIMER --------- PRS CH1 --------> IADC0

  PRS_SourceAsyncSignalSet(LETIMER_TRIGGER_PRS_CHANNEL,
                           PRS_ASYNC_CH_CTRL_SOURCESEL_LETIMER0,
                           PRS_ASYNC_CH_CTRL_SIGSEL_LETIMER0CH0);
  // Select PRS channel 1 as trigger for IADC Single trigger
  PRS_ConnectConsumer(LETIMER_TRIGGER_PRS_CHANNEL,
                      prsTypeAsync,
                      prsConsumerIADC0_SINGLETRIGGER);
  // STR : disable pin output
  //GPIO_PinModeSet(gpioPortB, 2, gpioModePushPull,1);
  //PRS_PinOutput(LETIMER_TRIGGER_PRS_CHANNEL, prsTypeAsync, gpioPortB, 2);

}
#endif

/**************************************************************************//**
 * @brief  IADC Initializer
 *****************************************************************************/
void initIADC (void)
{
  // Declare init structs
  IADC_Init_t init = IADC_INIT_DEFAULT;
  IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
  IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
  IADC_SingleInput_t initSingleInput = IADC_SINGLEINPUT_DEFAULT;

  // Enable IADC clock
  CMU_ClockEnable(cmuClock_IADC0, true);

  // Reset IADC to reset configuration in case it has been modified
  IADC_reset(IADC0);

  // Configure IADC clock source for use while in EM2
  CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_FSRCO); // 20MHz
  //CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_EM23GRPACLK);

  // Modify init structs and initialize
  init.warmup = iadcWarmupNormal;

  // Set the HFSCLK prescale value here
  init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);

#if !(USE_LETIMER_AS_SAMPLING_TRIGGER)
  // Set timer cycles to configure sampling rate
  init.timerCycles = TIMER_CYCLES;
#endif

  // Configuration 0 is used by both scan and single conversions by default
  // Use unbuffered AVDD as reference
  initAllConfigs.configs[0].reference = iadcCfgReferenceInt1V2; //iadcCfgReferenceVddx; //TODO : review depending on power consumption
  initAllConfigs.configs[0].analogGain = (IADC_CfgAnalogGain_t) iadcCfgAnalogGain4x;    //TODO : review depending on power consumption

  // Divides CLK_SRC_ADC to set the CLK_ADC frequency
  // Default oversampling (OSR) is 2x, and Conversion Time = ((4 * OSR) + 2) / fCLK_ADC //TODO : review depending on power consumption
  initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
                                                                    CLK_ADC_FREQ,
                                                                    0,
                                                                    iadcCfgModeNormal,
                                                                    init.srcClkPrescale);

  initAllConfigs.configs[0].osrHighSpeed = (IADC_CfgOsrHighSpeed_t) iadcCfgOsrHighSpeed32x;//TODO : review depending on power consumption

  // Single initialization
  // On every trigger, start conversion
  initSingle.triggerAction = iadcTriggerActionOnce;

#if (USE_LETIMER_AS_SAMPLING_TRIGGER)
  // Set conversions to trigger from letimer/PRS
  initSingle.triggerSelect = iadcTriggerSelPrs0PosEdge;
#else
  // Set conversions to trigger from IADC internal timer
  initSingle.triggerSelect = iadcTriggerSelTimer;
#endif

  initSingle.dataValidLevel = iadcFifoCfgDvl4;

  // Set alignment to the left 16 bits
  initSingle.alignment = iadcAlignLeft16;

  // Enable triggering of single conversion
  initSingle.start = true;

  // Set to run in EM2
  initSingle.fifoDmaWakeup = true;

  // Configure Input sources for single ended conversion
  initSingleInput.posInput = IADC_INPUT_POS;
  initSingleInput.negInput = IADC_INPUT_NEG;

  // Initialize IADC
  IADC_init(IADC0, &init, &initAllConfigs);

  // Initialize Single
  IADC_initSingle(IADC0, &initSingle, &initSingleInput);

  // Allocate the analog bus for ADC0 inputs
  // Allocate the analog bus for IADC0 input
  GPIO->IADC_INPUT_BUS0 |= IADC_INPUT_BUSALLOC0;
  GPIO->IADC_INPUT_BUS1 |= IADC_INPUT_BUSALLOC1;
}

/**************************************************************************//**
 * @brief
 *   LDMA Initializer
 *
 * @param[in] buffer
 *   pointer to the array where ADC data will be stored.
 * @param[in] size
 *   size of the array
 *****************************************************************************/
void initLDMA(uint32_t *buffer, uint32_t size)
{
  // Declare LDMA init structs
  LDMA_Init_t init = LDMA_INIT_DEFAULT;

  // Enable LDMA clock branch
  CMU_ClockEnable(cmuClock_LDMA, true);

  // Initialize LDMA with default configuration
  LDMA_Init(&init);

  // Configure LDMA for transfer from IADC to memory
  // LDMA will loop continuously
  LDMA_TransferCfg_t transferCfg =
    LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_IADC0_IADC_SINGLE);

  // Set up descriptors for buffer transfer
  descriptor = (LDMA_Descriptor_t)LDMA_DESCRIPTOR_LINKREL_P2M_WORD(&IADC0->SINGLEFIFODATA, buffer, size, 0);

  // Set descriptor to loop NUM_SAMPLES times and run continuously
  descriptor.xfer.decLoopCnt = 0;
  descriptor.xfer.xferCnt = NUM_SAMPLES;
  descriptor.xfer.blockSize = ldmaCtrlBlockSizeUnit4;

  // Interrupt after transfer complete
  descriptor.xfer.doneIfs = 1;
  descriptor.xfer.ignoreSrec = 0;

  // Start transfer, LDMA will sample the IADC NUM_SAMPLES time, and then interrupt
  LDMA_StartTransfer(0, (void*)&transferCfg, (void*)&descriptor);
}

/**************************************************************************//**
 * @brief  LDMA Handler
 *****************************************************************************/
void LDMA_IRQHandler(void)
{
  // Clear interrupt flags
  LDMA_IntClear(LDMA_IF_DONE0);

//  // Re-enable GPIO clock branch (disabled prior to entering EM2)
//  CMU_ClockEnable(cmuClock_GPIO, true);
//
//  // Toggle GPIO to notify that transfer is complete
//  GPIO_PinOutToggle(gpioPortD, 2);
//
//  // Disable GPIO clock branch (shutting off unnecessary clock trees)
//  CMU_ClockEnable(cmuClock_GPIO, false);
}


/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
void app_init(void)
{

  // Initialize the IADC
  initIADC();

  // Initialize LDMA
  initLDMA(singleBuffer, NUM_SAMPLES);

#if (USE_LETIMER_AS_SAMPLING_TRIGGER)
  //Init LETIMER
  letimerInit();

  //Init PRS
  prsInit();
#else
  // IADC single already enabled; must enable timer block in order to trigger
  IADC_command(IADC0, iadcCmdEnableTimer);
#endif

}

/***************************************************************************//**
 * App ticking function.
 ******************************************************************************/
void app_process_action(void)
{

}
