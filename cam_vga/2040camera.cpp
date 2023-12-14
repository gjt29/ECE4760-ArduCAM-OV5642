/**
 * Grace Tang (gjt29), Jason Heller (jmh469), Noah Abramson (na325)
 * 
 * ArduCAM OV5642 RP2040 implementation
 * Uses PIO-assembly VGA driver
 *
 * HARDWARE CONNECTIONS
 *  VGA
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 330 ohm resistor ---> VGA Red
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - RP2040 GND ---> VGA GND
 *  
 *  ArduCAM
 *  - GPIO 2 ---> ArduCAM SCK
 *  - GPIO 3 ---> ArduCAM MOSI
 *  - GPIO 4 ---> ArduCAM MISO
 *  - GPIO 5 ---> ArduCAM CS
 *  - GPIO 8 ---> ArduCAM SDA
 *  - GPIO 9 ---> ArduCAM SCL
 *  - RP2040 GND ---> ArduCAM GND
 *  - RP2040 3.3V ---> ArduCAM VCC
 * 
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels 0 and 1
 *  
 */

// These files are in C but this is in C++ so need extern
extern "C"{
    #include "vga_graphics.h"
    #include <stdio.h>
    #include <stdlib.h>
    #include <string.h>
    #include "pico/stdlib.h"
    #include "hardware/pio.h"
    #include "hardware/dma.h"
    #include "pio_spi.h"
}

#include "hardware/clocks.h"
#include "ArduCAM.h"
#include "hardware/irq.h"
#include "ov5642_regs.h"
#include <cstdlib>
#include "stdio.h"
#include "pico/mutex.h"

// Include protothreads
#include "pt_cornell_rp2040_v1.h"

const uint8_t CS = 5;
ArduCAM myCAM( OV5642, CS );

//Used for toggling if color is enabled
volatile int color_enabled = 1;
//Used for toggling edge detection and saving those edges onto the RP2040
// 0 = no edge detection
// 1 = simple edge detection (looking at the previous pixels to determine if its an edge)
// 2 = lookback edge detection (looking at all pixels around a pixel to determine if its an edge)
volatile int edge_detection_en = 0;

//Used in the calculation for if an edge is present
//If in simple edge detection:
//If this number of pixels in a row (from left to right) are white, that position will be saved into edge_locations
//If in lookback edge detection:
//The number of pixels around a pixel that have to be white for an edge to be detected and saved into edge_locations
volatile int consecutive_threshold = 7;

//The minimum number of pixels between two solid pixels in edge detection, required to save memory
volatile int dithering_number = 3;

//Previous 3 rows (used for edge detection)
volatile bool prev_3_rows[3][640];
//The array that stores the locations of detected edges
volatile short edge_locations[2][22000];

pio_spi_inst_t spi = {
    .pio = pio0,
    .sm = 0,
    .cs_pin = PICO_DEFAULT_SPI_CSN_PIN
};

//This would be to store the full image if you had enough memory
//volatile bool image[480][640];

// ==================================================
// === users serial input thread
// ==================================================
static PT_THREAD (protothread_serial(struct pt *pt))
{
  PT_BEGIN(pt);
  // stores user input
  static char user_input ;
  // wait for 0.1 sec
  PT_YIELD_usec(1000000) ;
  // announce the threader version
  sprintf(pt_serial_out_buffer, "Protothreads RP2040 v1.0\n\r");
  // non-blocking write
  serial_write ;
    while(1) {
      // print prompt: 
      // b=border, t=turn factor, v=visual range, p=protected range, c=centering factor, 
      // a=avoid factor, m=matching, +=max speed, -=min speed
      sprintf(pt_serial_out_buffer, "Input command code:");
      
      // non-blocking write
      serial_write ;

      serial_read ;
      // convert input string to number
      sscanf(pt_serial_in_buffer,"%c", &user_input) ;

    // m : toggle color mode
    // c : contrast
    // b : brightness
    // l : lightness
    // f : flipping

    //EDGE DETECTION COMMANDS
    // n : change threshold for edge detection
    // e : toggle on dithering edge detection (when enabled, the edge locations will be saved into memory)
    // d : adjust dithering in edge detection, will allow for you to determine how many pixels there are in between 2 solid pixels
    // s : simple edge detection toggle on
    // r : Disable edge detection, show raw image on screen
    switch(user_input){
        case 'm':
            if(color_enabled){
                color_enabled = 0;
            }
            else{
                color_enabled = 1;
                edge_detection_en = 0;
            }
            break;
        case 'e':
            color_enabled = 0;
            edge_detection_en = 2;
            consecutive_threshold = 7;
            break;
        case 's':
            color_enabled = 0;
            edge_detection_en = 1;
            consecutive_threshold = 4;
            break;
        case 'r':
            color_enabled = 0;
            edge_detection_en = 0;
            break;
        case 'n':
            sprintf(pt_serial_out_buffer, "Input new consecutive threshold: ");
            serial_write ;
            serial_read ;
            // convert input string to number
            sscanf(pt_serial_in_buffer,"%i", &user_input) ;
            consecutive_threshold = user_input;
            break;
        case 'd':
            sprintf(pt_serial_out_buffer, "Input new number between 2 solids: ");
            serial_write ;
            serial_read ;
            // convert input string to number
            sscanf(pt_serial_in_buffer,"%i", &user_input) ;
            dithering_number = user_input;
            break;
        case 'c':
            sprintf(pt_serial_out_buffer, "Input new contrast value 0-8:");
            // non-blocking write
            serial_write ;

            serial_read ;
            // convert input string to number
            sscanf(pt_serial_in_buffer,"%c", &user_input) ;
                switch(user_input){
                    case '0':
                        myCAM.OV5642_set_Contrast(Contrast4);
                        break;
                    case '1':
                        myCAM.OV5642_set_Contrast(Contrast3);
                        break;
                    case '2':
                        myCAM.OV5642_set_Contrast(Contrast2);
                        break;
                    case '3':
                        myCAM.OV5642_set_Contrast(Contrast1);
                        break;
                    case '4':
                        myCAM.OV5642_set_Contrast(Contrast0);
                        break;
                    case '5':
                        myCAM.OV5642_set_Contrast(Contrast_1);
                        break;
                    case '6':
                        myCAM.OV5642_set_Contrast(Contrast_2);
                        break;
                    case '7':
                        myCAM.OV5642_set_Contrast(Contrast_3);
                        break;
                    case '8':
                        myCAM.OV5642_set_Contrast(Contrast_4);
                        break;
                }
                break;
        case 'b':
            sprintf(pt_serial_out_buffer, "Input new brightness value 0-8:");
            // non-blocking write
            serial_write ;

            serial_read ;
            // convert input string to number
            sscanf(pt_serial_in_buffer,"%c", &user_input) ;
                switch(user_input){
                    case '0':
                        myCAM.OV5642_set_Brightness(Brightness4);
                        break;
                    case '1':
                        myCAM.OV5642_set_Brightness(Brightness3);
                        break;
                    case '2':
                        myCAM.OV5642_set_Brightness(Brightness2);
                        break;
                    case '3':
                        myCAM.OV5642_set_Brightness(Brightness1);
                        break;
                    case '4':
                        myCAM.OV5642_set_Brightness(Brightness0);
                        break;
                    case '5':
                        myCAM.OV5642_set_Brightness(Brightness_1);
                        break;
                    case '6':
                        myCAM.OV5642_set_Brightness(Brightness_2);
                        break;
                    case '7':
                        myCAM.OV5642_set_Brightness(Brightness_3);
                        break;
                    case '8':
                        myCAM.OV5642_set_Brightness(Brightness_4);
                        break;
                }
                break;
        case 'f':
            sprintf(pt_serial_out_buffer, "Input 0 for mirror, 1 for flip, 2 for mirror flip, or 3 for normal:");
            // non-blocking write
            serial_write ;

            serial_read ;
            // convert input string to number
            sscanf(pt_serial_in_buffer,"%c", &user_input) ;
                switch(user_input){
                    case '0':
                        myCAM.OV5642_set_Mirror_Flip(MIRROR);
                        break;
                    case '1':
                        myCAM.OV5642_set_Mirror_Flip(FLIP);
                        break;
                    case '2':
                        myCAM.OV5642_set_Mirror_Flip(MIRROR_FLIP);
                        break;
                    case '3':
                        myCAM.OV5642_set_Mirror_Flip(Normal);
                }
                break;
        case 'l':
            sprintf(pt_serial_out_buffer, "Input new light setting value 0-8: \n"); 
            serial_write ;
            sprintf(pt_serial_out_buffer,"0 = Advanced_AWB, 1 = Simple_AWB, 2 = Manual_day, 3 = Manual_A, 4 = Manual_cwf, 5 = Manual_cloudy: ");
            // non-blocking write
            serial_write ;
            serial_read ;
            // convert input string to number
            sscanf(pt_serial_in_buffer,"%c", &user_input) ;
                switch(user_input){
                    case '0':
                        myCAM.OV5642_set_Light_Mode(Advanced_AWB);
                        break;
                    case '1':
                        myCAM.OV5642_set_Light_Mode(Simple_AWB);
                        break;
                    case '2': //makes much more noisy, brighter
                        myCAM.OV5642_set_Light_Mode(Manual_day);
                        break;
                    case '3':
                        myCAM.OV5642_set_Light_Mode(Manual_A);
                        break;
                    case '4':
                        myCAM.OV5642_set_Light_Mode(Manual_cwf);
                        break;
                    case '5': // Very noisy and mushed together colors
                        myCAM.OV5642_set_Light_Mode(Manual_cloudy);
                        break;
                }
                break;
        //ArduCAM has built in test patterns, but can't find what they are
        case 't':
            sprintf(pt_serial_out_buffer, "Input test pattern 0=color_bar, 1=color_square, 2=BW_square, 3=DLI: \n"); 
            serial_write ;
            serial_read ;
            // convert input string to number
            sscanf(pt_serial_in_buffer,"%c", &user_input) ;
                switch(user_input){
                    case '0':
                        myCAM.OV5642_Test_Pattern(Color_bar);
                        break;
                    case '1':
                        myCAM.OV5642_Test_Pattern(Color_square);
                        break;
                    case '2': //makes much more noisy, brighter
                        myCAM.OV5642_Test_Pattern(BW_square);
                        break;
                    case '3':
                        myCAM.OV5642_Test_Pattern(DLI);
                        break;
                }
                break;
        default:
            break;
    }  
  
    } // END WHILE(1)
  PT_END(pt);
} // timer thread

// Animation on core 0
static PT_THREAD (protothread_camera(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);
    uint8_t vid, pid;               //Used for identification of the camera
    uint8_t cameraCommand;   

    gpio_init(CS);
    gpio_set_dir(CS, GPIO_OUT);
    gpio_put(CS, 1);

    //Reset the CPLD
    myCAM.write_reg(0x07, 0x80);
    sleep_ms(100);
    myCAM.write_reg(0x07, 0x00);
    sleep_ms(100);

	while (1) 
    {
        //Check if the ArduCAM SPI bus is OK
        //Writes into the test register, and then reads from it
        //Stays in loop until successful SPI transaction
        myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
        cameraCommand = myCAM.read_reg(ARDUCHIP_TEST1);
        if (cameraCommand != 0x55) {
            printf(" SPI interface Error!");
            sleep_ms(1000); continue;
        } else {
            printf("ACK CMD SPI interface OK\n"); break;
        }
	}

    while (1) 
    {
        //Check if the camera module type is OV5640
        //Uses I2C to read the chip ID registers to determine model number
        //(if using different camera model, will need different IDs)
        myCAM.wrSensorReg16_8(0xff, 0x01);
        myCAM.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
        myCAM.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
        if((vid != 0x56) || (pid != 0x42))
        {
            printf("Can't find OV5642 module!\n");
            sleep_ms(1000); continue;
        }
        else 
        {
            printf("OV5642 detected\n"); break;
        }
    }

    //Change to RAW8 capture mode and initialize the OV5642 module
    myCAM.set_format(RAW);
    myCAM.InitCAM();
    myCAM.write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);    //VSYNC is active HIGH
    myCAM.OV5642_set_Contrast(Contrast_4);              //Max out contrast in image 
    myCAM.OV5642_set_Color_Saturation(Saturation_4);    //Max out saturation in image

    sleep_ms(1000);
    myCAM.write_reg(ARDUCHIP_FRAMES,0x00);  //FRAME control register, Number of frames to be captured
    printf("Starting capture loop\n");

    int num_consecutive = 0;
    int num_edges = 0;
    while(1){
        myCAM.flush_fifo();         //Clears out the previous capture
        myCAM.clear_fifo_flag();    //Clears the flag that an image is completed
        myCAM.start_capture();      //Start capture
        
        // Wait until capture is complete 
        while(myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK) == 0){}
        
        //Getting the length image buffer that the frame is loaded into 
        int length = myCAM.read_fifo_length();
        int count = 0;
        int first_pass = 1;
        if(edge_detection_en == 2){
            for(int i = 0; i < 640; i++){
                prev_3_rows[0][i] = 0;
                prev_3_rows[1][i] = 0;
                prev_3_rows[2][i] = 0;
            }
        }
        if(edge_detection_en){
            for(int i = 0; i < 10000; i++){
                edge_locations[0][i] = 0;
                edge_locations[1][i] = 0;
            }
        }
        for(int i =0; i < length ; i++){
            //Check to see if the first 3 rows are completed
            if(edge_detection_en == 2){
                if(i>=1917){
                    first_pass = 0;
                }
                else{
                    first_pass = 1;
                }
            }
            //Read in current pixel from the SPI FIFO
            uint8_t color = myCAM.read_fifo();

            // uint8_t blue = (int)(color%4)/2;
            // uint8_t green = (int)(((color - blue) >> 2)%16)/8;
            // uint8_t red = (int)((color - green - blue) >> 6)/2;

            //TRYING TO SHIFT THE DATA TO GET 1 BIT OF R G AND B (Good for BW)
            uint8_t red = (int)(color>>6)>>1;
            uint8_t green = (int)((color-(red<<6))>>4)>>1;
            uint8_t blue = (int)((color>>2)%4)>>1;

            //Color mode
            if(color_enabled){
                drawPixel(640- (i%640),480-((int)i/640),(red<<2)+(green<<1)+blue);
            }
            //BLACK "EDGE DETECTION"
            //Check if all of the bits of color data for a pixel are 0, if they are draw them in white
            else{
                if(edge_detection_en == 2){
                    //Add a dot to array
                    if((red<<2)+(green<<1)+blue == 0){
                        if(first_pass){
                            prev_3_rows[(int)(i/640)][(int)(i%640)] = 1;
                        }
                        else{
                            prev_3_rows[2][(int)(i%640)] = 1;
                        }
                    }
                    //Add no dot to array
                    else{
                        if(first_pass){
                            prev_3_rows[(int)(i/640)][(int)(i%640)] = 0;
                        }
                        else{
                            prev_3_rows[2][(int)(i%640)] = 0;
                        }
                    }
                    //Finished a row, not in the first pass, check for edges
                    if(i%639 == 0 && first_pass == 0){
                        int count_since_pixel = 0;
                        for(int j = 1; j <= 638; j++){
                            if(prev_3_rows[0][j-1] + prev_3_rows[0][j]+prev_3_rows[0][j+1]+prev_3_rows[1][j-1]+prev_3_rows[1][j+1]+prev_3_rows[2][j-1] + prev_3_rows[2][j]+prev_3_rows[2][j+1] > consecutive_threshold){
                                if(count_since_pixel == 0){
                                    edge_locations[0][num_edges] = (j%640);
                                    edge_locations[1][num_edges] = 480-(int)(i/640);
                                    num_edges = num_edges + 1;
                                    if(num_edges > 10000){
                                        num_edges = 0;
                                    }
                                    count_since_pixel = count_since_pixel + 1;
                                }
                                else{
                                    count_since_pixel = count_since_pixel + 1;
                                    if (count_since_pixel >= dithering_number){
                                        count_since_pixel = 0;
                                    }
                                }
                            }
                            else{
                                count_since_pixel = 0;
                            }
                        }
                        //Shift rows up
                        for(int j = 0; j < 640; j++){
                            prev_3_rows[0][j] = prev_3_rows[1][j];
                            prev_3_rows[1][j] = prev_3_rows[2][j];
                        }
                    }
                }
                //Simple edge detection
                else if(edge_detection_en == 1){
                    if((red<<2)+(green<<1)+blue == 0){
                        if(num_consecutive == consecutive_threshold){
                            edge_locations[0][num_edges] = 640- (i%640);
                            edge_locations[1][num_edges] = 480-((int)i/640);
                            num_edges = num_edges + 1;
                        }
                        num_consecutive = num_consecutive + 1;
                        if(num_consecutive >= 9999){
                            num_consecutive = 0;
                        }
                    }
                    else{
                        num_consecutive = 0;
                    }
                }
                else{
                    if((red<<2)+(green<<1)+blue == 0){
                        drawPixel(640-(i%640),480-((int)i/640),WHITE);
                    }
                    else{
                        drawPixel(640-(i%640),480-((int)i/640),BLACK);
                    }
                }
            }
            //The first 3 rows are filled, can start doing edge detection
            //VERY BASIC IMPLEMENTATAION (BAD)
            //drawPixel((i%640),480-((int)i/640),color>>5);
        }

        //Edge detection: Clear the screen and then draw the pixels of edges stored in edge_locations
        if(edge_detection_en != 0){
            for(int i = 0; i < 640; i ++){
                for(int j = 0; j < 480; j++){
                    drawPixel(i,j,BLACK);
                }
            }

            //Draw the edges to the screen
            for(int i = 0; i < 10000; i++){
                drawPixel(edge_locations[0][i],edge_locations[1][i],WHITE);
            }

            //Reset the edge location arrays
            for(int i = 0; i < 10000; i++){
                edge_locations[0][i] = 0;
                edge_locations[1][i] = 0;
            }
        }
        num_edges = 0;
        
        //Yield for a short amount of time for serial 
        PT_YIELD_usec(5000) ;
    }
    PT_END(pt);
} //Camera Thread

// ========================================
// === main
// ========================================
int main(){
    // initialize stio
    stdio_init_all() ;

    stdio_init_all();
    myCAM.Arducam_init();	        //Initialize camera
    initVGA() ;                     // Initialize VGA

    // add threads
    pt_add_thread(protothread_serial);
    pt_add_thread(protothread_camera);

    // start scheduler
    pt_schedule_start ;
} 