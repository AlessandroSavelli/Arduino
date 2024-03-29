// ATtiny85 sleep mode, wake on pin change interrupt demo
// Author: Nick Gammon
// Date: 12 October 2013

// ATMEL ATTINY 25/45/85 / ARDUINO
//
//                  +-\/-+
// Ain0 (D 5) PB5  1|    |8  Vcc
// Ain3 (D 3) PB3  2|    |7  PB2 (D 2) Ain1
// Ain2 (D 4) PB4  3|    |6  PB1 (D 1) pwm1
//            GND  4|    |5  PB0 (D 0) pwm0
//                  +----+
#include <avr/sleep.h>    // Sleep Modes
#include <avr/power.h>    // Power management
#include <TinyWireM.h>
#include <Tiny4kOLED.h>
#include <avr/pgmspace.h>
#include "Arduino.h"

const uint8_t ssd1306xled_font16x32_digits [] PROGMEM = {
  // @0 ',' (16 pixels wide)
  0x00, //          
  0x00, //          
  0x00, //          
  0x00, //          
  0x00, //          
  0x00, //  #       
  0x00, // ###   ###
  0x00, //  ########
  0x00, //   #######
  0x00, //          
  0x00, //          
  0x00, //          
  0x00, //          
  0x00, //          
  0x00, //          
  0x00, //          

  0x00, //          
  0x00, //          
  0x00, //          
  0x00, //          
  0x00, //          
  0x00, //  #       
  0x00, // ###   ###
  0x00, //  ########
  0x00, //   #######
  0x00, //          
  0x00, //          
  0x00, //          
  0x00, //          
  0x00, //          
  0x00, //          
  0x00, //          

  0x00, //          
  0x00, //          
  0x00, //          
  0x00, //          
  0x00, //          
  0x00, //  #       
  0x80, // ###   ###
  0x80, //  ########
  0x80, //   #######
  0x00, //          
  0x00, //          
  0x00, //          
  0x00, //          
  0x00, //          
  0x00, //          
  0x00, //          

  0x00, //          
  0x00, //          
  0x00, //          
  0x00, //          
  0x00, //          
  0x40, //  #       
  0xE3, // ###   ###
  0x7F, //  ########
  0x3F, //   #######
  0x00, //          
  0x00, //          
  0x00, //          
  0x00, //          
  0x00, //          
  0x00, //          
  0x00, //          

  // @32 '-' (16 pixels wide)
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //                 ###
  0x00, //                 ###
  0x00, //                 ###
  0x00, //                 ###
  0x00, //                 ###
  0x00, //                 ###
  0x00, //                 ###
  0x00, //                 ###
  0x00, //                 ###
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    

  0x00, //    
  0x00, //    
  0x00, //    
  0xE0, //                 ###
  0xE0, //                 ###
  0xE0, //                 ###
  0xE0, //                 ###
  0xE0, //                 ###
  0xE0, //                 ###
  0xE0, //                 ###
  0xE0, //                 ###
  0xE0, //                 ###
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    

  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //                 ###
  0x00, //                 ###
  0x00, //                 ###
  0x00, //                 ###
  0x00, //                 ###
  0x00, //                 ###
  0x00, //                 ###
  0x00, //                 ###
  0x00, //                 ###
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    

  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //                 ###
  0x00, //                 ###
  0x00, //                 ###
  0x00, //                 ###
  0x00, //                 ###
  0x00, //                 ###
  0x00, //                 ###
  0x00, //                 ###
  0x00, //                 ###
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    

  // @48 '.' (16 pixels wide)
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, // ###
  0x00, // ###
  0x00, // ###
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    

  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, // ###
  0x00, // ###
  0x00, // ###
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    

  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, // ###
  0x00, // ###
  0x00, // ###
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    

  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    
  0xE0, // ###
  0xE0, // ###
  0xE0, // ###
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    
  0x00, //    

  // @64 '/' (16 pixels wide)
  0x00, //                              
  0x00, //                              
  0x00, // #                            
  0x00, // #####                        
  0x00, // #########                    
  0x00, //  ############                
  0x00, //      ############            
  0x00, //          ############        
  0x80, //              ############    
  0xF8, //                  ############
  0xF8, //                      ########
  0x78, //                          ####
  0x00, //                              
  0x00, //                              
  0x00, //                              
  0x00, //                              

  0x00, //                              
  0x00, //                              
  0x00, // #                            
  0x00, // #####                        
  0x00, // #########                    
  0x00, //  ############                
  0x80, //      ############            
  0xF8, //          ############        
  0xFF, //              ############    
  0x7F, //                  ############
  0x07, //                      ########
  0x00, //                          ####
  0x00, //                              
  0x00, //                              
  0x00, //                              
  0x00, //                              

  0x00, //                              
  0x00, //                              
  0x00, // #                            
  0x00, // #####                        
  0x80, // #########                    
  0xF8, //  ############                
  0xFF, //      ############            
  0x7F, //          ############        
  0x07, //              ############    
  0x00, //                  ############
  0x00, //                      ########
  0x00, //                          ####
  0x00, //                              
  0x00, //                              
  0x00, //                              
  0x00, //                              

  0x00, //                              
  0x00, //                              
  0x80, // #                            
  0xF8, // #####                        
  0xFF, // #########                    
  0x7F, //  ############                
  0x07, //      ############            
  0x00, //          ############        
  0x00, //              ############    
  0x00, //                  ############
  0x00, //                      ########
  0x00, //                          ####
  0x00, //                              
  0x00, //                              
  0x00, //                              
  0x00, //                              

  // @128 '0' (16 pixels wide)
  0x00, //         #############        
  0x00, //     ####################     
  0xC0, //    #######################   
  0xE0, //   ######             ######  
  0xF0, //  ####                   #### 
  0x78, // ####                     ####
  0x38, // ###                       ###
  0x38, // ###                       ###
  0x38, // ###                       ###
  0x78, // ####                     ####
  0xF0, //  ####                   #### 
  0xE0, //   ######             ######  
  0xC0, //    #######################   
  0x00, //      ###################     
  0x00, //         #############        
  0x00, //                              

  0xF8, //         #############        
  0xFF, //     ####################     
  0xFF, //    #######################   
  0x07, //   ######             ######  
  0x00, //  ####                   #### 
  0x00, // ####                     ####
  0x00, // ###                       ###
  0x00, // ###                       ###
  0x00, // ###                       ###
  0x00, // ####                     ####
  0x00, //  ####                   #### 
  0x07, //   ######             ######  
  0xFF, //    #######################   
  0xFF, //      ###################     
  0xF8, //         #############        
  0x00, //                              

  0xFF, //         #############        
  0xFF, //     ####################     
  0xFF, //    #######################   
  0x00, //   ######             ######  
  0x00, //  ####                   #### 
  0x00, // ####                     ####
  0x00, // ###                       ###
  0x00, // ###                       ###
  0x00, // ###                       ###
  0x00, // ####                     ####
  0x00, //  ####                   #### 
  0x00, //   ######             ######  
  0xFF, //    #######################   
  0xFF, //      ###################     
  0xFF, //         #############        
  0x00, //                              

  0x00, //         #############        
  0x0F, //     ####################     
  0x1F, //    #######################   
  0x3F, //   ######             ######  
  0x78, //  ####                   #### 
  0xF0, // ####                     ####
  0xE0, // ###                       ###
  0xE0, // ###                       ###
  0xE0, // ###                       ###
  0xF0, // ####                     ####
  0x78, //  ####                   #### 
  0x3F, //   ######             ######  
  0x1F, //    #######################   
  0x07, //      ###################     
  0x00, //         #############        
  0x00, //                              

  // @192 '1' (16 pixels wide)
  0x00, //                              
  0x00, //                              
  0x00, //                   ###        
  0x00, //                    ###       
  0x00, //                     ###      
  0x00, //                     ####     
  0x80, //                      ####    
  0xC0, //                       ####   
  0xF0, // ############################ 
  0xF8, // #############################
  0xF8, // #############################
  0x00, //                              
  0x00, //                              
  0x00, //                              
  0x00, //                              
  0x00, //                              

  0x00, //                              
  0x00, //                              
  0x38, //                   ###        
  0x1C, //                    ###       
  0x0E, //                     ###      
  0x0F, //                     ####     
  0x07, //                      ####    
  0x03, //                       ####   
  0xFF, // ############################ 
  0xFF, // #############################
  0xFF, // #############################
  0x00, //                              
  0x00, //                              
  0x00, //                              
  0x00, //                              
  0x00, //                              

  0x00, //                              
  0x00, //                              
  0x00, //                   ###        
  0x00, //                    ###       
  0x00, //                     ###      
  0x00, //                     ####     
  0x00, //                      ####    
  0x00, //                       ####   
  0xFF, // ############################ 
  0xFF, // #############################
  0xFF, // #############################
  0x00, //                              
  0x00, //                              
  0x00, //                              
  0x00, //                              
  0x00, //                              

  0x00, //                              
  0x00, //                              
  0x00, //                   ###        
  0x00, //                    ###       
  0x00, //                     ###      
  0x00, //                     ####     
  0x00, //                      ####    
  0x00, //                       ####   
  0xFF, // ############################ 
  0xFF, // #############################
  0xFF, // #############################
  0x00, //                              
  0x00, //                              
  0x00, //                              
  0x00, //                              
  0x00, //                              

  // @256 '2' (16 pixels wide)
  0x00, // ###                 ###      
  0xC0, // #####               ######   
  0xE0, // #######             #######  
  0xF0, // ########               ##### 
  0x70, // ### #####                ### 
  0x78, // ###  #####               ####
  0x38, // ###    ####               ###
  0x38, // ###     ####              ###
  0x38, // ###      ####             ###
  0x38, // ###       #####           ###
  0x78, // ###        #####         ####
  0xF0, // ###         ######     ##### 
  0xE0, // ###           #############  
  0xC0, // ###            ###########   
  0x00, // ###               ######     
  0x00, //                              

  0x0E, // ###                 ###      
  0x0F, // #####               ######   
  0x0F, // #######             #######  
  0x01, // ########               ##### 
  0x00, // ### #####                ### 
  0x00, // ###  #####               ####
  0x00, // ###    ####               ###
  0x00, // ###     ####              ###
  0x00, // ###      ####             ###
  0x00, // ###       #####           ###
  0x00, // ###        #####         ####
  0xC1, // ###         ######     ##### 
  0xFF, // ###           #############  
  0xFF, // ###            ###########   
  0x3F, // ###               ######     
  0x00, //                              

  0x00, // ###                 ###      
  0x00, // #####               ######   
  0x00, // #######             #######  
  0x00, // ########               ##### 
  0x80, // ### #####                ### 
  0xC0, // ###  #####               ####
  0xE0, // ###    ####               ###
  0xF0, // ###     ####              ###
  0x78, // ###      ####             ###
  0x3E, // ###       #####           ###
  0x1F, // ###        #####         ####
  0x0F, // ###         ######     ##### 
  0x03, // ###           #############  
  0x01, // ###            ###########   
  0x00, // ###               ######     
  0x00, //                              

  0xE0, // ###                 ###      
  0xF8, // #####               ######   
  0xFE, // #######             #######  
  0xFF, // ########               ##### 
  0xEF, // ### #####                ### 
  0xE7, // ###  #####               ####
  0xE1, // ###    ####               ###
  0xE0, // ###     ####              ###
  0xE0, // ###      ####             ###
  0xE0, // ###       #####           ###
  0xE0, // ###        #####         ####
  0xE0, // ###         ######     ##### 
  0xE0, // ###           #############  
  0xE0, // ###            ###########   
  0xE0, // ###               ######     
  0x00, //                              

  // @320 '3' (16 pixels wide)
  0x00, //      ###              ##     
  0xC0, //    #####              ####   
  0xE0, //   ######              #####  
  0xF0, //  #####                  #### 
  0x78, //  ###                     ####
  0x38, // ###                       ###
  0x38, // ###           ###         ###
  0x38, // ###           ###         ###
  0x38, // ###           ###         ###
  0x78, // ####          ####       ####
  0xF0, //  ###         ######     #### 
  0xE0, //  #####     ################  
  0xC0, //   #############  #########   
  0x80, //    ###########     ######    
  0x00, //       ######                 
  0x00, //                              

  0x03, //      ###              ##     
  0x03, //    #####              ####   
  0x03, //   ######              #####  
  0x00, //  #####                  #### 
  0x00, //  ###                     ####
  0x00, // ###                       ###
  0x80, // ###           ###         ###
  0x80, // ###           ###         ###
  0x80, // ###           ###         ###
  0xC0, // ####          ####       ####
  0xE0, //  ###         ######     #### 
  0xFF, //  #####     ################  
  0x7F, //   #############  #########   
  0x1F, //    ###########     ######    
  0x00, //       ######                 
  0x00, //                              

  0x00, //      ###              ##     
  0x00, //    #####              ####   
  0x00, //   ######              #####  
  0x00, //  #####                  #### 
  0x00, //  ###                     ####
  0x00, // ###                       ###
  0x03, // ###           ###         ###
  0x03, // ###           ###         ###
  0x03, // ###           ###         ###
  0x03, // ####          ####       ####
  0x07, //  ###         ######     #### 
  0x1F, //  #####     ################  
  0xFE, //   #############  #########   
  0xFC, //    ###########     ######    
  0xF0, //       ######                 
  0x00, //                              

  0x07, //      ###              ##     
  0x1F, //    #####              ####   
  0x3F, //   ######              #####  
  0x7C, //  #####                  #### 
  0x70, //  ###                     ####
  0xE0, // ###                       ###
  0xE0, // ###           ###         ###
  0xE0, // ###           ###         ###
  0xE0, // ###           ###         ###
  0xF0, // ####          ####       ####
  0x70, //  ###         ######     #### 
  0x7C, //  #####     ################  
  0x3F, //   #############  #########   
  0x1F, //    ###########     ######    
  0x03, //       ######                 
  0x00, //                              

  // @384 '4' (16 pixels wide)
  0x00, //        ####                  
  0x00, //        ######                
  0x00, //        ########              
  0x00, //        ### ######            
  0x00, //        ###   ######          
  0x00, //        ###     #####         
  0x00, //        ###       #####       
  0x00, //        ###        ######     
  0xC0, //        ###          ######   
  0xF0, //        ###            ###### 
  0xF8, // #############################
  0xF8, // #############################
  0xF8, // #############################
  0x00, //        ###                   
  0x00, //        ###                   
  0x00, //        ###                   

  0x00, //        ####                  
  0x00, //        ######                
  0x00, //        ########              
  0x80, //        ### ######            
  0xE0, //        ###   ######          
  0xF0, //        ###     #####         
  0x7C, //        ###       #####       
  0x3F, //        ###        ######     
  0x0F, //        ###          ######   
  0x03, //        ###            ###### 
  0xFF, // #############################
  0xFF, // #############################
  0xFF, // #############################
  0x00, //        ###                   
  0x00, //        ###                   
  0x00, //        ###                   

  0xE0, //        ####                  
  0xF8, //        ######                
  0xFE, //        ########              
  0xDF, //        ### ######            
  0xC7, //        ###   ######          
  0xC1, //        ###     #####         
  0xC0, //        ###       #####       
  0xC0, //        ###        ######     
  0xC0, //        ###          ######   
  0xC0, //        ###            ###### 
  0xFF, // #############################
  0xFF, // #############################
  0xFF, // #############################
  0xC0, //        ###                   
  0xC0, //        ###                   
  0xC0, //        ###                   

  0x01, //        ####                  
  0x01, //        ######                
  0x01, //        ########              
  0x01, //        ### ######            
  0x01, //        ###   ######          
  0x01, //        ###     #####         
  0x01, //        ###       #####       
  0x01, //        ###        ######     
  0x01, //        ###          ######   
  0x01, //        ###            ###### 
  0xFF, // #############################
  0xFF, // #############################
  0xFF, // #############################
  0x01, //        ###                   
  0x01, //        ###                   
  0x01, //        ###                   

  // @448 '5' (16 pixels wide)
  0x00, //      ###      ####           
  0x80, //    #####      ###########    
  0xF8, //   ######      ###############
  0xF8, //  #####         ##############
  0x78, //  ###            ###      ####
  0x38, // ###              ###      ###
  0x38, // ###              ###      ###
  0x38, // ###              ###      ###
  0x38, // ###              ###      ###
  0x38, // ####            ####      ###
  0x38, //  ####          ####       ###
  0x38, //  ######       #####       ###
  0x38, //    ###############        ###
  0x38, //     #############         ###
  0x00, //        #######               
  0x00, //                              

  0xC0, //      ###      ####           
  0xFF, //    #####      ###########    
  0xFF, //   ######      ###############
  0xFF, //  #####         ##############
  0xE0, //  ###            ###      ####
  0x70, // ###              ###      ###
  0x70, // ###              ###      ###
  0x70, // ###              ###      ###
  0x70, // ###              ###      ###
  0xF0, // ####            ####      ###
  0xE0, //  ####          ####       ###
  0xE0, //  ######       #####       ###
  0xC0, //    ###############        ###
  0x80, //     #############         ###
  0x00, //        #######               
  0x00, //                              

  0x03, //      ###      ####           
  0x03, //    #####      ###########    
  0x03, //   ######      ###############
  0x01, //  #####         ##############
  0x00, //  ###            ###      ####
  0x00, // ###              ###      ###
  0x00, // ###              ###      ###
  0x00, // ###              ###      ###
  0x00, // ###              ###      ###
  0x00, // ####            ####      ###
  0x01, //  ####          ####       ###
  0x03, //  ######       #####       ###
  0xFF, //    ###############        ###
  0xFF, //     #############         ###
  0xFC, //        #######               
  0x00, //                              

  0x07, //      ###      ####           
  0x1F, //    #####      ###########    
  0x3F, //   ######      ###############
  0x7C, //  #####         ##############
  0x70, //  ###            ###      ####
  0xE0, // ###              ###      ###
  0xE0, // ###              ###      ###
  0xE0, // ###              ###      ###
  0xE0, // ###              ###      ###
  0xF0, // ####            ####      ###
  0x78, //  ####          ####       ###
  0x7E, //  ######       #####       ###
  0x1F, //    ###############        ###
  0x0F, //     #############         ###
  0x01, //        #######               
  0x00, //                              

  // @512 '6' (16 pixels wide)
  0x00, //         ############         
  0x00, //     ####################     
  0xC0, //    #######################   
  0xE0, //   #####      ####     #####  
  0xF0, //  ####          ###      #### 
  0x70, // ####           ###       ### 
  0x38, // ###             ###       ###
  0x38, // ###             ###       ###
  0x38, // ###             ###       ###
  0x38, // ####           ####       ###
  0x78, //  ####          ###       ####
  0xF0, //  #####       #####      #### 
  0xE0, //   ###############     #####  
  0xC0, //     ############      ####   
  0x00, //       ########        ##     
  0x00, //                              

  0xF0, //         ############         
  0xFF, //     ####################     
  0xFF, //    #######################   
  0x83, //   #####      ####     #####  
  0xC0, //  ####          ###      #### 
  0xC0, // ####           ###       ### 
  0xE0, // ###             ###       ###
  0xE0, // ###             ###       ###
  0xE0, // ###             ###       ###
  0xE0, // ####           ####       ###
  0xC0, //  ####          ###       ####
  0xC0, //  #####       #####      #### 
  0x83, //   ###############     #####  
  0x03, //     ############      ####   
  0x03, //       ########        ##     
  0x00, //                              

  0xFF, //         ############         
  0xFF, //     ####################     
  0xFF, //    #######################   
  0x07, //   #####      ####     #####  
  0x01, //  ####          ###      #### 
  0x01, // ####           ###       ### 
  0x00, // ###             ###       ###
  0x00, // ###             ###       ###
  0x00, // ###             ###       ###
  0x01, // ####           ####       ###
  0x01, //  ####          ###       ####
  0x07, //  #####       #####      #### 
  0xFF, //   ###############     #####  
  0xFF, //     ############      ####   
  0xFC, //       ########        ##     
  0x00, //                              

  0x00, //         ############         
  0x0F, //     ####################     
  0x1F, //    #######################   
  0x3E, //   #####      ####     #####  
  0x78, //  ####          ###      #### 
  0xF0, // ####           ###       ### 
  0xE0, // ###             ###       ###
  0xE0, // ###             ###       ###
  0xE0, // ###             ###       ###
  0xF0, // ####           ####       ###
  0x78, //  ####          ###       ####
  0x7C, //  #####       #####      #### 
  0x3F, //   ###############     #####  
  0x0F, //     ############      ####   
  0x03, //       ########        ##     
  0x00, //                              

  // @576 '7' (16 pixels wide)
  0x38, //                           ###
  0x38, //                           ###
  0x38, //                           ###
  0x38, // #####                     ###
  0x38, // ##########                ###
  0x38, // #############             ###
  0x38, //      ###########          ###
  0x38, //           ########        ###
  0x38, //              #######      ###
  0x38, //                 ######    ###
  0x38, //                   ######  ###
  0xB8, //                     ##### ###
  0xF8, //                       #######
  0xF8, //                         #####
  0x38, //                           ###
  0x00, //                              

  0x00, //                           ###
  0x00, //                           ###
  0x00, //                           ###
  0x00, // #####                     ###
  0x00, // ##########                ###
  0x00, // #############             ###
  0x00, //      ###########          ###
  0xC0, //           ########        ###
  0xF0, //              #######      ###
  0xFC, //                 ######    ###
  0x3F, //                   ######  ###
  0x0F, //                     ##### ###
  0x03, //                       #######
  0x00, //                         #####
  0x00, //                           ###
  0x00, //                              

  0x00, //                           ###
  0x00, //                           ###
  0x00, //                           ###
  0x00, // #####                     ###
  0xC0, // ##########                ###
  0xF8, // #############             ###
  0xFF, //      ###########          ###
  0x3F, //           ########        ###
  0x07, //              #######      ###
  0x00, //                 ######    ###
  0x00, //                   ######  ###
  0x00, //                     ##### ###
  0x00, //                       #######
  0x00, //                         #####
  0x00, //                           ###
  0x00, //                              

  0x00, //                           ###
  0x00, //                           ###
  0x00, //                           ###
  0xF8, // #####                     ###
  0xFF, // ##########                ###
  0xFF, // #############             ###
  0x07, //      ###########          ###
  0x00, //           ########        ###
  0x00, //              #######      ###
  0x00, //                 ######    ###
  0x00, //                   ######  ###
  0x00, //                     ##### ###
  0x00, //                       #######
  0x00, //                         #####
  0x00, //                           ###
  0x00, //                              

  // @640 '8' (16 pixels wide)
  0x00, //      #######                 
  0x80, //    ###########     ######    
  0xE0, //   #############  ##########  
  0xF0, //  #####     #### ############ 
  0xF0, //  ###         ######     #### 
  0x78, // ####          ####       ####
  0x38, // ###           ###         ###
  0x38, // ###           ###         ###
  0x38, // ###           ###         ###
  0x78, // ###          #####       ####
  0xF0, //  ###         ######     #### 
  0xF0, //  #####     #### ############ 
  0xC0, //   ############   #########   
  0x80, //    ##########      ######    
  0x00, //      #######                 
  0x00, //                              

  0x00, //      #######                 
  0x1F, //    ###########     ######    
  0x7F, //   #############  ##########  
  0xFF, //  #####     #### ############ 
  0xE0, //  ###         ######     #### 
  0xC0, // ####          ####       ####
  0x80, // ###           ###         ###
  0x80, // ###           ###         ###
  0x80, // ###           ###         ###
  0xC0, // ###          #####       ####
  0xE0, //  ###         ######     #### 
  0xFF, //  #####     #### ############ 
  0x7F, //   ############   #########   
  0x1F, //    ##########      ######    
  0x00, //      #######                 
  0x00, //                              

  0xF0, //      #######                 
  0xFC, //    ###########     ######    
  0xFE, //   #############  ##########  
  0x1E, //  #####     #### ############ 
  0x07, //  ###         ######     #### 
  0x03, // ####          ####       ####
  0x03, // ###           ###         ###
  0x03, // ###           ###         ###
  0x03, // ###           ###         ###
  0x07, // ###          #####       ####
  0x07, //  ###         ######     #### 
  0x1E, //  #####     #### ############ 
  0xFC, //   ############   #########   
  0xF8, //    ##########      ######    
  0xF0, //      #######                 
  0x00, //                              

  0x07, //      #######                 
  0x1F, //    ###########     ######    
  0x3F, //   #############  ##########  
  0x7C, //  #####     #### ############ 
  0x70, //  ###         ######     #### 
  0xF0, // ####          ####       ####
  0xE0, // ###           ###         ###
  0xE0, // ###           ###         ###
  0xE0, // ###           ###         ###
  0xE0, // ###          #####       ####
  0x70, //  ###         ######     #### 
  0x7C, //  #####     #### ############ 
  0x3F, //   ############   #########   
  0x1F, //    ##########      ######    
  0x07, //      #######                 
  0x00, //                              

  // @704 '9' (16 pixels wide)
  0x00, //      ##         #######      
  0xC0, //    ####      #############   
  0xE0, //  ######     ###############  
  0xF0, //  ####      #####      ###### 
  0xF0, // ####       ###          #### 
  0x78, // ###       ####           ####
  0x38, // ###       ###             ###
  0x38, // ###       ###             ###
  0x38, // ###       ###             ###
  0x78, //  ###       ###           ####
  0xF0, //  ####      ###          #### 
  0xE0, //   #####     ####       ####  
  0xC0, //    #######################   
  0x80, //      ####################    
  0x00, //          ############        
  0x00, //                              

  0xFE, //      ##         #######      
  0xFF, //    ####      #############   
  0xFF, //  ######     ###############  
  0x03, //  ####      #####      ###### 
  0x00, // ####       ###          #### 
  0x00, // ###       ####           ####
  0x00, // ###       ###             ###
  0x00, // ###       ###             ###
  0x00, // ###       ###             ###
  0x00, //  ###       ###           ####
  0x00, //  ####      ###          #### 
  0x01, //   #####     ####       ####  
  0xFF, //    #######################   
  0xFF, //      ####################    
  0xF8, //          ############        
  0x00, //                              

  0x00, //      ##         #######      
  0x07, //    ####      #############   
  0x0F, //  ######     ###############  
  0x1F, //  ####      #####      ###### 
  0x1C, // ####       ###          #### 
  0x3C, // ###       ####           ####
  0x38, // ###       ###             ###
  0x38, // ###       ###             ###
  0x38, // ###       ###             ###
  0x1C, //  ###       ###           ####
  0x1C, //  ####      ###          #### 
  0x0F, //   #####     ####       ####  
  0xFF, //    #######################   
  0xFF, //      ####################    
  0x7F, //          ############        
  0x00, //                              

  0x06, //      ##         #######      
  0x1E, //    ####      #############   
  0x7E, //  ######     ###############  
  0x78, //  ####      #####      ###### 
  0xF0, // ####       ###          #### 
  0xE0, // ###       ####           ####
  0xE0, // ###       ###             ###
  0xE0, // ###       ###             ###
  0xE0, // ###       ###             ###
  0x70, //  ###       ###           ####
  0x78, //  ####      ###          #### 
  0x3E, //   #####     ####       ####  
  0x1F, //    #######################   
  0x07, //      ####################    
  0x00, //          ############        
  0x00, //                              

  // @768 ':' (16 pixels wide)
  0x00, //                      
  0x00, //                      
  0x00, //                      
  0x00, //                      
  0x00, //                      
  0x00, //                      
  0x38, // ###               ###
  0x38, // ###               ###
  0x38, // ###               ###
  0x00, //                      
  0x00, //                      
  0x00, //                      
  0x00, //                      
  0x00, //                      
  0x00, //                      
  0x00, //             

  0x00, //                      
  0x00, //                      
  0x00, //                      
  0x00, //                      
  0x00, //                      
  0x00, //                      
  0x00, // ###               ###
  0x00, // ###               ###
  0x00, // ###               ###
  0x00, //                      
  0x00, //                      
  0x00, //                      
  0x00, //                      
  0x00, //                      
  0x00, //                      
  0x00, //             

  0x00, //                      
  0x00, //                      
  0x00, //                      
  0x00, //                      
  0x00, //                      
  0x00, //                      
  0xE0, // ###               ###
  0xE0, // ###               ###
  0xE0, // ###               ###
  0x00, //                      
  0x00, //                      
  0x00, //                      
  0x00, //                      
  0x00, //                      
  0x00, //                      
  0x00, //             

  0x00, //                      
  0x00, //                      
  0x00, //                      
  0x00, //                      
  0x00, //                      
  0x00, //                      
  0x00, // ###               ###
  0x00, // ###               ###
  0x00, // ###               ###
  0x00, //                      
  0x00, //                      
  0x00, //                      
  0x00, //                      
  0x00, //                      
  0x00, //                      
  0x00  //             
};

const DCfont TinyOLED4kfont16x32Digits = {
  (uint8_t *)ssd1306xled_font16x32_digits,
  16, // character width in pixels
  4, // character height in pages (8 pixels)
  0x2C,0x3A // ASCII extents
  };

// for backwards compatibility
#define FONT16X32DIGITS (&TinyOLED4kfont16x32Digits)

const DCfont *MyFont=FONT16X32DIGITS;

//                  +-\/-+
// Ain0 (D 5) PB5  1|    |8  Vcc
// Ain3 (D 3) PB3  2|    |7  PB2 (D 2) Ain1
// Ain2 (D 4) PB4  3|    |6  PB1 (D 1) pwm1
//            GND  4|    |5  PB0 (D 0) pwm0
//                  +----+
//    SCK---> PB2
//    SDA---> PB0

const byte SWITCH = 4; // pin 3 / PCINT4
const byte PIN_TEMP = 1; // pin 1 / PB1
int inp;
float val;
float fin;
char buf[5];
byte keep_ADCSRA; 

ISR (PCINT0_vect) 
 {

 }
void setup ()
  {
  analogReference(INTERNAL); //1.1V
  oled.begin(128, 32, sizeof(tiny4koled_init_128x32br), tiny4koled_init_128x32br); 
  oled.clear();
  pinMode(PIN_TEMP, OUTPUT);
  pinMode (SWITCH, INPUT);
  digitalWrite (SWITCH, HIGH);  // internal pull-up
  // pin change interrupt (for D4)
  PCMSK  |= bit (PCINT4);  // want pin D4 / pin 3
  GIFR   |= bit (PCIF);    // clear any outstanding interrupts
  GIMSK  |= bit (PCIE);    // enable pin change interrupts 
  
  }  // end of setup

void loop ()
  {
  oled.clear();   
  oled.on();  
  Visualizza();
  delay(3000);
  oled.clear();  
  oled.off();
  goToSleep ();
  }  // end of loop
  
  
void goToSleep ()
  {
  keep_ADCSRA = ADCSRA;  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  ADCSRA = 0;            // turn off ADC
  power_all_disable ();  // power off ADC, Timer 0 and 1, serial interface
  sleep_enable();
  sleep_cpu();
                               
  sleep_disable();
  power_all_enable();    // power everything back on
  ADCSRA = keep_ADCSRA;
  }  // end of goToSleep 

void Visualizza() {
 
  digitalWrite(PIN_TEMP,HIGH);
  delay(1000);

  oled.setFont(MyFont);
  oled.setCursor(45, 0);
  inp=analogRead(A3);
  val=inp*(1.1/1024);
  fin=(val-0.5)*100;
  dtostrf(fin, 3, 1, buf );
  oled.print(buf);
  
  oled.setFont(FONT8X16P);
  oled.setCursor(0, 1);
  oled.print("TEMP:");
  oled.setCursor(118, 0);
  oled.print("c");
/*
  oled.setFont(MyFont);
  oled.setCursor(50, 0);
  val=analogRead(A3)*(3.30/1024);
  val=(val-0.5)*100;
  dtostrf(val, 3, 1, buf );
  oled.print(buf);
*/


  digitalWrite(PIN_TEMP,LOW);
  

/*
  oled.setFont(FONT8X16P);
  oled.setCursor(103, 0);
  oled.print(".");
*/
}

  
