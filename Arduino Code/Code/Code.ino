
#include "<mavlink library location > mavlink.h"
#include <SoftwareSerial.h>
SoftwareSerial SSerial(8, 7); // RX, TX
#include <Pixy2.h>

// Arduino MAVLink test code
Pixy2 pixy;
float mtx,mty,tx,ty;

float uyl,dyl,lxl,rxl;

float vx ,vy;
float px,py;
void setup() {
        Serial.begin(57600);
         SSerial.begin(57600);        
pixy.init();
uyl=52;
dyl=-52;
lxl=-79;
rxl=79;
  vx=0;
vy=0;
}

void loop() 
{
  Serial.print("  mx: ");
  Serial.print(mtx);
  Serial.print("  my: ");
  Serial.print(mty);
  
      Serial.print("  px: ");
  Serial.print(px);
  Serial.print("  py: ");
  Serial.print(py);
  
   callpixy();



//px=0.0;
//py=0.0;
        /* The default UART header for your MCU */ 
     int sysid = 20;                   ///< ID 20 for this airplane
    int compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
    int type = MAV_TYPE_QUADROTOR;   ///< This system is an airplane / fixed wing
 
// Define the system type, in this quadrotor
    uint8_t system_type = MAV_TYPE_QUADROTOR;
   // uint8_t autopilot_type = MAV_AUTOPILOT_PIXHAWK;
 
    uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
    uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
    uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight
    // Initialize the required buffers
    mavlink_message_t msg;
    mavlink_message_t msg1;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
// Pack the message
//mavlink_msg_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status)
    mavlink_msg_heartbeat_pack(sysid,compid, &msg, type, system_type, system_mode, custom_mode, system_state);

  // mavlink_msg_set_position_target_local_ned_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask, float x, float y, float z, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate)
 
  //bit1:PosX, bit2:PosY, bit3:PosZ, bit4:VelX, bit5:VelY, bit6:VelZ, bit7:AccX, bit8:AccY, bit9:AccZ, bit11:yaw, bit12:yaw rate
 uint16_t typemask=0b110111000000;
//coordinate_frame= MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9*/
uint8_t coordinateframe=9;
  mavlink_msg_set_position_target_local_ned_pack(sysid,compid, &msg1,0, 0, 0, coordinateframe,typemask, px, py, 0, vx, vy, 0, 0, 0,0, 0, 0);

// Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
     delay(100);
     SSerial.write(buf, len);
     uint16_t len1 = mavlink_msg_to_send_buffer(buf, &msg1);
     SSerial.write(buf, len1);
 
// Send the message with the standard UART send function
// uart0_send might be named differently depending on
// the individual microcontroller / library in use.
    
      
         Serial.println();
         }
       
       
       
void callpixy()
       {
    pixy.ccc.getBlocks();
  if (pixy.ccc.numBlocks)
  {
    //Serial.print("Detected ");
    //Serial.println(pixy.ccc.);
  tx= pixy.ccc.blocks[0].m_x; //0-316
  ty= pixy.ccc.blocks[0].m_y; //0-208
  Serial.print("x: ");
  Serial.print(tx);
  Serial.print("  y: ");
  Serial.println(ty);
//pixy.ccc.blocks[i].print();

mty=-1*(map(tx,0,316,-158,158));
mtx=-1*(map(ty,0,208,-104,104));
 Serial.print("  mx: ");
  Serial.print(mtx);
  Serial.print("  my: ");
  Serial.print(mty);

if(mty<-110)
{py=0.5;
vy=1;
}
else if(mty>110)
{
  py=-0.5;
  vy=-1;
}
else
{
py=0;
vy=0;
}
if(mtx<-73)
{
px=-0.5;
vx=-1;
}
else if(mtx>73)
{
  px=0.5;
  vx=1;
}
else
{8
px=0;
vx=0;
}
  
}
else
{px=0;
py=0;
vx=0;
vy=0;
  }
}
