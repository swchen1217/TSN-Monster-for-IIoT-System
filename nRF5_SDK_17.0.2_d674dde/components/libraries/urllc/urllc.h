#ifndef __URLLC_H
#define __URLLC_H

#define     URLLC_DATA_PKT                      0           
#define     SYNC_PKT                            1
#define     CH_PKT                              2 

#define     URLLC_PAYLOAD_LENGTH                32
typedef struct
{
    uint8_t message_id;    //recogonize sync_pkt or urllc_payload 
   
} urllc_header;

typedef struct{
  urllc_header header;         
  uint32_t ch_tick;        // tick target to start channel hopping 
  //uint8_t  ch_idx;          
} ch_pkt;

// typedef struct {

//     urllc_header header;
//     uint8_t chn_map[CHANNEL_MAP_SIZE];
//     uint32_t ch_target;

// }chn_map_update_req;

typedef struct
{
    urllc_header header;
    int32_t  timer_val;
    uint32_t counter_val;
   
} sync_pkt_t;

typedef struct
{
    urllc_header header;
    int32_t seq_num;
    uint32_t time_stamp;                            //time stamp to evaluate latency
    uint8_t data[URLLC_PAYLOAD_LENGTH];
} urllc_payload;

#endif // _URLLC_H