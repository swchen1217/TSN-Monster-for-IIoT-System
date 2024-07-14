#ifndef __URLLC_H
#define __URLLC_H

#define     URLLC_DATA_PKT                      0           
#define     SYNC_PKT                            1
#define     CH_PKT                              2 

#define     URLLC_PAYLOAD_LENGTH                32

#define SYNC_INTERVAL     500 //(ms)

// @MWNL TLV Begin
#define TLV_HEADER_LEN 2 // cdc acm should read two bytes first,type and length each with one byte
#define TLV_TYPE_INDEX 0
#define TLV_LENGTH_INDEX 1

// TLV Types
#define CDC_ACM_DATA 0
#define CDC_ACM_CHN_MAP_UPDATE 1
#define CDC_ACM_TS_1 11
#define CDC_ACM_TS_2 12
#define CDC_ACM_CHN_SET 2
// @MWNL TLV End

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
    int32_t  rtc_val;
} sync_pkt_t;

typedef struct
{
    urllc_header header;
    int32_t seq_num;
    uint32_t time_stamp;                            //time stamp to evaluate latency
    uint8_t data[URLLC_PAYLOAD_LENGTH];
} urllc_payload;

#endif // _URLLC_H